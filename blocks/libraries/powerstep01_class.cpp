/**
 ******************************************************************************
 * @file    powerstep01_class.cpp
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    March 18th, 2016
 * @brief   Powerstep01 motor driver (Microstepping controller with power MOSFETs)
 * @note    (C) COPYRIGHT 2016 STMicroelectronics
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "powerstep01_class.h"

/* Definitions ---------------------------------------------------------------*/

/* Error of bad SPI transaction. */
#define POWERSTEP01_ERROR_1        (POWERSTEP01_ERROR_BASE|0x0001)

/* Variables  ----------------------------------------------------------------*/

/* Number of devices. */
uint8_t POWERSTEP01::numberOfDevices = 0;

/* ISR flags used to restart an interrupted SPI transfer when an error is reported. */
bool POWERSTEP01::spiPreemptionByIsr = FALSE;
bool POWERSTEP01::isrFlag = FALSE;

/* SPI Transmission for Daisy-Chain Configuration. */
uint8_t POWERSTEP01::spiTxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
uint8_t POWERSTEP01::spiRxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];


/* Methods -------------------------------------------------------------------*/

/**********************************************************
 * @brief Starts the Powerstep01 library
 * @param[in] pInit pointer to the initialization data
 * @retval COMPONENT_OK in case of success.
 **********************************************************/
Status_t POWERSTEP01::Powerstep01_Init(void* pInit)
{ 
  
  /* configure the step clock */
  Powerstep01_Board_StepClockInit();
  
  /* Standby-reset deactivation */
  Powerstep01_Board_ReleaseReset();
  
  /* Let a delay after reset */
  Powerstep01_Board_Delay(1);

  if (pInit == 0)
  {
    // Set all registers to their predefined values 
    // from powerstep01_target_config.h 
    Powerstep01_SetRegisterToPredefinedValues();
  }
  else
  {
    Powerstep01_SetDeviceParamsToGivenValues((powerstep01_Init_u_t*) pInit);
  }
  
  // Put the Powerstep01 in HiZ state
  Powerstep01_CmdHardHiZ();
  
  Powerstep01_FetchAndClearAllStatus();

  return COMPONENT_OK;
}

/**********************************************************
 * @brief Read id
 * @param[in] id pointer to the identifier to be read.
 * @retval COMPONENT_OK in case of success.
 **********************************************************/
Status_t POWERSTEP01::Powerstep01_ReadID(uint8_t *id)
{
  *id = deviceInstance;

  return COMPONENT_OK;
}

/**********************************************************
 * @brief  Attaches a user callback to the error Handler.
 * The call back will be then called each time the library 
 * detects an error
 * @param[in] callback Name of the callback to attach 
 * to the error Hanlder
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_AttachErrorHandler(void (*callback)(uint16_t error))
{
  errorHandlerCallback = (void (*)(uint16_t error)) callback;
}

/**********************************************************
 * @brief  Issues the GetStatus command to the Powerstep01 device
 * @retval Status Register value
 * @note Once the GetStatus command is performed, the flags of the
 * status register are reset. 
 * This is not the case when the status register is read with the
 * GetParam command (via the functions Powerstep01_ReadStatusRegister
 * or Powerstep01_CmdGetParam).
 **********************************************************/
uint16_t POWERSTEP01::Powerstep01_CmdGetStatus(void)
{
  uint16_t status = 0;
  uint32_t loop;
  uint8_t spiIndex = numberOfDevices - deviceInstance - 1;
  bool itDisable = FALSE;  
 
  do
  {
    spiPreemptionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable Powerstep01_Board_EnableIrq if disable in previous iteration */
      Powerstep01_Board_EnableIrq();
      itDisable = FALSE;
    }    
    for (loop = 0; loop < numberOfDevices; loop++)
    {
       spiTxBursts[0][loop] = POWERSTEP01_NOP;
       spiTxBursts[1][loop] = POWERSTEP01_NOP;
       spiTxBursts[2][loop] = POWERSTEP01_NOP;
       spiTxBursts[3][loop] = POWERSTEP01_NOP;
       spiRxBursts[0][loop] = 0;
       spiRxBursts[1][loop] = 0;
       spiRxBursts[2][loop] = 0;
       spiRxBursts[3][loop] = 0;       
    }
    spiTxBursts[0][spiIndex] = POWERSTEP01_GET_STATUS;
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    Powerstep01_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemptionByIsr); // check pre-emption by ISR  
  for (loop = 0; loop < POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS + POWERSTEP01_RSP_NB_BYTES_GET_STATUS; loop++)
  {
     Powerstep01_WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
  status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  /* re-enable Powerstep01_Board_EnableIrq after SPI transfers*/
  Powerstep01_Board_EnableIrq();  

  return (status);
}

/**********************************************************
 * @brief  Requests the motor to move to the home position (ABS_POSITION = 0)
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdGoHome(void)
{
  Powerstep01_SendCommand(POWERSTEP01_GO_HOME, 0);
} 

/**********************************************************
 * @brief  Requests the motor to move to the mark position 
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdGoMark(void)
{
    Powerstep01_SendCommand(POWERSTEP01_GO_MARK, 0); 
}

/**********************************************************
 * @brief  Requests the motor to move to the specified position 
 * @param[in] targetPosition absolute position in steps
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdGoTo(int32_t targetPosition)
{
  Powerstep01_SendCommand(POWERSTEP01_GO_TO, targetPosition);
}

/******************************************************//**
 * @brief Issues PowerStep01 Go To Dir command
 * @param[in] direction movement direction
 * @param[in] abs_pos absolute position where requested to move
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdGoToDir(motorDir_t direction,
                                         int32_t abs_pos)
{
  Powerstep01_SendCommand((uint8_t)POWERSTEP01_GO_TO_DIR|
                          (uint8_t)direction, abs_pos);  
}

/******************************************************//**
 * @brief Issues PowerStep01 Go Until command
 * @param[in] action ACTION_RESET or ACTION_COPY
 * @param[in] direction movement direction
 * @param[in] speed in steps/tick
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdGoUntil(motorAction_t action,
                            motorDir_t direction,
                            uint32_t speed)
{
  Powerstep01_SendCommand(
    (uint8_t)POWERSTEP01_GO_UNTIL|(uint8_t)action|(uint8_t)direction,
    speed);
}

/**********************************************************
 * @brief Immediatly stops the motor and disable the power bridge
 * @retval None
 * @note The HardHiZ command immediately disables the power bridges
 * (high impedance state) and raises the HiZ flag. 
 * When the motor is stopped, a HardHiZ command forces the bridges 
 * to enter high impedance state.
 * This command can be given anytime and is immediately executed.
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdHardHiZ(void)
{
  Powerstep01_SendCommand(POWERSTEP01_HARD_HIZ, 0);    
}

/**********************************************************
 * @brief  Immediatly stops the motor and disable the power bridge
 * @retval None
 * @note The HardStop command causes an immediate motor stop with
 * infinite deceleration.
 * When the motor is in high impedance state, a HardStop command
 * forces the bridges to exit high impedance state; no motion is performed.
 * This command can be given anytime and is immediately executed.
 * This command keeps the BUSY flag low until the motor is stopped.
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdHardStop(void) 
{
  Powerstep01_SendCommand(POWERSTEP01_HARD_STOP, 0);
}

/**********************************************************
 * @brief  Moves the motor of the specified number of steps
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] stepCount Number of steps to perform
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdMove(motorDir_t direction,
                                      uint32_t stepCount)
{
  Powerstep01_SendCommand((uint8_t)POWERSTEP01_MOVE|(uint8_t)direction,
    stepCount);
}

/**********************************************************
 * @brief  Issues the Nop command to the Powerstep01 device
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdNop(void)
{
  Powerstep01_SendCommand(POWERSTEP01_NOP, 0);
}

/******************************************************//**
 * @brief Issues PowerStep01 Release SW command
 * @param[in] action type of action to undertake when the SW
 * input is forced high
 * @param[in] direction movement direction
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdReleaseSw(motorAction_t action,
                                           motorDir_t direction)
{
   Powerstep01_SendCommand((uint8_t)POWERSTEP01_RELEASE_SW|
                           (uint8_t)action|
                           (uint8_t)direction,
                           0);
}

/******************************************************//**
 * @brief Issues PowerStep01 Reset Device command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdResetDevice(void)
{
  Powerstep01_SendCommand(POWERSTEP01_RESET_DEVICE, 0);         
}

/******************************************************//**
 * @brief Issues PowerStep01 Reset Pos command
 * @param[in] deviceId (from 0 to MAX_NUMBER_OF_DEVICES-1 )
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdResetPos(void)
{
  Powerstep01_SendCommand(POWERSTEP01_RESET_POS, 0);       
}

/**********************************************************
 * @brief  Runs the motor. It will accelerate from the min 
 * speed up to the max speed by using the device acceleration.
 * @param[in] direction FORWARD or BACKWARD
 * @param[in] speed in steps/s
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdRun(motorDir_t direction, uint32_t speed)
{
  Powerstep01_SendCommand((uint8_t)POWERSTEP01_RUN|(uint8_t)direction, speed);
}

/**********************************************************
 * @brief Stops the motor by using the device deceleration
 * and disables the power bridges
 * @retval None
 * @note The SoftHiZ command disables the power bridges
 * (high impedance state) after a deceleration to zero.
 * The deceleration value used is the one stored in the DEC register.
 * When bridges are disabled, the HiZ flag is raised.
 * When the motor is stopped, a SoftHiZ command forces the bridges
 * to enter high impedance state.
 * This command can be given anytime and is immediately executed.
 * This command keeps the BUSY flag low until the motor is stopped.
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdSoftHiZ(void)
{
  Powerstep01_SendCommand(POWERSTEP01_SOFT_HIZ, 0);           
}

/**********************************************************
 * @brief  Stops the motor by using the device deceleration
 * @retval None
 * @note The SoftStop command causes an immediate deceleration
 * to zero speed and a consequent motor stop.
 * The deceleration value used is the one stored in the DEC register.
 * When the motor is in high impedance state, a SoftStop
 * command forces the bridges to exit from high impedance state.
 * No motion is performed.
 * This command can be given anytime and is immediately executed.
 * This command keeps the BUSY flag low until the motor is stopped.
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdSoftStop(void)
{   
  Powerstep01_SendCommand(POWERSTEP01_SOFT_STOP, 0);
}

/******************************************************//**
 * @brief Issues PowerStep01 Step Clock command
 * @param[in] direction Movement direction (FORWARD, BACKWARD)
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_CmdStepClock(motorDir_t direction)
{
  Powerstep01_SendCommand((uint8_t)POWERSTEP01_STEP_CLOCK|(uint8_t)direction,
                          0);  
}

/**********************************************************
 * @brief Error handler which calls the user callback (if defined)
 * @param[in] error Number of the error
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_ErrorHandler(uint16_t error)
{
  if (errorHandlerCallback != 0)
  {
    (void) errorHandlerCallback(error);
  }
  else   
  {
    /* Aborting the program. */
    exit(EXIT_FAILURE);
  }
}

/******************************************************//**
 * @brief Fetch and clear status flags of all devices 
 * by issuing a GET_STATUS command simultaneously  
 * to all devices.
 * Then, the fetched status of each device can be retrieved
 * by using the Powerstep01_GetFetchedStatus function
 * provided there is no other calls to functions which 
 * use the SPI in between.
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_FetchAndClearAllStatus(void)
{
  uint8_t loop;

  for (loop = 0; loop < numberOfDevices; loop++)
  {
     spiTxBursts[0][loop] = POWERSTEP01_GET_STATUS;
     spiTxBursts[1][loop] = POWERSTEP01_NOP;
     spiTxBursts[2][loop] = POWERSTEP01_NOP;
     spiTxBursts[3][loop] = POWERSTEP01_NOP;
     spiRxBursts[0][loop] = 0;
     spiRxBursts[1][loop] = 0;
     spiRxBursts[2][loop] = 0;
     spiRxBursts[3][loop] = 0;
  }
  for (loop = 0; 
       loop < POWERSTEP01_CMD_ARG_NB_BYTES_GET_STATUS + 
              POWERSTEP01_RSP_NB_BYTES_GET_STATUS; 
       loop++)
  {
     Powerstep01_WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
}

/******************************************************//**
 * @brief Get the value of the STATUS register which was 
 * fetched by using Powerstep01_FetchAndClearAllStatus.
 * The fetched values are available  as long as there
 * no other calls to functions which use the SPI.
 * @retval Last fetched value of the STATUS register
 *********************************************************/
uint16_t POWERSTEP01::Powerstep01_GetFetchedStatus(void)
{
  uint16_t status = 0;
  if (numberOfDevices > deviceInstance)
  {
    uint8_t spiIndex = numberOfDevices - deviceInstance - 1;
    status = (spiRxBursts[1][spiIndex] << 8) | (spiRxBursts[2][spiIndex]);
  }
  return (status);
}

/**********************************************************
 * @brief Returns the FW version of the library
 * @retval POWERSTEP01_FW_VERSION
 **********************************************************/
uint32_t POWERSTEP01::Powerstep01_GetFwVersion(void)
{
  return (POWERSTEP01_FW_VERSION);
}

/**********************************************************
 * @brief  Returns the mark position  device
 * @retval Mark register value converted in a 32b signed integer 
 **********************************************************/
int32_t POWERSTEP01::Powerstep01_GetMark(void)
{
  return Powerstep01_ConvertPosition(Powerstep01_CmdGetParam(POWERSTEP01_MARK));
}

/**********************************************************
 * @brief  Returns the ABS_POSITION device
 * @retval ABS_POSITION register value converted in a 32b signed integer
 **********************************************************/
int32_t POWERSTEP01::Powerstep01_GetPosition(void)
{
  return Powerstep01_ConvertPosition(
    Powerstep01_CmdGetParam(POWERSTEP01_ABS_POS));
}

/**********************************************************
 * @brief Checks if the device is busy
 * by reading the Busy flag bit of its status Register
 * This operation clears the status register
 * @retval true if device is busy, false zero
 *********************************************************/
bool POWERSTEP01::Powerstep01_IsDeviceBusy(void)
{
  if(!(Powerstep01_CmdGetStatus() & POWERSTEP01_STATUS_BUSY)) 
  {
    return TRUE;
  }
  else 
  {
    return FALSE;
  }
}

/**********************************************************
 * @brief  Reads the Status Register value
 * @retval Status register value
 * @note The status register flags are not cleared 
 * at the difference with Powerstep01_CmdGetStatus()
 **********************************************************/
uint16_t POWERSTEP01::Powerstep01_ReadStatusRegister(void)
{
  return (Powerstep01_CmdGetParam(POWERSTEP01_STATUS));
}

/**********************************************************
 * @brief  Set the stepping mode 
 * @param[in] stepMode from full step to 1/128 microstep
 * as specified in enum motorStepMode_t
 * @retval None
 **********************************************************/
bool POWERSTEP01::Powerstep01_SelectStepMode(motorStepMode_t stepMode)
{
  uint8_t stepModeRegister;
  powerstep01_StepSel_t powerstep01StepMode;

  switch (stepMode)
  {
    case STEP_MODE_FULL:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1;
      break;
    case STEP_MODE_HALF:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_2;
      break;    
    case STEP_MODE_1_4:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_4;
      break;        
    case STEP_MODE_1_8:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_8;
      break;
    case STEP_MODE_1_16:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_16;
      break;        
    case STEP_MODE_1_32:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_32;
      break;
    case STEP_MODE_1_64:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_64;
      break;
    case STEP_MODE_1_128:
      powerstep01StepMode = POWERSTEP01_STEP_SEL_1_128;
      break;
    default:
      return false;
  }
  
  /* Set the powerstep01 in HiZ state */
  Powerstep01_CmdHardHiZ();  
  
  /* Read Step mode register and clear STEP_SEL field */
  stepModeRegister = (uint8_t)(0xF8 & Powerstep01_CmdGetParam(POWERSTEP01_STEP_MODE)) ;
  
  /* Apply new step mode */
  Powerstep01_CmdSetParam(POWERSTEP01_STEP_MODE, stepModeRegister | (uint8_t)powerstep01StepMode);

  /* Reset abs pos register */
  Powerstep01_CmdResetPos();

  return true;
}

/**********************************************************
 * @brief  Set current position to be the Home position (ABS pos set to 0)
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_SetHome(void)
{
  Powerstep01_CmdSetParam(POWERSTEP01_ABS_POS, 0);
}

/**********************************************************
 * @brief  Sets current position to be the Mark position 
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_SetMark(void)
{
  Powerstep01_CmdSetParam(POWERSTEP01_MARK,
    Powerstep01_CmdGetParam(POWERSTEP01_ABS_POS));
}                                                  

/**********************************************************
 * @brief  Locks until the device state becomes Inactive
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_WaitWhileActive(void)
{
  /* Wait while motor is running */
  while (Powerstep01_IsDeviceBusy() != 0);
}

/**
 * @brief To and from register parameter conversion functions
 */

/**********************************************************
 * @brief  Converts the ABS_POSITION register value to a 32b signed integer
 * @param[in] abs_position_reg value of the ABS_POSITION register
 * @retval operation_result 32b signed integer corresponding to the absolute position 
 **********************************************************/
int32_t POWERSTEP01::Powerstep01_ConvertPosition(uint32_t abs_position_reg)
{
  int32_t operation_result;

  if (abs_position_reg & POWERSTEP01_ABS_POS_SIGN_BIT_MASK) 
  {
    /* Negative register value */
    abs_position_reg = ~abs_position_reg;
    abs_position_reg += 1;

    operation_result = (int32_t) (abs_position_reg & POWERSTEP01_ABS_POS_VALUE_MASK);
    operation_result = -operation_result;
  } 
  else 
  {
    operation_result = (int32_t) abs_position_reg;
  }
  return operation_result;
}

/**
 * @brief Functions to initialize the registers
 */

/**********************************************************
 * @brief Set the parameters of the device to values of initPrm structure
 * @param[in] initPrm structure containing values to initialize the device
 * parameters
 * @retval None.
 **********************************************************/
void POWERSTEP01::Powerstep01_SetDeviceParamsToGivenValues(
  powerstep01_Init_u_t *initPrm)
{
  Powerstep01_CmdSetParam(POWERSTEP01_ABS_POS, 0);
  Powerstep01_CmdSetParam(POWERSTEP01_EL_POS, 0);
  Powerstep01_CmdSetParam(POWERSTEP01_MARK, 0);
  Powerstep01_CmdSetParam(POWERSTEP01_ACC,
    AccDec_Steps_s2_to_RegVal(initPrm->cm.cp.acceleration));
  Powerstep01_CmdSetParam(POWERSTEP01_DEC,
    AccDec_Steps_s2_to_RegVal(initPrm->cm.cp.deceleration));
  Powerstep01_CmdSetParam(POWERSTEP01_MAX_SPEED,
    MaxSpd_Steps_s_to_RegVal(initPrm->cm.cp.maxSpeed));
  Powerstep01_CmdSetParam(POWERSTEP01_MIN_SPEED,
    initPrm->cm.cp.lowSpeedOptimization|
    MaxSpd_Steps_s_to_RegVal(initPrm->cm.cp.minSpeed));
  Powerstep01_CmdSetParam(POWERSTEP01_FS_SPD,
    initPrm->cm.cp.boostMode|
    FSSpd_Steps_s_to_RegVal(initPrm->cm.cp.fullStepSpeed));
  Powerstep01_CmdSetParam(POWERSTEP01_OCD_TH,
    StallOcd_Th_to_RegVal(initPrm->cm.cp.ocdThreshold));
  Powerstep01_CmdSetParam(POWERSTEP01_STEP_MODE,
    (uint8_t)initPrm->cm.cp.syncClockSelection|
    (uint8_t)initPrm->cm.cp.cmVmSelection|
    (uint8_t)(uint8_t)initPrm->cm.cp.stepMode);
  Powerstep01_CmdSetParam(POWERSTEP01_ALARM_EN,
    initPrm->cm.cp.alarmsSelection);
  Powerstep01_CmdSetParam(POWERSTEP01_GATECFG1,
    (uint16_t)initPrm->cm.cp.iGate|
    (uint16_t)initPrm->cm.cp.tcc|
    (uint16_t)initPrm->cm.cp.tBoost|
    (uint16_t)initPrm->cm.cp.wdEn);
  Powerstep01_CmdSetParam(POWERSTEP01_GATECFG2,
    (uint16_t)initPrm->cm.cp.tBlank|
    (uint16_t)initPrm->cm.cp.tdt);  
  if (initPrm->cm.cp.cmVmSelection == POWERSTEP01_CM_VM_VOLTAGE)
  {
    //Voltage mode
    Powerstep01_CmdSetParam(POWERSTEP01_INT_SPD,
      IntSpd_Steps_s_to_RegVal(
        initPrm->vm.intersectSpeed));
    Powerstep01_CmdSetParam(POWERSTEP01_K_THERM,
      KTherm_Comp_to_RegVal(
        initPrm->vm.thermalCompensationFactor)); 
    Powerstep01_CmdSetParam(POWERSTEP01_STALL_TH,
      StallOcd_Th_to_RegVal(
        initPrm->vm.stallThreshold));
    Powerstep01_CmdSetParam(POWERSTEP01_KVAL_HOLD,
      Kval_Perc_to_RegVal(
        initPrm->vm.kvalHold));
    Powerstep01_CmdSetParam(POWERSTEP01_KVAL_RUN,
      Kval_Perc_to_RegVal(
        initPrm->vm.kvalRun));
    Powerstep01_CmdSetParam(POWERSTEP01_KVAL_ACC,
      Kval_Perc_to_RegVal(
        initPrm->vm.kvalAcc));
    Powerstep01_CmdSetParam(POWERSTEP01_KVAL_DEC,
      Kval_Perc_to_RegVal(
        initPrm->vm.kvalDec));
    Powerstep01_CmdSetParam(POWERSTEP01_ST_SLP,
      BEMFslope_Perc_to_RegVal(
        initPrm->vm.startSlope));
    Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_ACC,
      BEMFslope_Perc_to_RegVal(
        initPrm->vm.accelerationFinalSlope));
    Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_DEC,
      BEMFslope_Perc_to_RegVal(
        initPrm->vm.decelerationFinalSlope));
    Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
      (uint16_t)initPrm->vm.oscClkSel| 
      (uint16_t)initPrm->vm.swMode | 
      (uint16_t)initPrm->vm.enVsComp| 
      (uint16_t)initPrm->vm.ocSd| 
      (uint16_t)initPrm->vm.uvloVal| 
      (uint16_t)initPrm->vm.vccVal| 
      (uint16_t)initPrm->vm.fPwmInt| 
      (uint16_t)initPrm->vm.fPwmDec);
  }
  else
  {
    // Current mode
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_HOLD,
          Tval_RefVoltage_to_RegVal(
            initPrm->cm.tvalHold));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_RUN,
          Tval_RefVoltage_to_RegVal(
            initPrm->cm.tvalRun));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_ACC,
          Tval_RefVoltage_to_RegVal(
            initPrm->cm.tvalAcc));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_DEC,
          Tval_RefVoltage_to_RegVal(
            initPrm->cm.tvalDec));
        Powerstep01_CmdSetParam(POWERSTEP01_T_FAST,
          (uint8_t)initPrm->cm.toffFast|
          (uint8_t)initPrm->cm.fastStep);
        Powerstep01_CmdSetParam(POWERSTEP01_TON_MIN,
          Tmin_Time_to_RegVal(
            initPrm->cm.tonMin));
        Powerstep01_CmdSetParam(POWERSTEP01_TOFF_MIN,
          Tmin_Time_to_RegVal(
            initPrm->cm.toffMin));       
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)initPrm->cm.oscClkSel| 
          (uint16_t)initPrm->cm.swMode| 
          (uint16_t)initPrm->cm.tqReg| 
          (uint16_t)initPrm->cm.ocSd| 
          (uint16_t)initPrm->cm.uvloVal| 
          (uint16_t)initPrm->cm.vccVal|
          (uint16_t)initPrm->cm.tsw|
          (uint16_t)initPrm->cm.predEn);
  }
}

/**********************************************************
 * @brief Sets the registers of the Powerstep01 to their predefined values 
 * from powerstep01_target_config.h
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_SetRegisterToPredefinedValues(void)
{
  powerstep01_CmVm_t cmVm;
  
  Powerstep01_CmdSetParam(
    POWERSTEP01_ABS_POS,
    0);
  Powerstep01_CmdSetParam(
    POWERSTEP01_EL_POS,
    0);
  Powerstep01_CmdSetParam(
    POWERSTEP01_MARK,
    0);
  switch (deviceInstance)
  {
    case 0:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0;
      Powerstep01_CmdSetParam(POWERSTEP01_ACC,
        AccDec_Steps_s2_to_RegVal(
          POWERSTEP01_CONF_PARAM_ACC_DEVICE_0));
      Powerstep01_CmdSetParam(POWERSTEP01_DEC,
        AccDec_Steps_s2_to_RegVal(
          POWERSTEP01_CONF_PARAM_DEC_DEVICE_0));
      Powerstep01_CmdSetParam(POWERSTEP01_MAX_SPEED,
        MaxSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_0));
      Powerstep01_CmdSetParam(POWERSTEP01_MIN_SPEED,
        POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_0|
        MinSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_0));
      Powerstep01_CmdSetParam(POWERSTEP01_FS_SPD,
        POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_0|
        FSSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_0)); 
      Powerstep01_CmdSetParam(POWERSTEP01_OCD_TH,
        (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_0);
      Powerstep01_CmdSetParam(POWERSTEP01_STEP_MODE,
        (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_0 |
        (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0|
        (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_0);
      Powerstep01_CmdSetParam(POWERSTEP01_ALARM_EN,
        POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_0);
      Powerstep01_CmdSetParam(POWERSTEP01_GATECFG1,
        (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_0 | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_0   | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_0|
        (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_0);
      Powerstep01_CmdSetParam(POWERSTEP01_GATECFG2,
        (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_0 | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_0);
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam(POWERSTEP01_INT_SPD,
          IntSpd_Steps_s_to_RegVal(
            POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_K_THERM,
          KTherm_Comp_to_RegVal(
            POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_STALL_TH,
          StallOcd_Th_to_RegVal(
            POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_HOLD,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_RUN,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_ACC,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_DEC,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_ST_SLP,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_ACC,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_0));  
        Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_DEC,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 | 
          (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_0       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0         | 
          (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_0       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_0);
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_HOLD,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_RUN,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_ACC,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_DEC,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_T_FAST,
          (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_0 |
          (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_0);
        Powerstep01_CmdSetParam(POWERSTEP01_TON_MIN,
          Tmin_Time_to_RegVal(
            POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_TOFF_MIN,
          Tmin_Time_to_RegVal(
            POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_0));
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 | 
          (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_0        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0         | 
          (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_0           |
          (uint16_t)POWERSTEP01_CONF_PARAM_PRED_DEVICE_0);          
      }
      break;
#if (MAX_NUMBER_OF_DEVICES > 1)
   case 1:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1;
      Powerstep01_CmdSetParam(POWERSTEP01_ACC,
        AccDec_Steps_s2_to_RegVal(
          POWERSTEP01_CONF_PARAM_ACC_DEVICE_1));
      Powerstep01_CmdSetParam(POWERSTEP01_DEC,
        AccDec_Steps_s2_to_RegVal(
          POWERSTEP01_CONF_PARAM_DEC_DEVICE_1));
      Powerstep01_CmdSetParam(POWERSTEP01_MAX_SPEED,
        MaxSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_1));
      Powerstep01_CmdSetParam(POWERSTEP01_MIN_SPEED,
        POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_1|
        MinSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_1));
      Powerstep01_CmdSetParam(POWERSTEP01_FS_SPD,
        POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_1|
        FSSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_1)); 
      Powerstep01_CmdSetParam(POWERSTEP01_OCD_TH,
        (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_1);
      Powerstep01_CmdSetParam(POWERSTEP01_STEP_MODE,
        (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_1 |
        (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1|
        (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_1);
      Powerstep01_CmdSetParam(POWERSTEP01_ALARM_EN,
        POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_1);
      Powerstep01_CmdSetParam(POWERSTEP01_GATECFG1,
        (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_1 | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_1   | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_1|
        (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_1);
      Powerstep01_CmdSetParam(POWERSTEP01_GATECFG2,
        (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_1 | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_1);
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam(POWERSTEP01_INT_SPD,
          IntSpd_Steps_s_to_RegVal(
            POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_K_THERM,
          KTherm_Comp_to_RegVal(
            POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_STALL_TH,
          StallOcd_Th_to_RegVal(
            POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_HOLD,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_RUN,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_ACC,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_DEC,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_ST_SLP,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_ACC,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_1));  
        Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_DEC,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 | 
          (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_1       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1         | 
          (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_1       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_1);
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_HOLD,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_RUN,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_ACC,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_DEC,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_T_FAST,
          (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_1 |
          (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_1);
        Powerstep01_CmdSetParam(POWERSTEP01_TON_MIN,
          Tmin_Time_to_RegVal(
            POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_TOFF_MIN,
          Tmin_Time_to_RegVal(
            POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_1));
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 | 
          (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_1        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1         | 
          (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_1           |
          (uint16_t)POWERSTEP01_CONF_PARAM_PRED_DEVICE_1);          
      }
      break;     
#endif
#if (MAX_NUMBER_OF_DEVICES > 2)
   case 2:
      cmVm = POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2;
      Powerstep01_CmdSetParam(POWERSTEP01_ACC,
        AccDec_Steps_s2_to_RegVal(
          POWERSTEP01_CONF_PARAM_ACC_DEVICE_2));
      Powerstep01_CmdSetParam(POWERSTEP01_DEC,
        AccDec_Steps_s2_to_RegVal(
          POWERSTEP01_CONF_PARAM_DEC_DEVICE_2));
      Powerstep01_CmdSetParam(POWERSTEP01_MAX_SPEED,
        MaxSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_2));
      Powerstep01_CmdSetParam(POWERSTEP01_MIN_SPEED,
        POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_2|
        MinSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_2));
      Powerstep01_CmdSetParam(POWERSTEP01_FS_SPD,
        POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_2|
        FSSpd_Steps_s_to_RegVal(
          POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_2)); 
      Powerstep01_CmdSetParam(POWERSTEP01_OCD_TH,
        (uint8_t)POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_2);
      Powerstep01_CmdSetParam(POWERSTEP01_STEP_MODE,
        (uint8_t)POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_2 |
        (uint8_t)POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2|
        (uint8_t)POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_2);
      Powerstep01_CmdSetParam(POWERSTEP01_ALARM_EN,
        POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_2);
      Powerstep01_CmdSetParam(POWERSTEP01_GATECFG1,
        (uint16_t)POWERSTEP01_CONF_PARAM_IGATE_DEVICE_2 | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TCC_DEVICE_2   | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_2|
        (uint16_t)POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_2);
      Powerstep01_CmdSetParam(POWERSTEP01_GATECFG2,
        (uint16_t)POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_2 | 
        (uint16_t)POWERSTEP01_CONF_PARAM_TDT_DEVICE_2);
      // Voltage mode
      if (cmVm == POWERSTEP01_CM_VM_VOLTAGE)
      {
        Powerstep01_CmdSetParam(POWERSTEP01_INT_SPD,
          IntSpd_Steps_s_to_RegVal(
            POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_K_THERM,
          KTherm_Comp_to_RegVal(
            POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_STALL_TH,
          StallOcd_Th_to_RegVal(
            POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_HOLD,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_RUN,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_ACC,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_KVAL_DEC,
          Kval_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_ST_SLP,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_ACC,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_2));  
        Powerstep01_CmdSetParam(POWERSTEP01_FN_SLP_DEC,
          BEMFslope_Perc_to_RegVal(
            POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 | 
          (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_2       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2         | 
          (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_2       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_2);
      }
      else
      {
        // Current mode
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_HOLD,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_RUN,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_ACC,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_TVAL_DEC,
          Tval_RefVoltage_to_RegVal(
            POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_T_FAST,
          (uint8_t)POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_2 |
          (uint8_t)POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_2);
        Powerstep01_CmdSetParam(POWERSTEP01_TON_MIN,
          Tmin_Time_to_RegVal(
            POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_TOFF_MIN,
          Tmin_Time_to_RegVal(
            POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_2));
        Powerstep01_CmdSetParam(POWERSTEP01_CONFIG,
          (uint16_t)POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 | 
          (uint16_t)POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_2        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2         | 
          (uint16_t)POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2       | 
          (uint16_t)POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2        | 
          (uint16_t)POWERSTEP01_CONF_PARAM_TSW_DEVICE_2           |
          (uint16_t)POWERSTEP01_CONF_PARAM_PRED_DEVICE_2);          
      }
      break;
#endif
    default: ;
  }
}

/**
  * @brief Functions to get and set parameters using digital or analog values
  */

/**********************************************************
 * @brief  Issues the GetParam command to the Powerstep01 device
 * @param[in] parameter Register adress (POWERSTEP01_ABS_POS,
 * POWERSTEP01_MARK,...)
 * @retval Register value
 **********************************************************/
uint32_t POWERSTEP01::Powerstep01_CmdGetParam(powerstep01_Registers_t param)
{

  uint32_t spiRxData;
  uint32_t loop;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfDevices - deviceInstance - 1;
  bool itDisable = FALSE;
  
  do
  {
    spiPreemptionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable Powerstep01_Board_EnableIrq if disable in previous iteration */
      Powerstep01_Board_EnableIrq();
      itDisable = FALSE;
    }
    for (loop = 0; loop < numberOfDevices; loop++)
    {
      spiTxBursts[0][loop] = POWERSTEP01_NOP;
      spiTxBursts[1][loop] = POWERSTEP01_NOP;
      spiTxBursts[2][loop] = POWERSTEP01_NOP;
      spiTxBursts[3][loop] = POWERSTEP01_NOP;
      spiRxBursts[0][loop] = 0;
      spiRxBursts[1][loop] = 0;
      spiRxBursts[2][loop] = 0;
      spiRxBursts[3][loop] = 0;    
    }
    switch (param)
    {
      case POWERSTEP01_ABS_POS: 
      case POWERSTEP01_MARK:
      case POWERSTEP01_SPEED:
        spiTxBursts[0][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 3;
        break;
      case POWERSTEP01_EL_POS:
      case POWERSTEP01_ACC:
      case POWERSTEP01_DEC:
      case POWERSTEP01_MAX_SPEED:
      case POWERSTEP01_MIN_SPEED:
      case POWERSTEP01_FS_SPD:
      case POWERSTEP01_INT_SPD:
      case POWERSTEP01_CONFIG:
      case POWERSTEP01_GATECFG1:
      case POWERSTEP01_STATUS:
        spiTxBursts[1][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 2;
        break;
    default:
        spiTxBursts[2][spiIndex] = ((uint8_t)POWERSTEP01_GET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    Powerstep01_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemptionByIsr); // check pre-emption by ISR
  for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES-1-maxArgumentNbBytes;
       loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
       loop++)
  {
     Powerstep01_WriteBytes(&spiTxBursts[loop][0],
                           &spiRxBursts[loop][0]);
  }
  spiRxData = ((uint32_t)spiRxBursts[1][spiIndex] << 16)|
               (spiRxBursts[2][spiIndex] << 8) |
               (spiRxBursts[3][spiIndex]);    
  /* re-enable Powerstep01_Board_EnableIrq after SPI transfers*/
  Powerstep01_Board_EnableIrq();
  return (spiRxData);
}

/**********************************************************
 * @brief  Issues the SetParam command to the PowerStep01 device
 * @param[in] parameter Register adress (POWERSTEP01_ABS_POS,
 * POWERSTEP01_MARK,...)
 * @param[in] value Value to set in the register
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_CmdSetParam(powerstep01_Registers_t param,
  uint32_t value)
{
  uint32_t loop;
  uint8_t maxArgumentNbBytes = 0;
  uint8_t spiIndex = numberOfDevices - deviceInstance - 1;
  bool itDisable = FALSE;
  
  do
  {
    spiPreemptionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable Powerstep01_Board_EnableIrq if disable in previous iteration */
      Powerstep01_Board_EnableIrq();
      itDisable = FALSE;
    }  
    for (loop = 0;loop < numberOfDevices; loop++)
    {
      spiTxBursts[0][loop] = POWERSTEP01_NOP;
      spiTxBursts[1][loop] = POWERSTEP01_NOP;
      spiTxBursts[2][loop] = POWERSTEP01_NOP;
      spiTxBursts[3][loop] = POWERSTEP01_NOP;
    }
    switch (param)
    {
      case POWERSTEP01_ABS_POS: ;
      case POWERSTEP01_MARK:
        spiTxBursts[0][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 3;
        break;
      case POWERSTEP01_EL_POS:
      case POWERSTEP01_ACC:
      case POWERSTEP01_DEC:
      case POWERSTEP01_MAX_SPEED:
      case POWERSTEP01_MIN_SPEED:
      case POWERSTEP01_FS_SPD:
      case POWERSTEP01_INT_SPD:
      case POWERSTEP01_CONFIG:
      case POWERSTEP01_GATECFG1:
        spiTxBursts[1][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        maxArgumentNbBytes = 2;
        break;
      default:
        spiTxBursts[2][spiIndex] = ((uint8_t)POWERSTEP01_SET_PARAM )| (param);
        maxArgumentNbBytes = 1;
    }
    spiTxBursts[3][spiIndex] = (uint8_t)(value);
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    Powerstep01_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemptionByIsr); // check pre-emption by ISR  
  /* SPI transfer */
  for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes;
       loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES;
       loop++)
  {
    Powerstep01_WriteBytes(&spiTxBursts[loop][0],&spiRxBursts[loop][0]);
  }
  /* re-enable Powerstep01_Board_EnableIrq after SPI transfers*/
  Powerstep01_Board_EnableIrq();
}

/**********************************************************
 * @brief Issues PowerStep01 Get Parameter command and convert the result to
 * float value
 * @param[in] param PowerStep01 register address
 * @retval The parameter's float value.
 *********************************************************/
float POWERSTEP01::Powerstep01_GetAnalogValue(powerstep01_Registers_t param)
{
  bool voltageMode = ((POWERSTEP01_CM_VM_CURRENT&Powerstep01_CmdGetParam(POWERSTEP01_STEP_MODE))==0);
  uint32_t registerValue = Powerstep01_CmdGetParam((powerstep01_Registers_t) param);
  float value;
  switch (param)
  {
    case POWERSTEP01_ABS_POS:
    case POWERSTEP01_MARK:
      value = (float) Powerstep01_ConvertPosition(registerValue);
      break;
    case POWERSTEP01_ACC:
    case POWERSTEP01_DEC:
      value = AccDec_RegVal_to_Steps_s2(registerValue);
      break;
    case POWERSTEP01_SPEED:
      value = Speed_RegVal_to_Steps_s(registerValue);
      break;
    case POWERSTEP01_MAX_SPEED:
      value = MaxSpd_RegVal_to_Steps_s(registerValue);
      break;
    case POWERSTEP01_MIN_SPEED:
      registerValue &= POWERSTEP01_MIN_SPEED_MASK;
      value = MinSpd_RegVal_to_Steps_s(registerValue);
      break;      
    case POWERSTEP01_FS_SPD:
      registerValue &= POWERSTEP01_FS_SPD_MASK;
      value = FSSpd_RegVal_to_Steps_s(registerValue);
      break;
    case POWERSTEP01_INT_SPD:
      value = IntSpd_RegVal_to_Steps_s(registerValue);
      break;
    case POWERSTEP01_K_THERM:
      value = KTherm_RegVal_to_Comp(registerValue);
      break;
    case POWERSTEP01_OCD_TH:
    case POWERSTEP01_STALL_TH:
      value = StallOcd_RegVal_to_Th(registerValue);
      break;
    case POWERSTEP01_KVAL_HOLD:  //POWERSTEP01_TVAL_HOLD
    case POWERSTEP01_KVAL_RUN:   //POWERSTEP01_TVAL_RUN
    case POWERSTEP01_KVAL_ACC:   //POWERSTEP01_TVAL_ACC
    case POWERSTEP01_KVAL_DEC:   //POWERSTEP01_TVAL_DEC
      if (voltageMode!=FALSE)  value = Kval_RegVal_to_Perc(registerValue);
      else value = Kval_RegVal_to_Perc(registerValue);     
      break;
    case POWERSTEP01_ST_SLP:
      if (voltageMode==FALSE) 
      {
        break;
      }      
    case POWERSTEP01_FN_SLP_ACC: //POWERSTEP01_TON_MIN
    case POWERSTEP01_FN_SLP_DEC: //POWERSTEP01_TOFF_MIN
      if (voltageMode!=FALSE) value = BEMFslope_RegVal_to_Perc(registerValue);
      else value = Tmin_RegVal_to_Time(registerValue);
      break;
    default:
      value = (float) registerValue;
  }
  return value;
}

/******************************************************//**
 * @brief Put commands in queue before synchronous sending
 * done by calling Powerstep01_SendQueuedCommands.
 * Any call to functions that use the SPI between the calls of 
 * Powerstep01_QueueCommands and Powerstep01_SendQueuedCommands 
 * will corrupt the queue.
 * A command for each device of the daisy chain must be 
 * specified before calling Powerstep01_SendQueuedCommands.
 * @param[in] command Command to queue (all Powerstep01 commmands 
 * except POWERSTEP01_SET_PARAM, POWERSTEP01_GET_PARAM, 
 * POWERSTEP01_GET_STATUS)
 * @param[in] value argument of the command to queue
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_QueueCommands(uint8_t command, int32_t value)
{
  if (numberOfDevices > deviceInstance)
  {
    uint8_t spiIndex = numberOfDevices - deviceInstance - 1;
    
    switch (command & DAISY_CHAIN_COMMAND_MASK)
    {
      case POWERSTEP01_RUN: ;
      case POWERSTEP01_MOVE: ;
      case POWERSTEP01_GO_TO: ;
      case POWERSTEP01_GO_TO_DIR: ;
      case POWERSTEP01_GO_UNTIL: ;
      case POWERSTEP01_GO_UNTIL_ACT_CPY:
       spiTxBursts[0][spiIndex] = command;
       spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
       spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
       spiTxBursts[3][spiIndex] = (uint8_t)(value);
       break;
      default:
       spiTxBursts[0][spiIndex] = POWERSTEP01_NOP;
       spiTxBursts[1][spiIndex] = POWERSTEP01_NOP;
       spiTxBursts[2][spiIndex] = POWERSTEP01_NOP;
       spiTxBursts[3][spiIndex] = command;
    }
  }
}

/**********************************************************
 * @brief  Sends a command to the device via the SPI
 * @param[in] command Command to send (all Powerstep01 commmands 
 * except POWERSTEP01_SET_PARAM, POWERSTEP01_GET_PARAM, 
 * POWERSTEP01_GET_STATUS)
 * @param[in] value arguments to send on 32 bits
 * @retval None
 **********************************************************/
void POWERSTEP01::Powerstep01_SendCommand(uint8_t command, uint32_t value)
{
  uint32_t loop;
  uint8_t maxArgumentNbBytes = 0;
  bool itDisable = FALSE;
  uint8_t spiIndex = numberOfDevices - deviceInstance - 1;
  
  do
  {
    spiPreemptionByIsr = FALSE;
    if (itDisable)
    {
      /* re-enable Powerstep01_Board_EnableIrq if disable in previous iteration */
      Powerstep01_Board_EnableIrq();
      itDisable = FALSE;
    }    
    for (loop = 0; loop < numberOfDevices; loop++)
    {
        spiTxBursts[0][loop] = POWERSTEP01_NOP;
        spiTxBursts[1][loop] = POWERSTEP01_NOP;
        spiTxBursts[2][loop] = POWERSTEP01_NOP;
        spiTxBursts[3][loop] = POWERSTEP01_NOP;   
    }
    switch (command & DAISY_CHAIN_COMMAND_MASK)
    {
      case POWERSTEP01_GO_TO:
      case POWERSTEP01_GO_TO_DIR:
        value = value & POWERSTEP01_ABS_POS_VALUE_MASK;
      case POWERSTEP01_RUN:
      case POWERSTEP01_MOVE:
      case POWERSTEP01_GO_UNTIL:
      case POWERSTEP01_GO_UNTIL_ACT_CPY:
        spiTxBursts[0][spiIndex] = command;
        spiTxBursts[1][spiIndex] = (uint8_t)(value >> 16);
        spiTxBursts[2][spiIndex] = (uint8_t)(value >> 8);
        spiTxBursts[3][spiIndex] = (uint8_t)(value);
        maxArgumentNbBytes = 3;
        break;
    default:
        spiTxBursts[0][spiIndex] = POWERSTEP01_NOP;
        spiTxBursts[1][spiIndex] = POWERSTEP01_NOP;
        spiTxBursts[2][spiIndex] = POWERSTEP01_NOP;
        spiTxBursts[3][spiIndex] = command;
    }
    /* Disable interruption before checking */
    /* pre-emption by ISR and SPI transfers*/
    Powerstep01_Board_DisableIrq();
    itDisable = TRUE;
  } while (spiPreemptionByIsr); // check pre-emption by ISR
  for (loop = POWERSTEP01_CMD_ARG_MAX_NB_BYTES - 1 - maxArgumentNbBytes; 
       loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES; 
       loop++)
  {
     Powerstep01_WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
  /* re-enable Powerstep01_Board_EnableIrq after SPI transfers*/
  Powerstep01_Board_EnableIrq();
}

/******************************************************//**
 * @brief Sends commands stored previously in the queue by 
 * Powerstep01_QueueCommands
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_SendQueuedCommands(void)
{
  uint8_t loop;
  
  for (loop = 0; 
       loop < POWERSTEP01_CMD_ARG_MAX_NB_BYTES; 
       loop++)
  {
     Powerstep01_WriteBytes(&spiTxBursts[loop][0], &spiRxBursts[loop][0]);
  }
}

/**********************************************************
 * @brief Issues the SetParam command to the PowerStep01
 * @param[in] param PowerStep01 Register address
 * @param[in] value Float value to convert and set into the register
 * @retval TRUE if param is valid, FALSE otherwise
 *********************************************************/
bool POWERSTEP01::Powerstep01_SetAnalogValue(powerstep01_Registers_t param, float value)
{
  uint32_t registerValue;
  bool result = TRUE;
  bool voltageMode = ((POWERSTEP01_CM_VM_CURRENT&Powerstep01_CmdGetParam(POWERSTEP01_STEP_MODE))==0);
  if ((value < 0)&&((param != POWERSTEP01_ABS_POS)&&(param != POWERSTEP01_MARK)))
  {
    result = FALSE;
  }
  switch (param)
  {
    case POWERSTEP01_EL_POS:
      if ((value > (POWERSTEP01_ELPOS_STEP_MASK|POWERSTEP01_ELPOS_MICROSTEP_MASK))||
          ((value!=0)&&(value < (1<<(7-(POWERSTEP01_STEP_MODE_STEP_SEL&Powerstep01_CmdGetParam(POWERSTEP01_STEP_MODE))))))) result = FALSE;
      else registerValue = ((uint32_t) value)&(POWERSTEP01_ELPOS_STEP_MASK|POWERSTEP01_ELPOS_MICROSTEP_MASK);
      break;        
    case POWERSTEP01_ABS_POS:
    case POWERSTEP01_MARK:
      if (value < 0)
      {
        value=-value;
        if (((uint32_t)value)<=(POWERSTEP01_MAX_POSITION+1))
          registerValue = (POWERSTEP01_ABS_POS_VALUE_MASK+1-(uint32_t)value)&POWERSTEP01_ABS_POS_VALUE_MASK;
        else result = FALSE;  
      }
      else
      {
        if (((uint32_t)value)<=POWERSTEP01_MAX_POSITION)
          registerValue = ((uint32_t) value)&POWERSTEP01_ABS_POS_VALUE_MASK;
        else result = FALSE; 
      }
      break;
    case POWERSTEP01_ACC:
    case POWERSTEP01_DEC:
      if (value > POWERSTEP01_ACC_DEC_MAX_VALUE) result = FALSE;
      else registerValue = AccDec_Steps_s2_to_RegVal(value);
      break;
    case POWERSTEP01_MAX_SPEED:
      if (value > POWERSTEP01_MAX_SPEED_MAX_VALUE) result = FALSE;
      else registerValue = MaxSpd_Steps_s_to_RegVal(value);
      break;
    case POWERSTEP01_MIN_SPEED:
      if (value > POWERSTEP01_MIN_SPEED_MAX_VALUE) result = FALSE;
      else registerValue = (POWERSTEP01_LSPD_OPT&Powerstep01_CmdGetParam(param))|MinSpd_Steps_s_to_RegVal(value);
      break;      
    case POWERSTEP01_FS_SPD:
      if (value > POWERSTEP01_FS_SPD_MAX_VALUE) result = FALSE;
      else registerValue = (POWERSTEP01_BOOST_MODE&Powerstep01_CmdGetParam(param))|FSSpd_Steps_s_to_RegVal(value);
      break;
    case POWERSTEP01_INT_SPD:
      if (value > POWERSTEP01_INT_SPD_MAX_VALUE) result = FALSE;
      else registerValue = IntSpd_Steps_s_to_RegVal(value);
      break;
    case POWERSTEP01_K_THERM:
      if ((value < POWERSTEP01_K_THERM_MIN_VALUE)||
          (value > POWERSTEP01_K_THERM_MAX_VALUE)) result = FALSE;
      else registerValue = KTherm_Comp_to_RegVal(value);
      break;
    case POWERSTEP01_OCD_TH:
    case POWERSTEP01_STALL_TH:
      if (value > POWERSTEP01_STALL_OCD_TH_MAX_VALUE) result = FALSE;
      else registerValue = StallOcd_Th_to_RegVal(value);
      break;
    case POWERSTEP01_KVAL_HOLD:  //POWERSTEP01_TVAL_HOLD
    case POWERSTEP01_KVAL_RUN:   //POWERSTEP01_TVAL_RUN
    case POWERSTEP01_KVAL_ACC:   //POWERSTEP01_TVAL_ACC
    case POWERSTEP01_KVAL_DEC:   //POWERSTEP01_TVAL_DEC
      if (voltageMode==FALSE)
      {
        if (value > POWERSTEP01_TVAL_MAX_VALUE) result = FALSE;
        else registerValue = Tval_RefVoltage_to_RegVal(value);
      }
      else
      {
        if (value > POWERSTEP01_KVAL_MAX_VALUE) result = FALSE;
        else registerValue = Kval_Perc_to_RegVal(value);
      }
      break;
    case POWERSTEP01_ST_SLP:
      if (voltageMode==FALSE) 
      {
        result = FALSE;
        break;
      }
    case POWERSTEP01_FN_SLP_ACC: //POWERSTEP01_TON_MIN
    case POWERSTEP01_FN_SLP_DEC: //POWERSTEP01_TOFF_MIN
      if (voltageMode==FALSE)
      {
        if (value>POWERSTEP01_TOFF_TON_MIN_MAX_VALUE) result = FALSE;
        else registerValue = Tmin_Time_to_RegVal(value);
      }
      else
      {
        if (value > POWERSTEP01_SLP_MAX_VALUE) result = FALSE;
        else registerValue = BEMFslope_Perc_to_RegVal(value);
      }
      break;
    default:
      result = FALSE;
  }
  if (result!=FALSE)
  {
    Powerstep01_CmdSetParam(param, registerValue);
  }
  return result;
}

/**********************************************************
 * @brief Write and receive a byte via SPI
 * @param[in] pByteToTransmit pointer to the byte to transmit
 * @param[in] pReceivedByte pointer to the received byte
 * @retval None
 *********************************************************/
void POWERSTEP01::Powerstep01_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
{
  if (Powerstep01_Board_SpiWriteBytes(pByteToTransmit, pReceivedByte) != 0)
  {
    Powerstep01_ErrorHandler(POWERSTEP01_ERROR_1);
  }
  if (isrFlag)
  {
    spiPreemptionByIsr = TRUE;
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
