/**
 ******************************************************************************
 * @file    powerstep01_class.h
 * @author  IPC Rennes
 * @version V1.0.0
 * @date    March 18th, 2016
 * @brief   This file contains the class of a Powerstep01 Motor Control component.
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


/* Define to prevent recursive inclusion -------------------------------------*/

#ifndef __POWERSTEP01_CLASS_H
#define __POWERSTEP01_CLASS_H


/* Includes ------------------------------------------------------------------*/

/* ACTION 1 ------------------------------------------------------------------*
 * Include here platform specific header files.                               *
 *----------------------------------------------------------------------------*/        
//#include "mbed.h"
#include "DevSPI.h"
/* ACTION 2 ------------------------------------------------------------------*
 * Include here component specific header files.                              *
 *----------------------------------------------------------------------------*/        
#include "powerstep01.h"
/* ACTION 3 ------------------------------------------------------------------*
 * Include here interface specific header files.                              *
 *                                                                            *
 * Example:                                                                   *
 *   #include "../Interfaces/Humidity_class.h"                                *
 *   #include "../Interfaces/Temperature_class.h"                             *
 *----------------------------------------------------------------------------*/
#include "StepperMotor_class.h"


/* Classes -------------------------------------------------------------------*/

/**
 * @brief Class representing a Powerstep01 component.
 */
class POWERSTEP01 : public StepperMotor
{
public:

    /*** Constructor and Destructor Methods ***/

    /**
     * @brief Constructor.
     * @param flag_irq      pin name of the FLAG pin of the component.
     * @param busy_irq      pin name of the BUSY pin of the component.
     * @param standby_reset pin name of the STBY\RST pin of the component.
     * @param pwm           pin name of the PWM pin of the component.
     * @param ssel          pin name of the SSEL pin of the SPI device to be used for communication.
     * @param spi           SPI device to be used for communication.
     */
     //POWERSTEP01(PinName flag_irq, PinName busy_irq, PinName standby_reset, PinName pwm, PinName ssel, DevSPI &spi) : StepperMotor(), flag_irq(flag_irq), busy_irq(busy_irq), standby_reset(standby_reset), pwm(pwm), ssel(ssel), dev_spi(spi)
    POWERSTEP01(PinName standby_reset,PinName ssel, DevSPI &spi) : StepperMotor(), standby_reset(standby_reset),  ssel(ssel), dev_spi(spi)
    {
        /* Checking stackability. */
        if (!(numberOfDevices < MAX_NUMBER_OF_DEVICES))
            error("Instantiation of the powerstep01 component failed: it can be stacked up to %d times.\r\n", MAX_NUMBER_OF_DEVICES);

        /* ACTION 4 ----------------------------------------------------------*
         * Initialize here the component's member variables, one variable per *
         * line.                                                              *
         *                                                                    *
         * Example:                                                           *
         *   measure = 0;                                                     *
         *   instance_id = number_of_instances++;                             *
         *--------------------------------------------------------------------*/
        errorHandlerCallback = 0;
        deviceInstance = numberOfDevices++;
        memset(spiTxBursts, 0, POWERSTEP01_CMD_ARG_MAX_NB_BYTES * MAX_NUMBER_OF_DEVICES * sizeof(uint8_t));
        memset(spiRxBursts, 0, POWERSTEP01_CMD_ARG_MAX_NB_BYTES * MAX_NUMBER_OF_DEVICES * sizeof(uint8_t));
    }
    
    /**
     * @brief Destructor.
     */
    virtual ~POWERSTEP01(void) {}
    

    /*** Public Component Related Methods ***/

    /* ACTION 5 --------------------------------------------------------------*
     * Implement here the component's public methods, as wrappers of the C    *
     * component's functions.                                                 *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2).                                     *
     *                                                                        *
     * Example:                                                               *
     *   virtual int GetValue(float *pData) //(1)                             *
     *   {                                                                    *
     *     return COMPONENT_GetValue(float *pfData);                          *
     *   }                                                                    *
     *                                                                        *
     *   virtual int EnableFeature(void) //(2)                                *
     *   {                                                                    *
     *     return COMPONENT_EnableFeature();                                  *
     *   }                                                                    *
     *------------------------------------------------------------------------*/

    /**
     * @brief Public functions inherited from the Component Class
     */

    /**
     * @brief  Initialize the component.
     * @param  init Pointer to device specific initalization structure.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int Init(void *init = NULL)
    {
        return (int) Powerstep01_Init((void *) init);
    }

    /**
     * @brief  Getting the ID of the component.
     * @param  id Pointer to an allocated variable to store the ID into.
     * @retval "0" in case of success, an error code otherwise.
     */
    virtual int ReadID(uint8_t *id = NULL)
    {
        return (int) Powerstep01_ReadID((uint8_t *) id);
    }

    /**
     * @brief Public functions inherited from the StepperMotor Class
     */

    /**
     * @brief  Getting the value of the Status Register.
     * @param  None.
     * @retval None.
     * @note   The Status Register's flags are cleared, contrary to ReadStatusRegister().
     */
    virtual unsigned int GetStatus(void)
    {
        return (unsigned int) Powerstep01_CmdGetStatus();
    }

    /**
     * @brief  Getting the position.
     * @param  None.
     * @retval The position.
     */
    virtual signed int GetPosition(void)
    {
        return (signed int)Powerstep01_GetPosition();
    }

    /**
     * @brief  Getting the marked position.
     * @param  None.
     * @retval The marked position.
     */
    virtual signed int GetMark(void)
    {
        return (signed int)Powerstep01_GetMark();
    }

    /**
     * @brief  Getting the current speed in pps.
     * @param  None.
     * @retval The current speed in pps.
     */
    virtual unsigned int GetSpeed(void)
    {
        return (unsigned int)round(Powerstep01_GetAnalogValue(POWERSTEP01_SPEED));
    }

    /**
     * @brief  Getting the maximum speed in pps.
     * @param  None.
     * @retval The maximum speed in pps.
     */
    virtual unsigned int GetMaxSpeed(void)
    {
        return (unsigned int)round(Powerstep01_GetAnalogValue(POWERSTEP01_MAX_SPEED));
    }

    /**
     * @brief  Getting the minimum speed in pps.
     * @param  None.
     * @retval The minimum speed in pps.
     */
    virtual unsigned int GetMinSpeed(void)
    {
        return (unsigned int)round(Powerstep01_GetAnalogValue(POWERSTEP01_MIN_SPEED));
    }

    /**
     * @brief  Getting the acceleration in pps^2.
     * @param  None.
     * @retval The acceleration in pps^2.
     */
    virtual unsigned int GetAcceleration(void)
    {
        return (unsigned int)round(Powerstep01_GetAnalogValue(POWERSTEP01_ACC));
    }

    /**
     * @brief  Getting the deceleration in pps^2.
     * @param  None.
     * @retval The deceleration in pps^2.
     */
    virtual unsigned int GetDeceleration(void)
    {
        return (unsigned int)round(Powerstep01_GetAnalogValue(POWERSTEP01_DEC));
    }
    
    /**
     * @brief  Getting the direction of rotation.
     * @param  None.
     * @retval The direction of rotation.
     */
    virtual direction_t GetDirection(void)
    {
        if ((POWERSTEP01_STATUS_DIR&Powerstep01_ReadStatusRegister())!=0)
        {
            return FWD;
        }
        else
        {
            return BWD;
        }
    }
    
    /**
     * @brief  Setting the current position to be the home position.
     * @param  None.
     * @retval None.
     */
    virtual void SetHome(void)
    {
        Powerstep01_SetHome();
    }

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void SetMark(void)
    {
        Powerstep01_SetMark();
    }

    /**
     * @brief  Setting the maximum speed in steps/s.
     * @param  speed The maximum speed in steps/s.
     * @retval TRUE if value is valid, FALSE otherwise.
     */
    virtual bool SetMaxSpeed(unsigned int speed)
    {
        return Powerstep01_SetAnalogValue(POWERSTEP01_MAX_SPEED, (float)speed);
    }

    /**
     * @brief  Setting the minimum speed in steps/s.
     * @param  speed The minimum speed in steps/s.
     * @retval TRUE if value is valid, FALSE otherwise.
     */
    virtual bool SetMinSpeed(unsigned int speed)
    {
        return Powerstep01_SetAnalogValue(POWERSTEP01_MIN_SPEED, (float)speed);
    }

    /**
     * @brief  Setting the acceleration in steps/s^2.
     * @param  acceleration The acceleration in steps/s^2.
     * @retval None.
     */
    virtual bool SetAcceleration(unsigned int acceleration)
    {
        return Powerstep01_SetAnalogValue(POWERSTEP01_ACC, (float)acceleration);
    }

    /**
     * @brief  Setting the deceleration in steps/s^2.
     * @param  deceleration The deceleration in steps/s^2.
     * @retval None.
     */
    virtual bool SetDeceleration(unsigned int deceleration)
    {
        return Powerstep01_SetAnalogValue(POWERSTEP01_DEC, (float)deceleration);
    }

    /**
     * @brief  Setting the Step Mode.
     * @param  step_mode The Step Mode.
     * @retval None.
     * @note   step_mode can be one of the following:
     *           + STEP_MODE_FULL
     *           + STEP_MODE_HALF
     *           + STEP_MODE_1_4
     *           + STEP_MODE_1_8
     *           + STEP_MODE_1_16
     *           + STEP_MODE_1_32
     *           + STEP_MODE_1_64
     *           + STEP_MODE_1_128
     */
    virtual bool SetStepMode(step_mode_t step_mode)
    {
        return Powerstep01_SelectStepMode((motorStepMode_t) step_mode);
    }

    /**
     * @brief  Going to a specified position.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void GoTo(signed int position)
    {
        Powerstep01_CmdGoTo((int32_t)position);
    }

    virtual void GoTo(direction_t direction, signed int position)
    {
        Powerstep01_CmdGoToDir((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD),(int32_t)position);
    }

    /**
     * @brief  Going to the home position.
     * @param  None.
     * @retval None.
     */
    virtual void GoHome(void)
    {
        Powerstep01_CmdGoHome();
    }

    /**
     * @brief  Going to the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void GoMark(void)
    {
        Powerstep01_CmdGoMark();
    }

    /**
     * @brief  Running the motor towards a specified direction.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void Run(direction_t direction)
    {
        Powerstep01_CmdRun((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD), Powerstep01_CmdGetParam((powerstep01_Registers_t) POWERSTEP01_MAX_SPEED));
    }

    /**
     * @brief  Moving the motor towards a specified direction for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     */
    virtual void Move(direction_t direction, unsigned int steps)
    {
        Powerstep01_CmdMove((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD), (uint32_t)steps);
    }

    /**
     * @brief  Stopping the motor through an immediate deceleration up to zero speed.
     * @param  None.
     * @retval None.
     */
    virtual void SoftStop(void)
    {
        Powerstep01_CmdSoftStop();
    }

    /**
     * @brief  Stopping the motor through an immediate infinite deceleration.
     * @param  None.
     * @retval None.
     */
    virtual void HardStop(void)
    {
        Powerstep01_CmdHardStop();
    }

    /**
     * @brief  Disabling the power bridge after performing a deceleration to zero.
     * @param  None.
     * @retval None.
     */
    virtual void SoftHiZ(void)
    {
        Powerstep01_CmdSoftHiZ();
    }

    /**
     * @brief  Disabling the power bridge immediately.
     * @param  None.
     * @retval None.
     */
    virtual void HardHiZ(void)
    {
        Powerstep01_CmdHardHiZ();
    }

    /**
     * @brief  Waiting while the motor is active.
     * @param  None.
     * @retval None.
     */
    virtual void WaitWhileActive(void)
    {
        Powerstep01_WaitWhileActive();
    }  

    /**
     * @brief Public functions NOT inherited
     */
     
    /**
     * @brief  Attaching an error handler.
     * @param  fptr An error handler.
     * @retval None.
     */
    virtual void AttachErrorHandler(void)// (*fptr)(uint16_t error))
    {
        //Powerstep01_AttachErrorHandler((void (*)(uint16_t error)) fptr);
    }
    
    /**
     * @brief  Checks if the device is busy by reading the busy pin position.
     * @param  None.
     * @retval One if the device his busy (low logic level on busy output),
     * otherwise zero
     */
    virtual int CheckBusyHw(void)
    {
       // if (busy_irq!=0) return 0x01;
       // else return 0x00;
       return 0;
    }

    /**
     * @brief  Checks if the device has an alarm flag set by reading the flag pin position.
     * @param  None.
     * @retval One if the device has an alarm flag set (low logic level on flag output),
     * otherwise zero
     */
    virtual unsigned int CheckStatusHw(void)
    {
       // if (flag_irq!=0) return 0x01;
       // else return 0x00;
       return 0;
    }
    
    /**
     * @brief Fetch and clear status flags of all devices 
     * by issuing a GET_STATUS command simultaneously  
     * to all devices.
     * Then, the fetched status of each device can be retrieved
     * by using the Powerstep01_GetFetchedStatus function
     * provided there is no other calls to functions which 
     * use the SPI in between.
     * @retval None
     */
    virtual void FetchAndClearAllStatus(void)
    {
        Powerstep01_FetchAndClearAllStatus();
    }

    /**
     * @brief  Getting a parameter float value.
     * @param  parameter A parameter's register adress.
     * @retval The parameter's float value.
     *         parameter can be one of the following:
     *           + POWERSTEP01_ABS_POS
     *           + POWERSTEP01_MARK
     *           + POWERSTEP01_ACC
     *           + POWERSTEP01_DEC
     *           + POWERSTEP01_SPEED     
     *           + POWERSTEP01_MAX_SPEED
     *           + POWERSTEP01_MIN_SPEED
     *           + POWERSTEP01_FS_SPD
     *           (voltage mode) + POWERSTEP01_INT_SPD
     *           (voltage mode) + POWERSTEP01_K_THERM
     *           + POWERSTEP01_OCD_TH
     *           (voltage mode) + POWERSTEP01_STALL_TH
     *           (voltage mode) + POWERSTEP01_KVAL_HOLD : value in %
     *           (current mode) + POWERSTEP01_TVAL_HOLD : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_RUN  : value in %
     *           (current mode) + POWERSTEP01_TVAL_RUN  : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_ACC  : value in %
     *           (current mode) + POWERSTEP01_TVAL_ACC  : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_DEC  : value in %
     *           (current mode) + POWERSTEP01_TVAL_DEC  : value in mV
     *           (voltage mode) + POWERSTEP01_ST_SLP
     *           (voltage mode) + POWERSTEP01_FN_SLP_ACC
     *           (voltage mode) + POWERSTEP01_FN_SLP_DEC
     */
    virtual float GetAnalogValue(unsigned int parameter)
    {
      return Powerstep01_GetAnalogValue((powerstep01_Registers_t)parameter);
    }

    /**
     * @brief Get the value of the STATUS register which was 
     * fetched by using Powerstep01_FetchAndClearAllStatus.
     * The fetched values are available  as long as there
     * no other calls to functions which use the SPI.
     * @retval Last fetched value of the STATUS register.
     */ 
    virtual uint16_t GetFetchedStatus(void)
    {
        return Powerstep01_GetFetchedStatus();
    }

    /**
     * @brief  Getting the version of the firmware.
     * @param  None.
     * @retval The version of the firmware.
     */
    virtual unsigned int GetFwVersion(void)
    {
        return (unsigned int) Powerstep01_GetFwVersion();
    }

    /**
     * @brief  Getting a parameter register value.
     * @param  parameter A parameter's register adress.
     * @retval The parameter's register value.
     *         parameter can be one of the following:
     *           + POWERSTEP01_ABS_POS
     *           + POWERSTEP01_EL_POS
     *           + POWERSTEP01_MARK
     *           + POWERSTEP01_SPEED
     *           + POWERSTEP01_ACC
     *           + POWERSTEP01_DEC
     *           + POWERSTEP01_MAX_SPEED
     *           + POWERSTEP01_MIN_SPEED
     *           (voltage mode) + POWERSTEP01_KVAL_HOLD : value in %
     *           (current mode) + POWERSTEP01_TVAL_HOLD : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_RUN  : value in %
     *           (current mode) + POWERSTEP01_TVAL_RUN  : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_ACC  : value in %
     *           (current mode) + POWERSTEP01_TVAL_ACC  : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_DEC  : value in %
     *           (current mode) + POWERSTEP01_TVAL_DEC  : value in mV
     *           (voltage mode) + POWERSTEP01_INT_SPD
     *           (voltage mode) + POWERSTEP01_ST_SLP
     *           (current mode) + POWERSTEP01_T_FAST
     *           (voltage mode) + POWERSTEP01_FN_SLP_ACC
     *           (current mode) + POWERSTEP01_TON_MIN
     *           (voltage mode) + POWERSTEP01_FN_SLP_DEC
     *           (current mode) + POWERSTEP01_TOFF_MIN
     *           (voltage mode) + POWERSTEP01_K_THERM
     *           + POWERSTEP01_ADC_OUT
     *           + POWERSTEP01_OCD_TH
     *           (voltage mode) + POWERSTEP01_STALL_TH
     *           + POWERSTEP01_FS_SPD
     *           + POWERSTEP01_STEP_MODE
     *           + POWERSTEP01_ALARM_EN
     *           + POWERSTEP01_GATECFG1
     *           + POWERSTEP01_GATECFG2
     *           + POWERSTEP01_CONFIG
     *           + POWERSTEP01_STATUS
     */
    virtual unsigned int GetRawParameter(unsigned int parameter)
    {
      return (unsigned int) Powerstep01_CmdGetParam((powerstep01_Registers_t)parameter);
    }
    
    /**
     * @brief  Issues PowerStep01 Go Until command.
     * @param  action type of action to undertake when the SW
     * input is forced high (ACTION_RESET or ACTION_COPY).
     * @param  direction The direction of rotation.
     * @param  speed in steps/s.
     * @retval One if the device has an alarm flag set (low logic level on flag output),
     * otherwise zero
     */    
    virtual void GoUntil(motorAction_t action, direction_t direction, float speed)
    {
        Powerstep01_CmdGoUntil(action,
                               (motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD),
                               Speed_Steps_s_to_RegVal(speed));
    }
    
    /**
     * @brief Checks if the device is busy
     * by reading the Busy flag bit ot its status Register
     * This operation clears the status register
     * @retval true if device is busy, false zero  
     */
    virtual bool IsDeviceBusy(void)
    {
        return Powerstep01_IsDeviceBusy();
    }

    /**
     * @brief Put commands in queue before synchronous sending
     * done by calling SendQueuedCommands.
     * Any call to functions that use the SPI between the calls of 
     * QueueCommands and SendQueuedCommands 
     * will corrupt the queue.
     * A command for each device of the daisy chain must be 
     * specified before calling SendQueuedCommands.
     * @param command Command to queue (all Powerstep01 commmands 
     * except POWERSTEP01_SET_PARAM, POWERSTEP01_GET_PARAM, 
     * POWERSTEP01_GET_STATUS).
     * @param value argument of the command to queue.
     * @retval None.
     */
    virtual void QueueCommands(uint8_t command, int32_t value)
    {
        Powerstep01_QueueCommands(command, value);
    }
    
    /**
     * @brief  Reading the Status Register.
     * @param  None.
     * @retval None.
     * @note   The Status Register's flags are not cleared, contrary to GetStatus().
     */
    virtual uint16_t ReadStatusRegister(void)
    {
        return Powerstep01_ReadStatusRegister();
    }
    
    /**
     * @brief  Issues PowerStep01 Release SW command.
     * @param  action type of action to undertake when the SW
     * input is forced high (ACTION_RESET or ACTION_COPY).
     * @param  direction The direction of rotation.
     * @param  speed in steps/s.
     * @retval One if the device has an alarm flag set (low logic level on flag output),
     * otherwise zero
     */  
    virtual void ReleaseSw(motorAction_t action, direction_t direction)
    {
        Powerstep01_CmdReleaseSw(action,
                                 (motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD));
    }
    
    /**
     * @brief  Issues PowerStep01 Reset Device command.
     * @param  None.
     * @retval None.
     */
    virtual void ResetCommand(void)
    {
        Powerstep01_CmdResetDevice();
    }

    /**
     * @brief  Issues PowerStep01 ResetPos command.
     * @param  None.
     * @retval None.
     * @note   Same effect as SetHome().
     */
    virtual void ResetPosition(void)
    {
        Powerstep01_CmdResetPos();
    }

    /**
     * @brief  Running the motor towards a specified direction.
     * @param  direction The direction of rotation.
     * @param  speed in steps/s.
     * @retval None.
     */
    virtual void Run(direction_t direction, float speed)
    {
        Powerstep01_CmdRun((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD), Speed_Steps_s_to_RegVal(speed));
    }
        
    /**
     * @brief Sends commands stored previously in the queue by QueueCommands.
     * @param  None.
     * @retval None.
     */ 
    virtual void SendQueuedCommands(void)
    {
        Powerstep01_SendQueuedCommands();
    }
    
    /**
     * @brief  Setting a parameter with an input float value.
     * @param  param Register adress.
     * @param  value Float value to convert and set into the register.
     * @retval TRUE if param and value are valid, FALSE otherwise
     * @note   parameter can be one of the following:
     *           + POWERSTEP01_EL_POS
     *           + POWERSTEP01_ABS_POS
     *           + POWERSTEP01_MARK
     *           + POWERSTEP01_ACC
     *           + POWERSTEP01_DEC
     *           + POWERSTEP01_MAX_SPEED
     *           + POWERSTEP01_MIN_SPEED
     *           + POWERSTEP01_FS_SPD
     *           + POWERSTEP01_INT_SPD
     *           + POWERSTEP01_K_THERM
     *           + POWERSTEP01_OCD_TH
     *           + POWERSTEP01_STALL_TH
     *           + POWERSTEP01_KVAL_HOLD
     *           + POWERSTEP01_KVAL_RUN
     *           + POWERSTEP01_KVAL_ACC
     *           + POWERSTEP01_KVAL_DEC
     *           + POWERSTEP01_ST_SLP
     *           + POWERSTEP01_FN_SLP_ACC
     *           + POWERSTEP01_FN_SLP_DEC
     *           + POWERSTEP01_TVAL_HOLD
     *           + POWERSTEP01_TVAL_RUN
     *           + POWERSTEP01_TVAL_ACC
     *           + POWERSTEP01_TVAL_DEC
     *           + POWERSTEP01_TON_MIN
     *           + POWERSTEP01_TOFF_MIN
     */
    virtual bool SetAnalogValue(unsigned int param, float value)
    {
      return Powerstep01_SetAnalogValue((powerstep01_Registers_t)param, value);
    }
    
    /**
     * @brief  Setting a parameter.
     * @param  parameter A parameter's register adress.
     * @param  value The parameter's value.
     * @retval None.
     *         parameter can be one of the following:
     *           + POWERSTEP01_ABS_POS
     *           + POWERSTEP01_EL_POS
     *           + POWERSTEP01_MARK
     *           + POWERSTEP01_ACC
     *           + POWERSTEP01_DEC
     *           + POWERSTEP01_MAX_SPEED
     *           + POWERSTEP01_MIN_SPEED
     *           (voltage mode) + POWERSTEP01_KVAL_HOLD : value in %
     *           (current mode) + POWERSTEP01_TVAL_HOLD : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_RUN  : value in %
     *           (current mode) + POWERSTEP01_TVAL_RUN  : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_ACC  : value in %
     *           (current mode) + POWERSTEP01_TVAL_ACC  : value in mV
     *           (voltage mode) + POWERSTEP01_KVAL_DEC  : value in %
     *           (current mode) + POWERSTEP01_TVAL_DEC  : value in mV
     *           (voltage mode) + POWERSTEP01_INT_SPD
     *           (voltage mode) + POWERSTEP01_ST_SLP
     *           (current mode) + POWERSTEP01_T_FAST
     *           (voltage mode) + POWERSTEP01_FN_SLP_ACC
     *           (current mode) + POWERSTEP01_TON_MIN
     *           (voltage mode) + POWERSTEP01_FN_SLP_DEC
     *           (current mode) + POWERSTEP01_TOFF_MIN
     *           (voltage mode) + POWERSTEP01_K_THERM
     *           + POWERSTEP01_ADC_OUT
     *           + POWERSTEP01_OCD_TH
     *           (voltage mode) + POWERSTEP01_STALL_TH
     *           + POWERSTEP01_FS_SPD
     *           + POWERSTEP01_STEP_MODE
     *           + POWERSTEP01_ALARM_EN
     *           + POWERSTEP01_GATECFG1
     *           + POWERSTEP01_GATECFG2
     *           + POWERSTEP01_CONFIG
     */
    virtual void SetRawParameter(unsigned int parameter, unsigned int value)
    {
        Powerstep01_CmdSetParam((powerstep01_Registers_t)parameter, (uint32_t)value);
    }
    
    /**
     * @brief  Enable the step clock mode.
     * @param  frequency the frequency of PWM.
     * @retval None.
     */
    virtual void StepClockModeEnable(direction_t direction)
    {
        Powerstep01_CmdStepClock((motorDir_t) (direction == StepperMotor::FWD ? FORWARD : BACKWARD));
    }
    
    /**
     * @brief  Setting the frequency of PWM.
     *         The frequency controls directly the speed of the device.
     * @param  frequency the frequency of PWM.
     * @retval None.
     */
    virtual void StepClockStart(void)//uint16_t frequency)
    {
        /* Computing the period of PWM. */
        //double period = 1.0f / frequency;
        
        /* Setting the period and the duty-cycle of PWM. */
       // pwm.period(period);
       // pwm.write(0.5f);
    }

    /**
     * @brief  Stopping the PWM.
     * @param  None.
     * @retval None.
     */
    virtual void StepClockStop(void)
    {
        //pwm.write(0.0f);
    }
    
    /**
     * @brief Public static functions
     */    

    static uint8_t GetNbDevices(void)
    {
        return numberOfDevices;
    }

    /**
     * @brief To and from register parameter conversion functions
     */
     
    /**********************************************************
     * @brief Convert the float formatted acceleration or
     * deceleration into respectively an ACC or DEC register value
     * @param[in] steps_s2 the acceleration or deceleration as
     * steps/s^2, range 14.55 to 59590 steps/s^2
     * @retval The acceleration or deceleration as steps/tick^2
     **********************************************************/
    static uint16_t AccDec_Steps_s2_to_RegVal(float steps_s2)
    {
        return ((uint16_t)(((float)(steps_s2)*0.068719476736f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert the ACC or DEC register value into step/s^2
     * @param[in] regVal The ACC or DEC register value
     * @retval The speed as steps/s
     **********************************************************/
    static float AccDec_RegVal_to_Steps_s2(uint32_t regVal)
    {
        return (((float)(regVal))*14.5519152283f);
    }
    
    /**********************************************************
     * @brief Converts BEMF compensation slope to values for ST_SLP,
     * FN_SLP_ACC or FN_SLP_DEC register
     * @param[in] percentage BEMF compensation slope percentage,
     * range 0 to 0.4% (0.004) s/step
     * @retval value for ST_SLP, FN_SLP_ACC or FN_SLP_DEC register
     **********************************************************/
    static uint8_t BEMFslope_Perc_to_RegVal(float percentage)
    {
        return ((uint8_t)(((float)(percentage)*637.5f)+0.5f));
    }
    
    /**********************************************************
     * @brief Converts values from ST_SLP, FN_SLP_ACC or
     * FN_SLP_DEC register to BEMF compensation slope percentage
     * @param[in] regVal The ST_SLP, FN_SLP_ACC or FN_SLP_DEC
     * register value
     * @retval BEMF compensation slope percentage
     **********************************************************/
    static float BEMFslope_RegVal_to_Perc(uint32_t regVal)
    {
        return (((float)(regVal))*0.00156862745098f);
    }
          
    /**********************************************************
     * @brief Convert the float formatted speed into a FS_SPD 
     * register value
     * @param[in] steps_s the speed as steps/s, range 15.25 to 15610 steps/s
     * @retval The speed as steps/tick
     **********************************************************/
    static uint16_t FSSpd_Steps_s_to_RegVal(float steps_s)
    {
        return ((uint16_t)((float)(steps_s)*0.065536f));
    }
    
    /**********************************************************
     * @brief Convert the FS_SPD register value into step/s
     * @param[in] regVal The FS_SPD register value
     * @retval The full Step speed as steps/s
     **********************************************************/
    static float FSSpd_RegVal_to_Steps_s(uint32_t regVal)
    {
        return (((float)regVal+0.999f)*15.258789f);
    }
    
    /**********************************************************
     * @brief Convert the float formatted speed into a INT_SPEED 
     * register value
     * @param[in] steps_s the speed as steps/s, range 0 to 976.5 steps/s
     * @retval The intersect speed as steps/tick
     **********************************************************/
    static uint16_t IntSpd_Steps_s_to_RegVal(float steps_s)
    {
        return ((uint16_t)(((float)(steps_s)*16.777216f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert the INT_SPEED register value into step/s
     * @param[in] regVal The INT_SPEED register value
     * @retval The speed as steps/s
     **********************************************************/
    static float IntSpd_RegVal_to_Steps_s(uint32_t regVal)
    {
        return (((float)(regVal))*0.0596045f);
    }
    
    /**********************************************************
     * @brief Convert the float formatted thermal compensation
     * factor into a K_THEM register value
     * @param[in] compFactor the float formatted thermal 
     * compensation factor, range 1 to 1.46875
     * @retval value for K_THERM register
     **********************************************************/
    static uint8_t KTherm_Comp_to_RegVal(float compFactor)
    {
        return ((uint8_t)((((float)(compFactor)-1.0f)*32.0f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert the K_THERM register value into a float 
     * formatted thermal compensation factor
     * @param[in] regVal The K_THERM register value
     * @retval The float formatted thermal compensation factor
     **********************************************************/
    static float KTherm_RegVal_to_Comp(uint32_t regVal)
    {
        return (((float)(regVal))*0.03125f+1);
    }
    
    /**********************************************************
     * @brief Converts voltage in percentage to values for KVAL_RUN,
     * KVAL_HOLD, KVAL_ACC or KVAL_DEC register
     * @param[in] percentage percentage of the power supply voltage
     * applied to the motor windings, range 0.4% to 99.6%
     * @retval value for KVAL_RUN, KVAL_HOLD, KVAL_ACC or
     * KVAL_DEC register
     * @note The voltage applied is sinusoidal
     **********************************************************/
    static uint8_t Kval_Perc_to_RegVal(float percentage)
    {
        return ((uint8_t)(((float)(percentage)*2.56f)+0.5f));
    }
    
    /**********************************************************
     * @brief Converts values from KVAL_RUN, KVAL_HOLD, KVAL_ACC
     * or KVAL_DEC register to percentage
     * @param[in] regVal The KVAL_RUN, KVAL_HOLD, KVAL_ACC
     * or KVAL_DEC register value
     * @retval percentage of the power supply voltage applied to
     * the motor windings
     * @note The voltage applied is sinusoidal
     **********************************************************/
    static float Kval_RegVal_to_Perc(uint32_t regVal)
    {
        return (((float)(regVal))*0.390625f);
    }
    
    /**********************************************************
     * @brief Convert the float formatted speed into a MAX_SPEED 
     * register value
     * @param[in] steps_s the speed as steps/s, range 15.25 to 15610 steps/s
     * @retval The speed as steps/tick
     **********************************************************/
    static uint16_t MaxSpd_Steps_s_to_RegVal(float steps_s)
    {
        return ((uint16_t)(((float)(steps_s)*0.065536f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert the MAX_SPEED register value into step/s
     * @param[in] regVal The MAX_SPEED register value
     * @retval The speed as steps/s
     **********************************************************/
    static float MaxSpd_RegVal_to_Steps_s(uint32_t regVal)
    {
        return (((float)(regVal))*15.258789f);
    }
    
    /**********************************************************
     * @brief Convert the float formatted speed into a MIN_SPEED 
     * register value
     * @param[in] steps_s the speed as steps/s, range 0 to 976.3 steps/s
     * @retval The speed as steps/tick
     **********************************************************/
    static uint16_t MinSpd_Steps_s_to_RegVal(float steps_s)
    {
        return ((uint16_t)(((float)(steps_s)*4.194304f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert the MIN_SPEED register value into step/s
     * @param[in] regVal The MIN_SPEED register value
     * @retval The speed as steps/s
     **********************************************************/
    static float MinSpd_RegVal_to_Steps_s(uint32_t regVal)
    {
        return (((float)(regVal))*0.238418579f);
    }
    
    /**********************************************************
     * @brief Convert the float formatted speed into a SPEED 
     * register value
     * @param[in] steps_s the speed as steps/s, range 0 to 15625 steps/s
     * @retval The speed as steps/tick
     **********************************************************/
    static uint32_t Speed_Steps_s_to_RegVal(float steps_s)
    {
        return ((uint32_t)(((float)(steps_s)*67.108864f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert the SPEED register value into step/s
     * @param[in] regVal The SPEED register value
     * @retval The speed as steps/s
     **********************************************************/
    static float Speed_RegVal_to_Steps_s(uint32_t regVal)
    {
        return (((float)(regVal))*0.01490116119f);
    }
    
    /**********************************************************
     * @brief Converts STALL or OCD Threshold voltage in mV to 
     * values for STALL_TH or OCD_TH register
     * @param[in] mV voltage in mV, range 31.25mV to 1000mV
     * @retval value for STALL_TH or OCD_TH register
     **********************************************************/
    static uint8_t StallOcd_Th_to_RegVal(float mV)
    {
        return ((uint8_t)((((float)(mV)-31.25f)*0.032f)+0.5f));
    }
    
    /**********************************************************
     * @brief Converts values from STALL_TH or OCD_TH register 
     * to mV
     * @param[in] regVal The STALL_TH or OCD_TH register value
     * @retval voltage in mV
     **********************************************************/
    static float StallOcd_RegVal_to_Th(uint32_t regVal)
    {
        return (((float)(regVal+1))*31.25f);
    }
    
    /**********************************************************
     * @brief Converts voltage in mV to values for TVAL_RUN,
     * TVAL_HOLD, TVAL_ACC or TVAL_DEC register
     * @param[in] voltage_mV voltage in mV, range 7.8mV to 1000mV
     * @retval value for TVAL_RUN, TVAL_HOLD, TVAL_ACC or
     * TVAL_DEC register
     * @note The voltage corresponds to a peak output current
     * accross the external sense power resistor
     **********************************************************/
    static uint8_t Tval_RefVoltage_to_RegVal(float voltage_mV)
    {
        return ((uint8_t)((((float)(voltage_mV)-7.8125f)*0.128f)+0.5f));
    }
    
    /**********************************************************
     * @brief Converts values from TVAL_RUN, TVAL_HOLD, TVAL_ACC
     * or TVAL_DEC register to mV
     * @param[in] regVal The TVAL_RUN, TVAL_HOLD, TVAL_ACC
     * or TVAL_DEC register value
     * @retval voltage in mV
     * @note The voltage corresponds to a peak output current
     * accross the external sense power resistor
     **********************************************************/
    static float Tval_RegVal_to_RefVoltage(uint32_t regVal)
    {
        return (((float)(regVal+1))*7.8125f);
    }
    
    /**********************************************************
     * @brief Convert time in us to values for TON_MIN register
     * @param[in] tmin_us time in us, range 0.5us to 64us
     * @retval value for TON_MIN register
     **********************************************************/
    static uint8_t Tmin_Time_to_RegVal(float tmin_us)
    {
        return ((uint8_t)((((float)(tmin_us)-0.5f)*2.0f)+0.5f));
    }
    
    /**********************************************************
     * @brief Convert values for TON_MIN or TOFF_MIN register to time in us
     * @param[in] regVal The TON_MIN or TOFF_MIN register value
     * @retval time in us
     **********************************************************/
    static float Tmin_RegVal_to_Time(uint32_t regVal)
    {
        return (((float)(regVal+1))*0.5f);
    }
    
    /*** Public Interrupt Related Methods ***/

    /* ACTION 6 --------------------------------------------------------------*
     * Implement here interrupt related methods, if any.                      *
     * Note that interrupt handling is platform dependent, e.g.:              *
     *   + mbed:                                                              *
     *     InterruptIn feature_irq(pin); //Interrupt object.                  *
     *     feature_irq.rise(callback);   //Attach a callback.                 *
     *     feature_irq.mode(PullNone);   //Set interrupt mode.                *
     *     feature_irq.enable_irq();     //Enable interrupt.                  *
     *     feature_irq.disable_irq();    //Disable interrupt.                 *
     *   + Arduino:                                                           *
     *     attachInterrupt(pin, callback, RISING); //Attach a callback.       *
     *     detachInterrupt(pin);                   //Detach a callback.       *
     *                                                                        *
     * Example (mbed):                                                        *
     *   void AttachFeatureIRQ(void (*fptr) (void))                           *
     *   {                                                                    *
     *     feature_irq.rise(fptr);                                            *
     *   }                                                                    *
     *                                                                        *
     *   void EnableFeatureIRQ(void)                                          *
     *   {                                                                    *
     *     feature_irq.enable_irq();                                          *
     *   }                                                                    *
     *                                                                        *
     *   void DisableFeatureIRQ(void)                                         *
     *   {                                                                    *
     *     feature_irq.disable_irq();                                         *
     *   }                                                                    *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Attaching an interrupt handler to the FLAG interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void AttachFlagIRQ(void)//(*fptr)(void))
    {
        // flag_irq.fall(fptr);
    }
    
    /**
     * @brief  Enabling the FLAG interrupt handling.
     * @param  None.
     * @retval None.
     */
    void EnableFlagIRQ(void)
    {
        //flag_irq.enable_irq();
    }
    
    /**
     * @brief  Disabling the FLAG interrupt handling.
     * @param  None.
     * @retval None.
     */
    void DisableFlagIRQ(void)
    {
        //flag_irq.disable_irq();
    }

    /**
     * @brief  Attaching an interrupt handler to the BUSY interrupt.
     * @param  fptr An interrupt handler.
     * @retval None.
     */
    void AttachBusyIRQ(void )//(*fptr)(void))
    {
        // busy_irq.fall(fptr);
    }
    
    /**
     * @brief  Enabling the BUSY interrupt handling.
     * @param  None.
     * @retval None.
     */
    void EnableBusyIRQ(void)
    {
        // busy_irq.enable_irq();
    }

    /**
     * @brief  Disabling the BUSY interrupt handling.
     * @param  None.
     * @retval None.
     */
    void DisableBusyIRQ(void)
    {
        // busy_irq.disable_irq();
    }
    
protected:

    /*** Protected Component Related Methods ***/

    /* ACTION 7 --------------------------------------------------------------*
     * Declare here the component's specific methods.                         *
     * They should be:                                                        *
     *   + Methods with the same name of the C component's virtual table's    *
     *     functions (1);                                                     *
     *   + Methods with the same name of the C component's extended virtual   *
     *     table's functions, if any (2);                                     *
     *   + Helper methods, if any, like functions declared in the component's *
     *     source files but not pointed by the component's virtual table (3). *
     *                                                                        *
     * Example:                                                               *
     *   Status_t COMPONENT_GetValue(float *f);   //(1)                       *
     *   Status_t COMPONENT_EnableFeature(void);  //(2)                       *
     *   Status_t COMPONENT_ComputeAverage(void); //(3)                       *
     *------------------------------------------------------------------------*/
    Status_t Powerstep01_Init(void *init);
    Status_t Powerstep01_ReadID(uint8_t *id);
    void Powerstep01_AttachErrorHandler(void (*callback)(uint16_t error));
    uint8_t Powerstep01_CheckBusyHw(void);
    uint8_t Powerstep01_CheckStatusHw(void);
    uint16_t Powerstep01_CmdGetStatus(void);
    void Powerstep01_CmdGoHome(void);
    void Powerstep01_CmdGoMark(void);
    void Powerstep01_CmdGoTo(int32_t targetPosition);
    void Powerstep01_CmdGoToDir(motorDir_t direction, int32_t targetPosition);
    void Powerstep01_CmdGoUntil(motorAction_t action, motorDir_t direction, uint32_t speed);
    void Powerstep01_CmdHardHiZ(void);
    void Powerstep01_CmdHardStop(void);
    void Powerstep01_CmdMove(motorDir_t direction, uint32_t stepCount);
    void Powerstep01_CmdNop(void);
    void Powerstep01_CmdReleaseSw(motorAction_t action, motorDir_t direction);
    void Powerstep01_CmdResetDevice(void);
    void Powerstep01_CmdResetPos(void);
    void Powerstep01_CmdRun(motorDir_t direction, uint32_t speed);
    void Powerstep01_CmdSoftHiZ(void);
    void Powerstep01_CmdSoftStop(void);
    void Powerstep01_CmdStepClock(motorDir_t direction);
    void Powerstep01_ErrorHandler(uint16_t error);
    void Powerstep01_FetchAndClearAllStatus(void); 
    uint16_t Powerstep01_GetFetchedStatus(void);
    uint32_t Powerstep01_GetFwVersion(void);
    int32_t Powerstep01_GetMark(void);
    int32_t Powerstep01_GetPosition(void);
    bool Powerstep01_IsDeviceBusy(void);
    uint16_t Powerstep01_ReadStatusRegister(void);
    bool Powerstep01_SelectStepMode(motorStepMode_t stepMode);
    void Powerstep01_SetHome(void);
    void Powerstep01_SetMark(void);
    void Powerstep01_WaitForAllDevicesNotBusy(void);
    void Powerstep01_WaitWhileActive(void);

    /**
     * @brief To and from register parameter conversion functions
     */
    int32_t Powerstep01_ConvertPosition(uint32_t abs_position_reg);

    /**
     * @brief Functions to initialize the registers
     */
    void Powerstep01_SetDeviceParamsToGivenValues(powerstep01_Init_u_t *initPrm);
    void Powerstep01_SetRegisterToPredefinedValues(void);
    
    /**
     * @brief Functions to get and set parameters using digital or analog values
     */
    uint32_t Powerstep01_CmdGetParam(powerstep01_Registers_t param);
    void Powerstep01_CmdSetParam(powerstep01_Registers_t param, uint32_t value);
    float Powerstep01_GetAnalogValue(powerstep01_Registers_t param);
    void Powerstep01_QueueCommands(uint8_t command, int32_t value);
    void Powerstep01_SendCommand(uint8_t command, uint32_t value);
    void Powerstep01_SendQueuedCommands(void);
    bool Powerstep01_SetAnalogValue(powerstep01_Registers_t param, float value);
    void Powerstep01_WriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte);    
    
    /**
     * @brief      Rounding a floating point number to the nearest unsigned integer number.
     * @param  f The floating point number.
     * @retval     The nearest unsigned integer number.
     */
    int round(float f)
    {
        if (f >= 0)
            return (int) f + (f - (int) f < 0.5f ? 0 : 1);
        else
            return (int) f - (f - (int) f < -0.5f ? 1 : 0);
    }
    
    /*** Component's I/O Methods ***/

    /**
     * @brief      Utility function to read data from Powerstep01.
     * @param[out] pBuffer pointer to the buffer to read data into.
     * @param  NumBytesToRead number of bytes to read.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    Status_t Read(uint8_t* pBuffer, uint16_t NumBytesToRead)
    {
        if (dev_spi.spi_read(pBuffer, ssel, NumBytesToRead) != 0)
            return COMPONENT_ERROR;
        return COMPONENT_OK;
    }
    
    /**
     * @brief      Utility function to write data to Powerstep01.
     * @param  pBuffer pointer to the buffer of data to send.
     * @param  NumBytesToWrite number of bytes to write.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    Status_t Write(uint8_t* pBuffer, uint16_t NumBytesToWrite)
    {
        if (dev_spi.spi_write(pBuffer, ssel, NumBytesToWrite) != 0)
            return COMPONENT_ERROR;
        return COMPONENT_OK;
    }

    /**
     * @brief      Utility function to read and write data from/to Powerstep01 at the same time.
     * @param[out] pBufferToRead pointer to the buffer to read data into.
     * @param  pBufferToWrite pointer to the buffer of data to send.
     * @param  NumBytes number of bytes to read and write.
     * @retval     COMPONENT_OK in case of success, COMPONENT_ERROR otherwise.
     */
    Status_t ReadWrite(uint8_t* pBufferToRead, uint8_t* pBufferToWrite, uint16_t NumBytes)
    {
        if (dev_spi.spi_read_write(pBufferToRead, pBufferToWrite, ssel, NumBytes) != 0)
            return COMPONENT_ERROR;
        return COMPONENT_OK;
    }

    /* ACTION 8 --------------------------------------------------------------*
     * Implement here other I/O methods beyond those already implemented      *
     * above, which are declared extern within the component's header file.   *
     *------------------------------------------------------------------------*/
    /**
     * @brief  Making the CPU wait.
     * @param  None.
     * @retval None.
     */
    void Powerstep01_Board_Delay(uint32_t delay)
    {
        thread_sleep_for(delay);
    }

    /**
     * @brief  Enabling interrupts.
     * @param  None.
     * @retval None.
     */
    void Powerstep01_Board_EnableIrq(void)
    {
        __enable_irq();
    }

    /**
     * @brief  Disabling interrupts.
     * @param  None.
     * @retval None.
     */
    void Powerstep01_Board_DisableIrq(void)
    {
        __disable_irq();
    }

    /**
     * @brief  Initialising the PWM.
     * @param  None.
     * @retval None.
     */
    void Powerstep01_Board_StepClockInit(void) {}

    /**
     * @brief  Exit the device from reset mode.
     * @param  None.
     * @retval None.
     */
    void Powerstep01_Board_ReleaseReset(void)
    {
        standby_reset = 1;
    }

    /**
     * @brief  Put the device in reset mode.
     * @param  None.
     * @retval None.
     */
    void Powerstep01_Board_Reset(void)
    {
        standby_reset = 0;
    }

    /**
     * @brief      Writing and reading bytes to/from the component through the SPI at the same time.
     * @param  pByteToTransmit pointer to the buffer of data to send.
     * @param[out] pReceivedByte pointer to the buffer to read data into.
     * @retval     "0" in case of success, "1" otherwise.
     */
    uint8_t Powerstep01_Board_SpiWriteBytes(uint8_t *pByteToTransmit, uint8_t *pReceivedByte)
    {
        return (uint8_t) (ReadWrite(pReceivedByte, pByteToTransmit, numberOfDevices) == COMPONENT_OK ? 0 : 1);
    }


    /*** Component's Instance Variables ***/

    /* ACTION 9 --------------------------------------------------------------*
     * Declare here interrupt related variables, if needed.                   *
     * Note that interrupt handling is platform dependent, see                *
     * "Interrupt Related Methods" above.                                     *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     InterruptIn feature_irq;                                           *
     *------------------------------------------------------------------------*/
    /* Flag Interrupt. */
    //InterruptIn flag_irq;
    
    /* Busy Interrupt. */
    //InterruptIn busy_irq;

    /* ACTION 10 -------------------------------------------------------------*
     * Declare here other pin related variables, if needed.                   *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     DigitalOut standby_reset;                                          *
     *------------------------------------------------------------------------*/
    /* Standby/reset pin. */
    DigitalOut standby_reset;

    /* Pulse Width Modulation pin. */
    //PwmOut pwm;

    /* ACTION 11 -------------------------------------------------------------*
     * Declare here communication related variables, if needed.               *
     *                                                                        *
     * Example:                                                               *
     *   + mbed:                                                              *
     *     DigitalOut ssel;                                                   *
     *     DevSPI &dev_spi;                                                   *
     *------------------------------------------------------------------------*/
    /* Configuration. */
    DigitalOut ssel;

    /* IO Device. */
    DevSPI &dev_spi;

    /* ACTION 12 -------------------------------------------------------------*
     * Declare here identity related variables, if needed.                    *
     * Note that there should be only a unique identifier for each component, *
     * which should be the "who_am_i" parameter.                              *
     *------------------------------------------------------------------------*/
    /* Identity */
    uint8_t who_am_i;

    /* ACTION 13 -------------------------------------------------------------*
     * Declare here the component's static and non-static data, one variable  *
     * per line.                                                              *
     *                                                                        *
     * Example:                                                               *
     *   float measure;                                                       *
     *   int instance_id;                                                     *
     *   static int number_of_instances;                                      *
     *------------------------------------------------------------------------*/
    /* Data. */
    void (*errorHandlerCallback)(uint16_t error);
    uint8_t deviceInstance;

    /* Static data. */
    static uint8_t numberOfDevices;
    static uint8_t spiTxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];
    static uint8_t spiRxBursts[POWERSTEP01_CMD_ARG_MAX_NB_BYTES][MAX_NUMBER_OF_DEVICES];


public:

    /* Static data. */
    static bool spiPreemptionByIsr;
    static bool isrFlag;
    
};

#endif // __POWERSTEP01_CLASS_H

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
