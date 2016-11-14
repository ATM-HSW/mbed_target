/**
  ******************************************************************************
  * @file    motor.h
  * @author  IPC Rennes
  * @version V1.6.0
  * @date    April 7th, 2016
  * @brief   This file contains all the functions prototypes for motor drivers.   
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
#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "component.h"

/* Definitions ---------------------------------------------------------------*/
/// boolean for false condition 
#ifndef FALSE
#define FALSE (0)
#endif
/// boolean for true condition 
#ifndef TRUE
#define TRUE  (1)
#endif

/* Types ---------------------------------------------------------------------*/
/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */ 

/** @defgroup Motor Motor
  * @{
  */
   
/** @defgroup Motor_Exported_Types Motor Exported Types
  * @{
  */
   
/** @defgroup Device_Direction_Options Device Direction Options
  * @{
  */
/// Direction options
typedef enum {
  BACKWARD = 0,
  FORWARD = 1,
  UNKNOW_DIR = ((uint8_t)0xFF)
} motorDir_t;
/**
  * @}
  */
  
/** @defgroup Device_Action_Options Device Action Options
  * @{
  */
/// Action options
typedef enum {
  ACTION_RESET = ((uint8_t)0x00),
  ACTION_COPY  = ((uint8_t)0x08)
} motorAction_t;
/**
  * @}
  */  

/** @defgroup Device_States Device States
  * @{
  */
/// Device states
typedef enum {
  ACCELERATING       = 0,
  DECELERATINGTOSTOP = 1,  
  DECELERATING       = 2, 
  STEADY             = 3,
  INDEX_ACCEL        = 4,
  INDEX_RUN          = 5,
  INDEX_DECEL        = 6,
  INDEX_DWELL        = 7,
  INACTIVE           = 8,
  STANDBY            = 9,
  STANDBYTOINACTIVE  = 10
} motorState_t;
/**
  * @}
  */   

/** @defgroup Device_Step_mode Device Step mode
  * @{
  */
 /// Stepping options 
typedef enum {
  STEP_MODE_FULL   = ((uint8_t)0x00), 
  STEP_MODE_HALF   = ((uint8_t)0x01),
  STEP_MODE_1_4    = ((uint8_t)0x02),
  STEP_MODE_1_8    = ((uint8_t)0x03),
  STEP_MODE_1_16   = ((uint8_t)0x04),
  STEP_MODE_1_32   = ((uint8_t)0x05),
  STEP_MODE_1_64   = ((uint8_t)0x06),
  STEP_MODE_1_128  = ((uint8_t)0x07),
  STEP_MODE_1_256  = ((uint8_t)0x08),
  STEP_MODE_UNKNOW = ((uint8_t)0xFE),
  STEP_MODE_WAVE   = ((uint8_t)0xFF)  
} motorStepMode_t;

/**
  * @}
  */
  
/** @defgroup Decay_mode Decay mode
  * @{
  */
/// Decay Mode 
typedef enum {
  SLOW_DECAY = 0,
  FAST_DECAY = 1,
  UNKNOW_DECAY = ((uint8_t)0xFF)
} motorDecayMode_t;
/**
  * @}
  */
  
/** @defgroup Stop_mode Stop mode
  * @{
  */
/// Stop mode
typedef enum
{ 
  HOLD_MODE = 0,
  HIZ_MODE = 1,
  STANDBY_MODE = 2,
  UNKNOW_STOP_MODE = ((uint8_t)0xFF)
} motorStopMode_t;
/**
  * @}
  */  

/** @defgroup Torque_mode Torque mode
  * @{
  */
/// Torque mode
typedef enum
{ 
  ACC_TORQUE = 0,
  DEC_TORQUE = 1,
  RUN_TORQUE = 2,
  HOLD_TORQUE = 3,
  CURRENT_TORQUE = 4,
  UNKNOW_TORQUE = ((uint8_t)0xFF)
} motorTorqueMode_t;
/**
  * @}
  */  
    
/** @defgroup Dual_Full_Bridge_Configuration Dual Full Bridge Configuration
  * @{
  */
///Dual full bridge configurations for brush DC motors
typedef enum {
  PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 0,
  PARALLELING_NONE___1_BIDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 1,
  PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 2,
  PARALLELING_NONE___2_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 3,
  PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__1_BIDIR_MOTOR_BRIDGE_B = 4,
  PARALLELING_IN1A_IN2A__1_UNDIR_MOTOR_BRIDGE_A__2_UNDIR_MOTOR_BRIDGE_B = 5,
  PARALLELING_IN1B_IN2B__1_BIDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 6,
  PARALLELING_IN1B_IN2B__2_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 7,
  PARALLELING_IN1A_IN2A__IN1B_IN2B__1_UNDIR_MOTOR_BRIDGE_A__1_UNDIR_MOTOR_BRIDGE_B = 8,
  PARALLELING_IN1A_IN2A__IN1B_IN2B__1_BIDIR_MOTOR = 9,
  PARALLELING_IN1A_IN1B__IN2A_IN2B__1_UNDIR_MOTOR_BRIDGE_1A__1_UNDIR_MOTOR_BRIDGE_2A = 10,
  PARALLELING_IN1A_IN1B__IN2A_IN2B__1_BIDIR_MOTOR = 11,
  PARALLELING_ALL_WITH_IN1A___1_UNDIR_MOTOR = 12,
  PARALLELING_END_ENUM = 13 
} dualFullBridgeConfig_t;
/**
  * @}
  */

/** @defgroup Motor_Driver_Structure Motor Driver Structure
  * @{
  */
/** 
 * @brief  MOTOR driver virtual table structure definition.
 */  
typedef struct
{
  /* ACTION ----------------------------------------------------------------*
   * Declare here the component's generic functions.                        *
   * Tag this group of functions with the " Generic " C-style comment.      *
   * A component's interface has to define at least the two generic         *
   * functions provided here below within the "Example" section, as the     *
   * first and second functions of its Virtual Table. They have to be       *
   * specified exactly in the given way.                                    *
   *                                                                        *
   * Example:                                                               *
   *   Status_t (*Init)   (void *handle, void *init);                       *
   *   Status_t (*ReadID) (void *handle, uint8_t *id);                      *
   *------------------------------------------------------------------------*/
  /* Generic */
  Status_t (*Init)(void *handle, void *init);
  Status_t (*ReadID)(void *handle, uint8_t *id);
  /* ACTION ----------------------------------------------------------------*
   * Declare here the component's interrupts related functions.             *
   * Tag this group of functions with the " Interrupts " C-style comment.   *
   * Do not specify any function if not required.                           *
   *                                                                        *
   * Example:                                                               *
   *   void     (*ConfigIT) (void *handle, int a);                          *
   *------------------------------------------------------------------------*/
  /* Interrupts */
  /// Function pointer to AttachErrorHandler
  void (*AttachErrorHandler)(void *handle, void (*callback)(void *handle, uint16_t error));
  /// Function pointer to AttachFlagInterrupt
  void (*AttachFlagInterrupt)(void *handle, void (*callback)(void *handle));
  /// Function pointer to AttachBusyInterrupt
  void (*AttachBusyInterrupt)(void *handle, void (*callback)(void *handle));
  /// Function pointer to FlagInterruptHandler
  void (*FlagInterruptHandler)(void *handle);
  /* ACTION ----------------------------------------------------------------*
   * Declare here the component's specific functions.                       *
   * Tag this group of functions with the " Specific " C-style comment.     *
   * Do not specify any function if not required.                           *
   *                                                                        *
   * Example:                                                               *
   *   Status_t (*GetValue) (void *handle, float *f);                       *
   *------------------------------------------------------------------------*/
  /* Specific */  
  /// Function pointer to GetAcceleration
  uint16_t (*GetAcceleration)(void *handle); 
  /// Function pointer to GetCurrentSpeed
  uint16_t (*GetCurrentSpeed)(void *handle); 
  /// Function pointer to GetDeceleration
  uint16_t (*GetDeceleration)(void *handle); 
  /// Function pointer to GetDeviceState
  motorState_t(*GetDeviceState)(void *handle); 
  /// Function pointer to GetFwVersion
  uint32_t (*GetFwVersion)(void *handle); 
  /// Function pointer to GetMark
  int32_t (*GetMark)(void *handle); 
  /// Function pointer to GetMaxSpeed
  uint16_t (*GetMaxSpeed)(void *handle); 
  /// Function pointer to GetMinSpeed
  uint16_t (*GetMinSpeed)(void *handle); 
  /// Function pointer to GetPosition
  int32_t (*GetPosition)(void *handle);
  /// Function pointer to GoHome
  void (*GoHome)(void *handle); 
  /// Function pointer to GoMark
  void (*GoMark)(void *handle); 
  /// Function pointer to GoTo
  void (*GoTo)(void *handle, int32_t targetPosition); 
  /// Function pointer to HardStop
  void (*HardStop)(void *handle); 
  /// Function pointer to Move
  void (*Move)(void *handle, motorDir_t direction, uint32_t stepCount); 
  /// Function pointer to ResetAllDevices
  //void (*ResetAllDevices)(void *handle); 
  /// Function pointer to Run
  void (*Run)(void *handle, motorDir_t direction);
  /// Function pointer to SetAcceleration
  bool (*SetAcceleration)(void *handle, uint16_t newAcc);
  /// Function pointer to SetDeceleration
  bool (*SetDeceleration)(void *handle, uint16_t newDec);
  /// Function pointer to SetHome
  void (*SetHome)(void *handle); 
  /// Function pointer to SetMark
  void (*SetMark)(void *handle); 
  /// Function pointer to SetMaxSpeed
  bool (*SetMaxSpeed)(void *handle, uint16_t newMaxSpeed); 
  /// Function pointer to SetMinSpeed
  bool (*SetMinSpeed)(void *handle, uint16_t newMinSpeed); 
  /// Function pointer to SoftStop
  bool (*SoftStop)(void *handle); 
  /// Function pointer to StepClockHandler
  void (*StepClockHandler)(void *handle);  
  /// Function pointer to WaitWhileActive
  void (*WaitWhileActive)(void *handle);
  /// Function pointer to CmdDisable
  void (*CmdDisable)(void *handle); 
  /// Function pointer to CmdEnable
  void (*CmdEnable)(void *handle);
  /// Function pointer to CmdGetParam
  uint32_t (*CmdGetParam)(void *handle, uint32_t param);
  /// Function pointer to CmdGetStatus
  uint16_t (*CmdGetStatus)(void *handle); 
  /// Function pointer to CmdNop
  void (*CmdNop)(void *handle); 
  /// Function pointer to CmdSetParam
  void (*CmdSetParam)(void *handle, uint32_t param, uint32_t value);
  /// Function pointer to ReadStatusRegister
  uint16_t (*ReadStatusRegister)(void *handle); 
  /// Function pointer to ReleaseReset
  void (*ReleaseReset)(void *handle);
  /// Function pointer to Reset
  void (*Reset)(void *handle); 
  /// Function pointer to SelectStepMode
  bool (*SelectStepMode)(void *handle, motorStepMode_t);
  /// Function pointer to SetDirection
  void (*SetDirection)(void *handle, motorDir_t direction);
  /// Function pointer to CmdGoToDir
  void (*CmdGoToDir)(void *handle, motorDir_t direction, int32_t targetPosition);
  /// Function pointer to CheckBusyHw
  uint8_t (*CheckBusyHw)(void *handle);
  /// Function pointer to CheckStatusHw
  uint8_t (*CheckStatusHw)(void *handle);
  /// Function pointer to CmdGoUntil
  void (*CmdGoUntil)(void *handle, motorAction_t action, motorDir_t direction, uint32_t targetPosition);
  /// Function pointer to CmdHardHiZ
  void (*CmdHardHiZ)(void *handle);
  /// Function pointer to CmdReleaseSw
  void (*CmdReleaseSw)(void *handle, motorAction_t action, motorDir_t direction);
  /// Function pointer to CmdResetDevice
  void (*CmdResetDevice)(void *handle);
  /// Function pointer to CmdResetPos
  void (*CmdResetPos)(void *handle);
  /// Function pointer to CmdRun
  void (*CmdRun)(void *handle, motorDir_t direction, uint32_t targetPosition);
  /// Function pointer to CmdSoftHiZ
  void (*CmdSoftHiZ)(void *handle);
  /// Function pointer to CmdStepClock
  void (*CmdStepClock)(void *handle, motorDir_t direction);
  /// Function pointer to FetchAndClearAllStatus
  void (*FetchAndClearAllStatus)(void *handle);
  /// Function pointer to GetFetchedStatus
  uint16_t (*GetFetchedStatus)(void *handle);
  /// Function pointer to GetNbDevices
  uint8_t (*GetNbDevices)(void *handle);
  /// Function pointer to IsDeviceBusy
  bool (*IsDeviceBusy)(void *handle);
  /// Function pointer to SendQueuedCommands
  void (*SendQueuedCommands)(void *handle);
  /// Function pointer to QueueCommands
  void (*QueueCommands)(void *handle, uint8_t command, int32_t value);
  /// Function pointer to WaitForAllDevicesNotBusy
  void (*WaitForAllDevicesNotBusy)(void *handle);
  /// Function pointer to ErrorHandler
  void (*ErrorHandler)(void *handle, uint16_t error);
  /// Function pointer to BusyInterruptHandler
  void (*BusyInterruptHandler)(void *handle);
  /// Function pointer to CmdSoftStop
  void (*CmdSoftStop)(void *handle);
  /// Function pointer to StartStepClock
  void (*StartStepClock)(void *handle, uint16_t newFreq);
  /// Function pointer to StopStepClock
  void (*StopStepClock)(void *handle);
  /// Function pointer to SetDualFullBridgeConfig
  void (*SetDualFullBridgeConfig)(void *handle, uint8_t config);
  /// Function pointer to GetBridgeInputPwmFreq
  uint32_t (*GetBridgeInputPwmFreq)(void *handle);
  /// Function pointer to SetBridgeInputPwmFreq
  void (*SetBridgeInputPwmFreq)(void *handle, uint32_t newFreq);
  /// Function pointer to SetStopMode
  void (*SetStopMode)(void *handle, motorStopMode_t stopMode);
  /// Function pointer to GetStopMode
  motorStopMode_t (*GetStopMode)(void *handle);
  /// Function pointer to SetDecayMode
  void (*SetDecayMode)(void *handle, motorDecayMode_t decayMode);
  /// Function pointer to GetDecayMode
  motorDecayMode_t (*GetDecayMode)(void *handle);
  /// Function pointer to GetStepMode
  motorStepMode_t (*GetStepMode)(void *handle);
  /// Function pointer to GetDirection
  motorDir_t (*GetDirection)(void *handle);
  /// Function pointer to ExitDeviceFromReset
  void (*ExitDeviceFromReset)(void *handle);
  /// Function pointer to SetTorque
  void (*SetTorque)(void *handle, motorTorqueMode_t torqueMode, uint8_t torqueValue);
  /// Function pointer to GetTorque
  uint8_t (*GetTorque)(void *handle, motorTorqueMode_t torqueMode);
  /// Function pointer to SetVRefFreq
  void (*SetRefFreq)(void *handle, uint32_t newFreq);
  /// Function pointer to GetVRefFreq
  uint32_t (*GetRefFreq)(void *handle);
  /// Function pointer to SetVRefDc
  void (*SetRefDc)(void *handle, uint8_t newDc);
  /// Function pointer to GetVRefDc
  uint8_t (*GetRefDc)(void *handle);
  /// Function pointer to SetNbDevices
  bool (*SetNbDevices)(void *handle, uint8_t nbDevices);
  /// Function pointer to set a parameter
  bool (*SetAnalogValue)(void *handle, uint32_t param, float value);
  /// Function pointer to get a parameter 
  float (*GetAnalogValue)(void *handle, uint32_t param); 
} MOTOR_VTable_t;
/**
* @}
  */

/**
  * @}
  */ 

/**
  * @}
  */
  
/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif /* __MOTOR_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
