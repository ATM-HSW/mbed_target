/**
 ******************************************************************************
 * @file    powerstep01_target_config.h
 * @author  IPC Rennes
 * @version V1.2.0
 * @date    March 18th, 2016
 * @brief   Predefines values for the Powerstep01 registers
 * and for the devices parameters
 * @note    (C) COPYRIGHT 2016 STMicroelectronics
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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

#ifndef __POWERSTEP01_TARGET_CONFIG_H
#define __POWERSTEP01_TARGET_CONFIG_H

#ifdef __cplusplus
 extern "C" {
#endif 

/* Definitions ---------------------------------------------------------------*/

/** @addtogroup POWERSTEP01
  * @{
  */

/** @addtogroup Powerstep01_Exported_Defines
  * @{
  */

/** @defgroup Predefined_Powerstep01_Registers_Values Predefined Powerstep01 Registers Values
  * @{
  */

/// The maximum number of devices in the daisy chain
#define MAX_NUMBER_OF_DEVICES                 (3)

/****************************************************************************/
/* Device 0                                                                 */
/****************************************************************************/
  
/**************************** Speed Profile *********************************/
/// Register : ACC
/// Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2 
#define POWERSTEP01_CONF_PARAM_ACC_DEVICE_0 (582)

/// Register : DEC
/// Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
#define POWERSTEP01_CONF_PARAM_DEC_DEVICE_0 (582)

///Register : MAX_SPEED
/// Maximum speed in step/s, range 15.25 to 15610 steps/s
#define POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_0 (488)

/// Register : MIN_SPEED 
/// Minimum speed in step/s, range 0 to 976.3 steps/s
#define POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_0 (0)

/// Register : FS_SPD 
/// Full step speed in step/s, range 7.63 to 15625 steps/s
#define POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_0 (244.16)

/// Register : FS_SPD - field : BOOST_MODE 
/// Boost of the amplitude square wave, enum powerstep01_BoostMode_t
#define POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_0 (POWERSTEP01_BOOST_MODE_OFF)


/************************ Voltage mode parameters  **************************/
/// Register : KVAL_ACC 
/// Acceleration duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_0 (16.02)

/// Register : KVAL_DEC 
/// Deceleration duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_0 (16.02)

/// Register : KVAL_RUN 
/// Run duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_0 (16.02)

/// Register : KVAL_HOLD 
/// Hold duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_0 (16.02)

/// Register : CONFIG - field : EN_VSCOMP 
/// Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
#define POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_0 (POWERSTEP01_CONFIG_VS_COMP_DISABLE)

/// Register : MIN_SPEED - field : LSPD_OPT 
/// Low speed optimization bit, enum powerstep01_LspdOpt_t
#define POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_0 (POWERSTEP01_LSPD_OPT_OFF)

/// Register : K_THERM 
/// Thermal compensation param, range 1 to 1.46875
#define POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_0 (1)

/// Register : INT_SPEED 
/// Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
#define POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_0 (61.512)

/// Register : ST_SLP 
/// BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_0 (0.03815)

/// Register : FN_SLP_ACC 
/// BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_0 (0.06256)

/// Register : FN_SLP_DEC 
/// BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_0 (0.06256)

/// Register : CONFIG - field : F_PWM_INT 
/// PWM Frequency Integer division, enum powerstep01_ConfigFPwmInt_t
#define POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_0 (POWERSTEP01_CONFIG_PWM_DIV_2)

/// Register : CONFIG - field : F_PWM_DEC 
/// PWM Frequency Integer Multiplier, enum powerstep01_ConfigFPwmDec_t
#define POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_0 (POWERSTEP01_CONFIG_PWM_MUL_1)

/******************** Advance current control parameters  *********************/

/// Register : TVAL_ACC 
/// Acceleration torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_0  (328.12)

/// Register : TVAL_DEC 
/// Deceleration torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_0  (328.12)

/// Register : TVAL_RUN 
/// Running torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_0  (328.12)

/// Register : TVAL_HOLD 
/// Holding torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_0 (328.12) 

/// Register : CONFIG - field : EN_TQREG 
/// External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t 
#define POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_0 (POWERSTEP01_CONFIG_TQ_REG_TVAL_USED)  

/// Register : CONFIG - field : PRED_EN 
/// Predictive current enabling , enum powerstep01_ConfigPredEn_t  
#define POWERSTEP01_CONF_PARAM_PRED_DEVICE_0 (POWERSTEP01_CONFIG_PRED_DISABLE)  

///  Register : TON_MIN 
///  Minimum on-time in us, range 0.5us to 64us 
#define POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_0 (3.0)

/// Register : TOFF_MIN 
///  Minimum off-time in us, range 0.5us to 64us 
#define POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_0 (21.0)

/// Register : T_FAST - field: TOFF_FAST 
/// Maximum fast decay time , enum powerstep01_ToffFast_t
#define POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_0  (POWERSTEP01_TOFF_FAST_8us)

/// Register : T_FAST - field: FAST_STEP 
///  Maximum fall step time , enum powerstep01_FastStep_t
#define POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_0  (POWERSTEP01_FAST_STEP_12us)

/// Register : CONFIG - field : TSW 
/// Switching period, enum powerstep01_ConfigTsw_t
#define POWERSTEP01_CONF_PARAM_TSW_DEVICE_0 (POWERSTEP01_CONFIG_TSW_048us)  

/****************************** Gate Driving **********************************/

/// Register : GATECFG1 - field : IGATE 
/// Gate sink/source current via enum powerstep01_Igate_t 
#define POWERSTEP01_CONF_PARAM_IGATE_DEVICE_0      (POWERSTEP01_IGATE_64mA)

/// Register : CONFIG - field : VCCVAL 
/// VCC Val, enum powerstep01_ConfigVccVal_t 
#define POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_0 (POWERSTEP01_CONFIG_VCCVAL_15V)

/// Register : CONFIG - field : UVLOVAL 
/// UVLO Threshold via powerstep01_ConfigUvLoVal_t 
#define POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_0    (POWERSTEP01_CONFIG_UVLOVAL_LOW)

/// Register : GATECFG1 - field : TBOOST 
/// Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t 
#define POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_0     (POWERSTEP01_TBOOST_0ns)

/// Register : GATECFG1 - field : TCC 
/// Controlled current time via enum powerstep01_Tcc_t
#define POWERSTEP01_CONF_PARAM_TCC_DEVICE_0        (POWERSTEP01_TCC_500ns)

/// Duration of the blanking time via enum powerstep01_TBlank_t 
#define POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_0     (POWERSTEP01_TBLANK_375ns)

/// Register : GATECFG2 - field : TDT 
/// Duration of the dead time via enum powerstep01_Tdt_t
#define POWERSTEP01_CONF_PARAM_TDT_DEVICE_0        (POWERSTEP01_TDT_125ns)

/******************************* Others *************************************/

/// Register : OCD_TH 
/// Overcurrent threshold settings via enum powerstep01_OcdTh_t
#define POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_0 (POWERSTEP01_OCD_TH_281_25mV)

/// Register : CONFIG - field : OC_SD 
/// Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t 
#define POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_0 (POWERSTEP01_CONFIG_OC_SD_DISABLE)

/// Register : STALL_TH 
/// Stall threshold settings in mV, range 31.25mV to 1000mV 
#define POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_0 (531.25)

/// Register : ALARM_EN 
/// Alarm settings via bitmap enum powerstep01_AlarmEn_t 
#define POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_0 (POWERSTEP01_ALARM_EN_OVERCURRENT | \
                                                  POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN |  \
                                                  POWERSTEP01_ALARM_EN_THERMAL_WARNING |  \
                                                  POWERSTEP01_ALARM_EN_UVLO |  \
                                                  POWERSTEP01_ALARM_EN_STALL_DETECTION |  \
                                                  POWERSTEP01_ALARM_EN_SW_TURN_ON | \
                                                  POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD)

/// Register : CONFIG - field : SW_MODE 
/// External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t 
#define POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_0 (POWERSTEP01_CONFIG_SW_HARD_STOP)

/// Register : STEP_MODE - field : STEP_MODE 
/// Step mode settings via enum motorStepMode_t 
#define POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_0 (STEP_MODE_1_16)

/// Register : STEP_MODE - field : CM_VM 
/// Current mode or Voltage mode via enum powerstep01_CmVm_t 
#define POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_0 (POWERSTEP01_CM_VM_CURRENT)

/// Register : STEP_MODE - Field : SYNC_MODE and SYNC_EN 
/// Synch. Mode settings via enum powerstep01_SyncSel_t 
#define POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_0 (POWERSTEP01_SYNC_SEL_DISABLED)

/// Register : CONFIG - field : OSC_CLK_SEL 
/// Clock setting , enum powerstep01_ConfigOscMgmt_t 
#define POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_0 (POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ)

/// Register : GATECFG1 - field : WD_EN 
/// External clock watchdog, enum powerstep01_WdEn_t
#define POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_0 (POWERSTEP01_WD_EN_DISABLE)

/****************************************************************************/
/* Device 1                                                                 */
/****************************************************************************/
  
/**************************** Speed Profile *********************************/
/// Register : ACC
/// Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2 
#define POWERSTEP01_CONF_PARAM_ACC_DEVICE_1 (2008.16)

/// Register : DEC
/// Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
#define POWERSTEP01_CONF_PARAM_DEC_DEVICE_1 (2008.16)

///Register : MAX_SPEED
/// Maximum speed in step/s, range 15.25 to 15610 steps/s
#define POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_1 (991.82)

/// Register : MIN_SPEED 
/// Minimum speed in step/s, range 0 to 976.3 steps/s
#define POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_1 (0)

/// Register : FS_SPD 
/// Full step speed in step/s, range 7.63 to 15625 steps/s
#define POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_1 (595.09)

/// Register : FS_SPD - field : BOOST_MODE 
/// Boost of the amplitude square wave, enum powerstep01_BoostMode_t
#define POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_1 (POWERSTEP01_BOOST_MODE_OFF)


/************************ Voltage mode parameters  **************************/
/// Register : KVAL_ACC 
/// Acceleration duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_1 (16.02)

/// Register : KVAL_DEC 
/// Deceleration duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_1 (16.02)

/// Register : KVAL_RUN 
/// Run duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_1 (16.02)

/// Register : KVAL_HOLD 
/// Hold duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_1 (16.02)

/// Register : CONFIG - field : EN_VSCOMP 
/// Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
#define POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_1 (POWERSTEP01_CONFIG_VS_COMP_DISABLE)

/// Register : MIN_SPEED - field : LSPD_OPT 
/// Low speed optimization bit, enum powerstep01_LspdOpt_t
#define POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_1 (POWERSTEP01_LSPD_OPT_OFF)

/// Register : K_THERM 
/// Thermal compensation param, range 1 to 1.46875
#define POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_1 (1)

/// Register : INT_SPEED 
/// Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
#define POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_1 (61.512)

/// Register : ST_SLP 
/// BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_1 (0.03815)

/// Register : FN_SLP_ACC 
/// BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_1 (0.06256)

/// Register : FN_SLP_DEC 
/// BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_1 (0.06256)

/// Register : CONFIG - field : F_PWM_INT 
/// PWM Frequency Integer division, enum powerstep01_ConfigFPwmInt_t
#define POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_1 (POWERSTEP01_CONFIG_PWM_DIV_2)

/// Register : CONFIG - field : F_PWM_DEC 
/// PWM Frequency Integer Multiplier, enum powerstep01_ConfigFPwmDec_t
#define POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_1 (POWERSTEP01_CONFIG_PWM_MUL_1)

/******************** Advance current control parameters  *********************/

/// Register : TVAL_ACC 
/// Acceleration torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_1  (328.12)

/// Register : TVAL_DEC 
/// Deceleration torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_1  (328.12)

/// Register : TVAL_RUN 
/// Running torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_1  (328.12)

/// Register : TVAL_HOLD 
/// Holding torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_1 (328.12) 

/// Register : CONFIG - field : EN_TQREG 
/// External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t 
#define POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_1 (POWERSTEP01_CONFIG_TQ_REG_TVAL_USED)  

/// Register : CONFIG - field : PRED_EN 
/// Predictive current enabling , enum powerstep01_ConfigPredEn_t  
#define POWERSTEP01_CONF_PARAM_PRED_DEVICE_1 (POWERSTEP01_CONFIG_PRED_DISABLE)  

///  Register : TON_MIN 
///  Minimum on-time in us, range 0.5us to 64us 
#define POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_1 (3.0)

/// Register : TOFF_MIN 
///  Minimum off-time in us, range 0.5us to 64us 
#define POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_1 (21.0)

/// Register : T_FAST - field: TOFF_FAST 
/// Maximum fast decay time , enum powerstep01_ToffFast_t
#define POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_1  (POWERSTEP01_TOFF_FAST_8us)

/// Register : T_FAST - field: FAST_STEP 
///  Maximum fall step time , enum powerstep01_FastStep_t
#define POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_1  (POWERSTEP01_FAST_STEP_12us)

/// Register : CONFIG - field : TSW 
/// Switching period, enum powerstep01_ConfigTsw_t
#define POWERSTEP01_CONF_PARAM_TSW_DEVICE_1 (POWERSTEP01_CONFIG_TSW_048us)  

/****************************** Gate Driving **********************************/

/// Register : GATECFG1 - field : IGATE 
/// Gate sink/source current via enum powerstep01_Igate_t 
#define POWERSTEP01_CONF_PARAM_IGATE_DEVICE_1      (POWERSTEP01_IGATE_64mA)

/// Register : CONFIG - field : VCCVAL 
/// VCC Val, enum powerstep01_ConfigVccVal_t 
#define POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_1 (POWERSTEP01_CONFIG_VCCVAL_15V)

/// Register : CONFIG - field : UVLOVAL 
/// UVLO Threshold via powerstep01_ConfigUvLoVal_t 
#define POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_1    (POWERSTEP01_CONFIG_UVLOVAL_LOW)

/// Register : GATECFG1 - field : TBOOST 
/// Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t 
#define POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_1     (POWERSTEP01_TBOOST_0ns)

/// Register : GATECFG1 - field : TCC 
/// Controlled current time via enum powerstep01_Tcc_t
#define POWERSTEP01_CONF_PARAM_TCC_DEVICE_1        (POWERSTEP01_TCC_500ns)

/// Duration of the blanking time via enum powerstep01_TBlank_t 
#define POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_1     (POWERSTEP01_TBLANK_375ns)

/// Register : GATECFG2 - field : TDT 
/// Duration of the dead time via enum powerstep01_Tdt_t
#define POWERSTEP01_CONF_PARAM_TDT_DEVICE_1        (POWERSTEP01_TDT_125ns)

/******************************* Others *************************************/

/// Register : OCD_TH 
/// Overcurrent threshold settings via enum powerstep01_OcdTh_t
#define POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_1 (POWERSTEP01_OCD_TH_281_25mV)

/// Register : CONFIG - field : OC_SD 
/// Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t 
#define POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_1 (POWERSTEP01_CONFIG_OC_SD_DISABLE)

/// Register : STALL_TH 
/// Stall threshold settings in mV, range 31.25mV to 1000mV 
#define POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_1 (531.25)

/// Register : ALARM_EN 
/// Alarm settings via bitmap enum powerstep01_AlarmEn_t 
#define POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_1 (POWERSTEP01_ALARM_EN_OVERCURRENT | \
                                                  POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN |  \
                                                  POWERSTEP01_ALARM_EN_THERMAL_WARNING |  \
                                                  POWERSTEP01_ALARM_EN_UVLO |  \
                                                  POWERSTEP01_ALARM_EN_STALL_DETECTION |  \
                                                  POWERSTEP01_ALARM_EN_SW_TURN_ON | \
                                                  POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD)

/// Register : CONFIG - field : SW_MODE 
/// External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t 
#define POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_1 (POWERSTEP01_CONFIG_SW_HARD_STOP)

/// Register : STEP_MODE - field : STEP_MODE 
/// Step mode settings via enum powerstep01_StepSel_t 
#define POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_1 (STEP_MODE_1_16)

/// Register : STEP_MODE - field : CM_VM 
/// Current mode or Voltage mode via enum powerstep01_CmVm_t 
#define POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_1 (POWERSTEP01_CM_VM_CURRENT)

/// Register : STEP_MODE - Field : SYNC_MODE and SYNC_EN 
/// Synch. Mode settings via enum powerstep01_SyncSel_t 
#define POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_1 (POWERSTEP01_SYNC_SEL_DISABLED)

/// Register : CONFIG - field : OSC_CLK_SEL 
/// Clock setting , enum powerstep01_ConfigOscMgmt_t 
#define POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_1 (POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ)

/// Register : GATECFG1 - field : WD_EN 
/// External clock watchdog, enum powerstep01_WdEn_t
#define POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_1 (POWERSTEP01_WD_EN_DISABLE)

/****************************************************************************/
/* Device 2                                                                 */
/****************************************************************************/
  
/**************************** Speed Profile *********************************/
/// Register : ACC
/// Acceleration rate in step/s2, range 14.55 to 59590 steps/s^2 
#define POWERSTEP01_CONF_PARAM_ACC_DEVICE_2 (2008.16)

/// Register : DEC
/// Deceleration rate in step/s2, range 14.55 to 59590 steps/s^2
#define POWERSTEP01_CONF_PARAM_DEC_DEVICE_2 (2008.16)

///Register : MAX_SPEED
/// Maximum speed in step/s, range 15.25 to 15610 steps/s
#define POWERSTEP01_CONF_PARAM_MAX_SPEED_DEVICE_2 (991.82)

/// Register : MIN_SPEED 
/// Minimum speed in step/s, range 0 to 976.3 steps/s
#define POWERSTEP01_CONF_PARAM_MIN_SPEED_DEVICE_2 (0)

/// Register : FS_SPD 
/// Full step speed in step/s, range 7.63 to 15625 steps/s
#define POWERSTEP01_CONF_PARAM_FS_SPD_DEVICE_2 (595.09)

/// Register : FS_SPD - field : BOOST_MODE 
/// Boost of the amplitude square wave, enum powerstep01_BoostMode_t
#define POWERSTEP01_CONF_PARAM_BOOST_MODE_DEVICE_2 (POWERSTEP01_BOOST_MODE_OFF)


/************************ Voltage mode parameters  **************************/
/// Register : KVAL_ACC 
/// Acceleration duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_ACC_DEVICE_2 (16.02)

/// Register : KVAL_DEC 
/// Deceleration duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_DEC_DEVICE_2 (16.02)

/// Register : KVAL_RUN 
/// Run duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_RUN_DEVICE_2 (16.02)

/// Register : KVAL_HOLD 
/// Hold duty cycle (torque) in %, range 0 to 99.6%
#define POWERSTEP01_CONF_PARAM_KVAL_HOLD_DEVICE_2 (16.02)

/// Register : CONFIG - field : EN_VSCOMP 
/// Motor Supply Voltage Compensation enabling , enum powerstep01_ConfigEnVscomp_t
#define POWERSTEP01_CONF_PARAM_VS_COMP_DEVICE_2 (POWERSTEP01_CONFIG_VS_COMP_DISABLE)

/// Register : MIN_SPEED - field : LSPD_OPT 
/// Low speed optimization bit, enum powerstep01_LspdOpt_t
#define POWERSTEP01_CONF_PARAM_LSPD_BIT_DEVICE_2 (POWERSTEP01_LSPD_OPT_OFF)

/// Register : K_THERM 
/// Thermal compensation param, range 1 to 1.46875
#define POWERSTEP01_CONF_PARAM_K_THERM_DEVICE_2 (1)

/// Register : INT_SPEED 
/// Intersect speed settings for BEMF compensation in steps/s, range 0 to 3906 steps/s
#define POWERSTEP01_CONF_PARAM_INT_SPD_DEVICE_2 (61.512)

/// Register : ST_SLP 
/// BEMF start slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_ST_SLP_DEVICE_2 (0.03815)

/// Register : FN_SLP_ACC 
/// BEMF final acc slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_FN_SLP_ACC_DEVICE_2 (0.06256)

/// Register : FN_SLP_DEC 
/// BEMF final dec slope settings for BEMF compensation in % step/s, range 0 to 0.4% s/step
#define POWERSTEP01_CONF_PARAM_FN_SLP_DEC_DEVICE_2 (0.06256)

/// Register : CONFIG - field : F_PWM_INT 
/// PWM Frequency Integer division, enum powerstep01_ConfigFPwmInt_t
#define POWERSTEP01_CONF_PARAM_PWM_DIV_DEVICE_2 (POWERSTEP01_CONFIG_PWM_DIV_2)

/// Register : CONFIG - field : F_PWM_DEC 
/// PWM Frequency Integer Multiplier, enum powerstep01_ConfigFPwmDec_t
#define POWERSTEP01_CONF_PARAM_PWM_MUL_DEVICE_2 (POWERSTEP01_CONFIG_PWM_MUL_1)

/******************** Advance current control parameters  *********************/

/// Register : TVAL_ACC 
/// Acceleration torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_ACC_DEVICE_2  (328.12)

/// Register : TVAL_DEC 
/// Deceleration torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_DEC_DEVICE_2  (328.12)

/// Register : TVAL_RUN 
/// Running torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_RUN_DEVICE_2  (328.12)

/// Register : TVAL_HOLD 
/// Holding torque in mV, range from 7.8mV to 1000 mV 
#define POWERSTEP01_CONF_PARAM_TVAL_HOLD_DEVICE_2 (328.12) 

/// Register : CONFIG - field : EN_TQREG 
/// External torque regulation enabling , enum powerstep01_ConfigEnTqReg_t 
#define POWERSTEP01_CONF_PARAM_TQ_REG_DEVICE_2 (POWERSTEP01_CONFIG_TQ_REG_TVAL_USED)  

/// Register : CONFIG - field : PRED_EN 
/// Predictive current enabling , enum powerstep01_ConfigPredEn_t  
#define POWERSTEP01_CONF_PARAM_PRED_DEVICE_2 (POWERSTEP01_CONFIG_PRED_DISABLE)  

///  Register : TON_MIN 
///  Minimum on-time in us, range 0.5us to 64us 
#define POWERSTEP01_CONF_PARAM_TON_MIN_DEVICE_2 (3.0)

/// Register : TOFF_MIN 
///  Minimum off-time in us, range 0.5us to 64us 
#define POWERSTEP01_CONF_PARAM_TOFF_MIN_DEVICE_2 (21.0)

/// Register : T_FAST - field: TOFF_FAST 
/// Maximum fast decay time , enum powerstep01_ToffFast_t
#define POWERSTEP01_CONF_PARAM_TOFF_FAST_DEVICE_2  (POWERSTEP01_TOFF_FAST_8us)

/// Register : T_FAST - field: FAST_STEP 
///  Maximum fall step time , enum powerstep01_FastStep_t
#define POWERSTEP01_CONF_PARAM_FAST_STEP_DEVICE_2  (POWERSTEP01_FAST_STEP_12us)

/// Register : CONFIG - field : TSW 
/// Switching period, enum powerstep01_ConfigTsw_t
#define POWERSTEP01_CONF_PARAM_TSW_DEVICE_2 (POWERSTEP01_CONFIG_TSW_048us)  

/****************************** Gate Driving **********************************/

/// Register : GATECFG1 - field : IGATE 
/// Gate sink/source current via enum powerstep01_Igate_t 
#define POWERSTEP01_CONF_PARAM_IGATE_DEVICE_2      (POWERSTEP01_IGATE_64mA)

/// Register : CONFIG - field : VCCVAL 
/// VCC Val, enum powerstep01_ConfigVccVal_t 
#define POWERSTEP01_CONF_PARAM_VCCVAL_DEVICE_2 (POWERSTEP01_CONFIG_VCCVAL_15V)

/// Register : CONFIG - field : UVLOVAL 
/// UVLO Threshold via powerstep01_ConfigUvLoVal_t 
#define POWERSTEP01_CONF_PARAM_UVLOVAL_DEVICE_2    (POWERSTEP01_CONFIG_UVLOVAL_LOW)

/// Register : GATECFG1 - field : TBOOST 
/// Duration of the overboost phase during gate turn-off via enum powerstep01_Tboost_t 
#define POWERSTEP01_CONF_PARAM_TBOOST_DEVICE_2     (POWERSTEP01_TBOOST_0ns)

/// Register : GATECFG1 - field : TCC 
/// Controlled current time via enum powerstep01_Tcc_t
#define POWERSTEP01_CONF_PARAM_TCC_DEVICE_2        (POWERSTEP01_TCC_500ns)

/// Duration of the blanking time via enum powerstep01_TBlank_t 
#define POWERSTEP01_CONF_PARAM_TBLANK_DEVICE_2     (POWERSTEP01_TBLANK_375ns)

/// Register : GATECFG2 - field : TDT 
/// Duration of the dead time via enum powerstep01_Tdt_t
#define POWERSTEP01_CONF_PARAM_TDT_DEVICE_2        (POWERSTEP01_TDT_125ns)

/******************************* Others *************************************/

/// Register : OCD_TH 
/// Overcurrent threshold settings via enum powerstep01_OcdTh_t
#define POWERSTEP01_CONF_PARAM_OCD_TH_DEVICE_2 (POWERSTEP01_OCD_TH_281_25mV)

/// Register : CONFIG - field : OC_SD 
/// Over current shutwdown enabling, enum powerstep01_ConfigOcSd_t 
#define POWERSTEP01_CONF_PARAM_OC_SD_DEVICE_2 (POWERSTEP01_CONFIG_OC_SD_DISABLE)

/// Register : STALL_TH 
/// Stall threshold settings in mV, range 31.25mV to 1000mV 
#define POWERSTEP01_CONF_PARAM_STALL_TH_DEVICE_2 (531.25)

/// Register : ALARM_EN 
/// Alarm settings via bitmap enum powerstep01_AlarmEn_t 
#define POWERSTEP01_CONF_PARAM_ALARM_EN_DEVICE_2 (POWERSTEP01_ALARM_EN_OVERCURRENT | \
                                                  POWERSTEP01_ALARM_EN_THERMAL_SHUTDOWN |  \
                                                  POWERSTEP01_ALARM_EN_THERMAL_WARNING |  \
                                                  POWERSTEP01_ALARM_EN_UVLO |  \
                                                  POWERSTEP01_ALARM_EN_STALL_DETECTION |  \
                                                  POWERSTEP01_ALARM_EN_SW_TURN_ON | \
                                                  POWERSTEP01_ALARM_EN_WRONG_NPERF_CMD)

/// Register : CONFIG - field : SW_MODE 
/// External switch hard stop interrupt mode, enum powerstep01_ConfigSwMode_t 
#define POWERSTEP01_CONF_PARAM_SW_MODE_DEVICE_2 (POWERSTEP01_CONFIG_SW_HARD_STOP)

/// Register : STEP_MODE - field : STEP_MODE 
/// Step mode settings via enum powerstep01_StepSel_t 
#define POWERSTEP01_CONF_PARAM_STEP_MODE_DEVICE_2 (STEP_MODE_1_16)

/// Register : STEP_MODE - field : CM_VM 
/// Current mode or Voltage mode via enum powerstep01_CmVm_t 
#define POWERSTEP01_CONF_PARAM_CM_VM_DEVICE_2 (POWERSTEP01_CM_VM_CURRENT)

/// Register : STEP_MODE - Field : SYNC_MODE and SYNC_EN 
/// Synch. Mode settings via enum powerstep01_SyncSel_t 
#define POWERSTEP01_CONF_PARAM_SYNC_MODE_DEVICE_2 (POWERSTEP01_SYNC_SEL_DISABLED)

/// Register : CONFIG - field : OSC_CLK_SEL 
/// Clock setting , enum powerstep01_ConfigOscMgmt_t 
#define POWERSTEP01_CONF_PARAM_CLOCK_SETTING_DEVICE_2 (POWERSTEP01_CONFIG_INT_16MHZ_OSCOUT_2MHZ)

/// Register : GATECFG1 - field : WD_EN 
/// External clock watchdog, enum powerstep01_WdEn_t
#define POWERSTEP01_CONF_PARAM_WD_EN_DEVICE_2 (POWERSTEP01_WD_EN_DISABLE)

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

#endif /* __POWERSTEP01_TARGET_CONFIG_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
