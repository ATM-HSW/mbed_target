/**
 ******************************************************************************
 * @file    StepperMotor_class.h
 * @author  Davide Aliprandi, STMicroelectronics
 * @version V1.1.0
 * @date    April 6th, 2016
 * @brief   This file contains the abstract class describing the interface of a
 *          stepper-motor component.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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


/* Define to prevent from recursive inclusion --------------------------------*/

#ifndef __STEPPERMOTOR_CLASS_H
#define __STEPPERMOTOR_CLASS_H


/* Includes ------------------------------------------------------------------*/

#include <Component_class.h>


/* Classes  ------------------------------------------------------------------*/

/** An abstract class for StepperMotor components.
 */
class StepperMotor : public Component
{
public:
    /**
     * @brief Rotation modes.
     */
    typedef enum
    {
        BWD = 0, /* Backward. */
        FWD = 1  /* Forward. */
    } direction_t;

    /**
     * @brief Step modes.
     */
    typedef enum
    {
        STEP_MODE_FULL = 0, /* Full-step. */
        STEP_MODE_HALF,     /* Half-step. */
        STEP_MODE_1_4,      /* 1/4 microstep. */
        STEP_MODE_1_8,      /* 1/8 microstep. */
        STEP_MODE_1_16,     /* 1/16 microstep. */
        STEP_MODE_1_32,     /* 1/32 microstep. */
        STEP_MODE_1_64,     /* 1/64 microstep. */
        STEP_MODE_1_128,    /* 1/128 microstep. */
        STEP_MODE_1_256,    /* 1/256 microstep. */
        STEP_MODE_UNKNOWN,
        STEP_MODE_WAVE      /* Full-step one-phase-on*/
    } step_mode_t;

    /**
     * @brief  Getting the status.
     * @param  None.
     * @retval The status.
     */
    virtual unsigned int GetStatus(void) = 0;

    /**
     * @brief  Getting the position.
     * @param  None.
     * @retval The position.
     */
    virtual signed int GetPosition(void) = 0;

    /**
     * @brief  Getting the marked position.
     * @param  None.
     * @retval The marked position.
     */
    virtual signed int GetMark(void) = 0;

    /**
     * @brief  Getting the current speed in pps.
     * @param  None.
     * @retval The current speed in pps.
     */
    virtual unsigned int GetSpeed(void) = 0;

    /**
     * @brief  Getting the maximum speed in pps.
     * @param  None.
     * @retval The maximum speed in pps.
     */
    virtual unsigned int GetMaxSpeed(void) = 0;

    /**
     * @brief  Getting the minimum speed in pps.
     * @param  None.
     * @retval The minimum speed in pps.
     */
    virtual unsigned int GetMinSpeed(void) = 0;

    /**
     * @brief  Getting the acceleration in pps^2.
     * @param  None.
     * @retval The acceleration in pps^2.
     */
    virtual unsigned int GetAcceleration(void) = 0;

    /**
     * @brief  Getting the deceleration in pps^2.
     * @param  None.
     * @retval The deceleration in pps^2.
     */
    virtual unsigned int GetDeceleration(void) = 0;

    /**
     * @brief  Getting the direction of rotation.
     * @param  None.
     * @retval The direction of rotation.
     */
    virtual direction_t GetDirection(void) = 0;

    /**
     * @brief  Setting the current position to be the home position.
     * @param  None.
     * @retval None.
     */
    virtual void SetHome(void) = 0;

    /**
     * @brief  Setting the current position to be the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void SetMark(void) = 0;

    /**
     * @brief  Setting the maximum speed in pps.
     * @param  speed The maximum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool SetMaxSpeed(unsigned int speed) = 0;

    /**
     * @brief  Setting the minimum speed in pps.
     * @param  speed The minimum speed in pps.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool SetMinSpeed(unsigned int speed) = 0;

    /**
     * @brief  Setting the acceleration in pps^2.
     * @param  acceleration The acceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool SetAcceleration(unsigned int acceleration) = 0;

    /**
     * @brief  Setting the deceleration in pps^2.
     * @param  deceleration The deceleration in pps^2.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool SetDeceleration(unsigned int deceleration) = 0;

    /**
     * @brief  Setting the Step Mode.
     * @param  step_mode The Step Mode.
     * @retval "true" in case of success, "false" otherwise.
     */
    virtual bool SetStepMode(step_mode_t step_mode) = 0;

    /**
     * @brief  Going to a specified position.
     * @param  position The desired position.
     * @retval None.
     */
    virtual void GoTo(signed int position) = 0;

    /**
     * @brief  Going to the home position.
     * @param  None.
     * @retval None.
     */
    virtual void GoHome(void) = 0;

    /**
     * @brief  Going to the marked position.
     * @param  None.
     * @retval None.
     */
    virtual void GoMark(void) = 0;

    /**
     * @brief  Running the motor towards a specified direction.
     * @param  direction The direction of rotation.
     * @retval None.
     */
    virtual void Run(direction_t direction) = 0;

    /**
     * @brief  Moving the motor towards a specified direction for a certain number of steps.
     * @param  direction The direction of rotation.
     * @param  steps The desired number of steps.
     * @retval None.
     */
    virtual void Move(direction_t direction, unsigned int steps) = 0;

    /**
     * @brief  Stopping the motor through an immediate deceleration up to zero speed.
     * @param  None.
     * @retval None.
     */
    virtual void SoftStop(void) = 0;

    /**
     * @brief  Stopping the motor through an immediate infinite deceleration.
     * @param  None.
     * @retval None.
     */
    virtual void HardStop(void) = 0;

    /**
     * @brief  Disabling the power bridge after performing a deceleration to zero.
     * @param  None.
     * @retval None.
     */
    virtual void SoftHiZ(void) = 0;

    /**
     * @brief  Disabling the power bridge immediately.
     * @param  None.
     * @retval None.
     */
    virtual void HardHiZ(void) = 0;

    /**
     * @brief  Waiting while the motor is active.
     * @param  None.
     * @retval None.
     */
    virtual void WaitWhileActive(void) = 0;
};

#endif /* __STEPPERMOTOR_CLASS_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/ 
