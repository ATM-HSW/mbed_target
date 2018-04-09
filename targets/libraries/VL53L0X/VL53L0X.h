/*******************************************************************************
 Copyright © 2016, STMicroelectronics International N.V.
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of STMicroelectronics nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND
 NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS ARE DISCLAIMED.
 IN NO EVENT SHALL STMICROELECTRONICS INTERNATIONAL N.V. BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#ifndef __VL53L0X_CLASS_H
#define __VL53L0X_CLASS_H


#ifdef _MSC_VER
#   ifdef VL53L0X_API_EXPORTS
#       define VL53L0X_API  __declspec(dllexport)
#   else
#       define VL53L0X_API
#   endif
#else
#   define VL53L0X_API
#endif


/* Includes ------------------------------------------------------------------*/
#include "mbed.h"
#include "RangeSensor.h"
#include "DevI2C.h"
#include "PinNames.h"
#include "VL53L0X_def.h"
#include "VL53L0X_platform.h"
#include "Stmpe1600.h"


/**
 * The device model ID
 */
#define IDENTIFICATION_MODEL_ID                 0x000


#define STATUS_OK              0x00
#define STATUS_FAIL            0x01

#define VL53L0X_OsDelay(...) wait_ms(2) // 2 msec delay. can also use wait(float secs)/wait_us(int)

#ifdef USE_EMPTY_STRING
#define  VL53L0X_STRING_DEVICE_INFO_NAME                             ""
#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS0                         ""
#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS1                         ""
#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS2                         ""
#define  VL53L0X_STRING_DEVICE_INFO_NAME_ES1                         ""
#define  VL53L0X_STRING_DEVICE_INFO_TYPE                             ""

/* PAL ERROR strings */
#define  VL53L0X_STRING_ERROR_NONE                                   ""
#define  VL53L0X_STRING_ERROR_CALIBRATION_WARNING                    ""
#define  VL53L0X_STRING_ERROR_MIN_CLIPPED                            ""
#define  VL53L0X_STRING_ERROR_UNDEFINED                              ""
#define  VL53L0X_STRING_ERROR_INVALID_PARAMS                         ""
#define  VL53L0X_STRING_ERROR_NOT_SUPPORTED                          ""
#define  VL53L0X_STRING_ERROR_RANGE_ERROR                            ""
#define  VL53L0X_STRING_ERROR_TIME_OUT                               ""
#define  VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED                     ""
#define  VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL                       ""
#define  VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING                      ""
#define  VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED       ""
#define  VL53L0X_STRING_ERROR_CONTROL_INTERFACE                      ""
#define  VL53L0X_STRING_ERROR_INVALID_COMMAND                        ""
#define  VL53L0X_STRING_ERROR_DIVISION_BY_ZERO                       ""
#define  VL53L0X_STRING_ERROR_REF_SPAD_INIT                          ""
#define  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED                        ""

#define  VL53L0X_STRING_UNKNOW_ERROR_CODE                            ""



/* Range Status */
#define  VL53L0X_STRING_RANGESTATUS_NONE                             ""
#define  VL53L0X_STRING_RANGESTATUS_RANGEVALID                       ""
#define  VL53L0X_STRING_RANGESTATUS_SIGMA                            ""
#define  VL53L0X_STRING_RANGESTATUS_SIGNAL                           ""
#define  VL53L0X_STRING_RANGESTATUS_MINRANGE                         ""
#define  VL53L0X_STRING_RANGESTATUS_PHASE                            ""
#define  VL53L0X_STRING_RANGESTATUS_HW                               ""


/* Range Status */
#define  VL53L0X_STRING_STATE_POWERDOWN                              ""
#define  VL53L0X_STRING_STATE_WAIT_STATICINIT                        ""
#define  VL53L0X_STRING_STATE_STANDBY                                ""
#define  VL53L0X_STRING_STATE_IDLE                                   ""
#define  VL53L0X_STRING_STATE_RUNNING                                ""
#define  VL53L0X_STRING_STATE_UNKNOWN                                ""
#define  VL53L0X_STRING_STATE_ERROR                                  ""


/* Device Specific */
#define  VL53L0X_STRING_DEVICEERROR_NONE                             ""
#define  VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE       ""
#define  VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE         ""
#define  VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND                  ""
#define  VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET                     ""
#define  VL53L0X_STRING_DEVICEERROR_SNRCHECK                         ""
#define  VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK                  ""
#define  VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK              ""
#define  VL53L0X_STRING_DEVICEERROR_TCC                              ""
#define  VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY                 ""
#define  VL53L0X_STRING_DEVICEERROR_MINCLIP                          ""
#define  VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE                    ""
#define  VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW                    ""
#define  VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW                     ""
#define  VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD             ""
#define  VL53L0X_STRING_DEVICEERROR_UNKNOWN                          ""

/* Check Enable */
#define  VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE                ""
#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE          ""
#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP                  ""
#define  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD           ""

/* Sequence Step */
#define  VL53L0X_STRING_SEQUENCESTEP_TCC                             ""
#define  VL53L0X_STRING_SEQUENCESTEP_DSS                             ""
#define  VL53L0X_STRING_SEQUENCESTEP_MSRC                            ""
#define  VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE                       ""
#define  VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE                     ""
#else
#define  VL53L0X_STRING_DEVICE_INFO_NAME          "VL53L0X cut1.0"
#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS0      "VL53L0X TS0"
#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS1      "VL53L0X TS1"
#define  VL53L0X_STRING_DEVICE_INFO_NAME_TS2      "VL53L0X TS2"
#define  VL53L0X_STRING_DEVICE_INFO_NAME_ES1      "VL53L0X ES1 or later"
#define  VL53L0X_STRING_DEVICE_INFO_TYPE          "VL53L0X"

/* PAL ERROR strings */
#define  VL53L0X_STRING_ERROR_NONE \
			"No Error"
#define  VL53L0X_STRING_ERROR_CALIBRATION_WARNING \
			"Calibration Warning Error"
#define  VL53L0X_STRING_ERROR_MIN_CLIPPED \
			"Min clipped error"
#define  VL53L0X_STRING_ERROR_UNDEFINED \
			"Undefined error"
#define  VL53L0X_STRING_ERROR_INVALID_PARAMS \
			"Invalid parameters error"
#define  VL53L0X_STRING_ERROR_NOT_SUPPORTED \
			"Not supported error"
#define  VL53L0X_STRING_ERROR_RANGE_ERROR \
			"Range error"
#define  VL53L0X_STRING_ERROR_TIME_OUT \
			"Time out error"
#define  VL53L0X_STRING_ERROR_MODE_NOT_SUPPORTED \
			"Mode not supported error"
#define  VL53L0X_STRING_ERROR_BUFFER_TOO_SMALL \
			"Buffer too small"
#define  VL53L0X_STRING_ERROR_GPIO_NOT_EXISTING \
			"GPIO not existing"
#define  VL53L0X_STRING_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED \
			"GPIO funct not supported"
#define  VL53L0X_STRING_ERROR_INTERRUPT_NOT_CLEARED \
			"Interrupt not Cleared"
#define  VL53L0X_STRING_ERROR_CONTROL_INTERFACE \
			"Control Interface Error"
#define  VL53L0X_STRING_ERROR_INVALID_COMMAND \
			"Invalid Command Error"
#define  VL53L0X_STRING_ERROR_DIVISION_BY_ZERO \
			"Division by zero Error"
#define  VL53L0X_STRING_ERROR_REF_SPAD_INIT \
			"Reference Spad Init Error"
#define  VL53L0X_STRING_ERROR_NOT_IMPLEMENTED \
			"Not implemented error"

#define  VL53L0X_STRING_UNKNOW_ERROR_CODE \
			"Unknown Error Code"



/* Range Status */
#define  VL53L0X_STRING_RANGESTATUS_NONE                 "No Update"
#define  VL53L0X_STRING_RANGESTATUS_RANGEVALID           "Range Valid"
#define  VL53L0X_STRING_RANGESTATUS_SIGMA                "Sigma Fail"
#define  VL53L0X_STRING_RANGESTATUS_SIGNAL               "Signal Fail"
#define  VL53L0X_STRING_RANGESTATUS_MINRANGE             "Min Range Fail"
#define  VL53L0X_STRING_RANGESTATUS_PHASE                "Phase Fail"
#define  VL53L0X_STRING_RANGESTATUS_HW                   "Hardware Fail"


/* Range Status */
#define  VL53L0X_STRING_STATE_POWERDOWN               "POWERDOWN State"
#define  VL53L0X_STRING_STATE_WAIT_STATICINIT \
			"Wait for staticinit State"
#define  VL53L0X_STRING_STATE_STANDBY                 "STANDBY State"
#define  VL53L0X_STRING_STATE_IDLE                    "IDLE State"
#define  VL53L0X_STRING_STATE_RUNNING                 "RUNNING State"
#define  VL53L0X_STRING_STATE_UNKNOWN                 "UNKNOWN State"
#define  VL53L0X_STRING_STATE_ERROR                   "ERROR State"


/* Device Specific */
#define  VL53L0X_STRING_DEVICEERROR_NONE              "No Update"
#define  VL53L0X_STRING_DEVICEERROR_VCSELCONTINUITYTESTFAILURE \
			"VCSEL Continuity Test Failure"
#define  VL53L0X_STRING_DEVICEERROR_VCSELWATCHDOGTESTFAILURE \
			"VCSEL Watchdog Test Failure"
#define  VL53L0X_STRING_DEVICEERROR_NOVHVVALUEFOUND \
			"No VHV Value found"
#define  VL53L0X_STRING_DEVICEERROR_MSRCNOTARGET \
			"MSRC No Target Error"
#define  VL53L0X_STRING_DEVICEERROR_SNRCHECK \
			"SNR Check Exit"
#define  VL53L0X_STRING_DEVICEERROR_RANGEPHASECHECK \
			"Range Phase Check Error"
#define  VL53L0X_STRING_DEVICEERROR_SIGMATHRESHOLDCHECK \
			"Sigma Threshold Check Error"
#define  VL53L0X_STRING_DEVICEERROR_TCC \
			"TCC Error"
#define  VL53L0X_STRING_DEVICEERROR_PHASECONSISTENCY \
			"Phase Consistency Error"
#define  VL53L0X_STRING_DEVICEERROR_MINCLIP \
			"Min Clip Error"
#define  VL53L0X_STRING_DEVICEERROR_RANGECOMPLETE \
			"Range Complete"
#define  VL53L0X_STRING_DEVICEERROR_ALGOUNDERFLOW \
			"Range Algo Underflow Error"
#define  VL53L0X_STRING_DEVICEERROR_ALGOOVERFLOW \
			"Range Algo Overlow Error"
#define  VL53L0X_STRING_DEVICEERROR_RANGEIGNORETHRESHOLD \
			"Range Ignore Threshold Error"
#define  VL53L0X_STRING_DEVICEERROR_UNKNOWN \
			"Unknown error code"

/* Check Enable */
#define  VL53L0X_STRING_CHECKENABLE_SIGMA_FINAL_RANGE \
			"SIGMA FINAL RANGE"
#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE \
			"SIGNAL RATE FINAL RANGE"
#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_REF_CLIP \
			"SIGNAL REF CLIP"
#define  VL53L0X_STRING_CHECKENABLE_RANGE_IGNORE_THRESHOLD \
			"RANGE IGNORE THRESHOLD"
#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_MSRC \
			"SIGNAL RATE MSRC"
#define  VL53L0X_STRING_CHECKENABLE_SIGNAL_RATE_PRE_RANGE \
			"SIGNAL RATE PRE RANGE"

/* Sequence Step */
#define  VL53L0X_STRING_SEQUENCESTEP_TCC                   "TCC"
#define  VL53L0X_STRING_SEQUENCESTEP_DSS                   "DSS"
#define  VL53L0X_STRING_SEQUENCESTEP_MSRC                  "MSRC"
#define  VL53L0X_STRING_SEQUENCESTEP_PRE_RANGE             "PRE RANGE"
#define  VL53L0X_STRING_SEQUENCESTEP_FINAL_RANGE           "FINAL RANGE"
#endif /* USE_EMPTY_STRING */

/* sensor operating modes */
typedef enum {
    range_single_shot_polling = 1,
    range_continuous_polling,
    range_continuous_interrupt,
    range_continuous_polling_low_threshold,
    range_continuous_polling_high_threshold,
    range_continuous_polling_out_of_window,
    range_continuous_interrupt_low_threshold,
    range_continuous_interrupt_high_threshold,
    range_continuous_interrupt_out_of_window,
} OperatingMode;

/** default device address */
#define VL53L0X_DEFAULT_ADDRESS		0x52 /* (8-bit) */

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL53L0 sensor component
 */
class VL53L0X : public RangeSensor
{
public:
    /** Constructor
     * @param[in] &i2c device I2C to be used for communication
     * @param[in] &pin_gpio1 pin Mbed InterruptIn PinName to be used as component GPIO_1 INT
     * @param[in] dev_addr device address, 0x29 by default
     */
    VL53L0X(DevI2C *i2c, DigitalOut *pin, PinName pin_gpio1, uint8_t dev_addr = VL53L0X_DEFAULT_ADDRESS) : _dev_i2c(i2c),
        _gpio0(pin)
    {
        _my_device.I2cDevAddr = dev_addr;
        _my_device.comms_type = 1; // VL53L0X_COMMS_I2C
        _my_device.comms_speed_khz = 400;
        _device = &_my_device;
        _expgpio0 = NULL;
        if (pin_gpio1 != NC) {
            _gpio1Int = new InterruptIn(pin_gpio1);
        } else {
            _gpio1Int = NULL;
        }
    }

    /** Constructor 2 (STMPE1600DigiOut)
     * @param[in] i2c device I2C to be used for communication
     * @param[in] &pin Gpio Expander STMPE1600DigiOut pin to be used as component GPIO_0 CE
     * @param[in] pin_gpio1 pin Mbed InterruptIn PinName to be used as component GPIO_1 INT
     * @param[in] device address, 0x29 by default
     */
    VL53L0X(DevI2C *i2c, Stmpe1600DigiOut *pin, PinName pin_gpio1,
            uint8_t dev_addr = VL53L0X_DEFAULT_ADDRESS) : _dev_i2c(i2c), _expgpio0(pin)
    {
        _my_device.I2cDevAddr = dev_addr;
        _my_device.comms_type = 1; // VL53L0X_COMMS_I2C
        _my_device.comms_speed_khz = 400;
        _device = &_my_device;
        _gpio0 = NULL;
        if (pin_gpio1 != NC) {
            _gpio1Int = new InterruptIn(pin_gpio1);
        } else {
            _gpio1Int = NULL;
        }
    }

    /** Destructor
     */
    virtual ~VL53L0X()
    {
        if (_gpio1Int != NULL) {
            delete _gpio1Int;
        }
    }

    /*** Interface Methods ***/
    /*** High level API ***/
    /**
     * @brief       PowerOn the sensor
     * @return      void
     */
    /* turns on the sensor */
    void VL53L0X_on(void)
    {
        if (_gpio0) {
            *_gpio0 = 1;
        } else {
            if (_expgpio0) {
                *_expgpio0 = 1;
            }
        }
        wait_ms(10);
    }

    /**
     * @brief       PowerOff the sensor
     * @return      void
     */
    /* turns off the sensor */
    void VL53L0X_off(void)
    {
        if (_gpio0) {
            *_gpio0 = 0;
        } else {
            if (_expgpio0) {
                *_expgpio0 = 0;
            }
        }
        wait_ms(10);
    }


    /**
     * @brief       Initialize the sensor with default values
     * @return      "0" on success
     */
    int init_sensor(uint8_t new_addr);

    /**
     * @brief       Start the measure indicated by operating mode
     * @param[in]   operating_mode specifies requested measure
     * @param[in]   fptr specifies call back function must be !NULL in case of interrupt measure
     * @return      "0" on success
     */
    int start_measurement(OperatingMode operating_mode, void (*fptr)(void));

    /**
     * @brief       Get results for the measure indicated by operating mode
     * @param[in]   operating_mode specifies requested measure results
     * @param[out]  p_data pointer to the MeasureData_t structure to read data in to
     * @return      "0" on success
     */
    int get_measurement(OperatingMode operating_mode, VL53L0X_RangingMeasurementData_t *p_data);

    /**
     * @brief       Stop the currently running measure indicate by operating_mode
     * @param[in]   operating_mode specifies requested measure to stop
     * @return      "0" on success
     */
    int stop_measurement(OperatingMode operating_mode);

    /**
     * @brief       Interrupt handling func to be called by user after an INT is occourred
     * @param[in]   opeating_mode indicating the in progress measure
     * @param[out]  Data pointer to the MeasureData_t structure to read data in to
     * @return      "0" on success
     */
    int handle_irq(OperatingMode operating_mode, VL53L0X_RangingMeasurementData_t *data);

    /**
     * @brief       Enable interrupt measure IRQ
     * @return      "0" on success
     */
    void enable_interrupt_measure_detection_irq(void)
    {
        if (_gpio1Int != NULL) {
            _gpio1Int->enable_irq();
        }
    }

    /**
     * @brief       Disable interrupt measure IRQ
     * @return      "0" on success
     */
    void disable_interrupt_measure_detection_irq(void)
    {
        if (_gpio1Int != NULL) {
            _gpio1Int->disable_irq();
        }
    }

    /**
     * @brief       Attach a function to call when an interrupt is detected, i.e. measurement is ready
     * @param[in]   fptr pointer to call back function to be called whenever an interrupt occours
     * @return      "0" on success
     */
    void attach_interrupt_measure_detection_irq(void (*fptr)(void))
    {
        if (_gpio1Int != NULL) {
            _gpio1Int->rise(fptr);
        }
    }

    /** Wrapper functions */
    /** @defgroup api_init Init functions
     *  @brief    API init functions
     *  @ingroup  api_hl
     *  @{
     */

    /**
     *
     * @brief One time device initialization
     *
     * To be called once and only once after device is brought out of reset (Chip enable) and booted.
     *
     * @par Function Description
     * When not used after a fresh device "power up" or reset, it may return @a #CALIBRATION_WARNING
     * meaning wrong calibration data may have been fetched from device that can result in ranging offset error\n
     * If application cannot execute device reset or need to run VL53L0X_data_init  multiple time
     * then it  must ensure proper offset calibration saving and restore on its own
     * by using @a VL53L0X_get_offset_calibration_data_micro_meter() on first power up and then @a VL53L0X_set_offset_calibration_data_micro_meter() all all subsequent init
     *
     * @param void
     * @return     "0" on success,  @a #CALIBRATION_WARNING if failed
     */
    virtual int init(void *init)
    {
        return VL53L0X_data_init(_device);
    }

    /**
      * @brief  Prepare device for operation
      * @par Function Description
      * Does static initialization and reprogram common default settings \n
      * Device is prepared for new measure, ready single shot ranging or ALS typical polling operation\n
      * After prepare user can : \n
      * @li Call other API function to set other settings\n
      * @li Configure the interrupt pins, etc... \n
      * @li Then start ranging or ALS operations in single shot or continuous mode
      *
      * @param void
      * @return  "0" on success
      */
    int prepare()
    {
        VL53L0X_Error status = VL53L0X_ERROR_NONE;
        uint32_t ref_spad_count;
        uint8_t is_aperture_spads;
        uint8_t vhv_settings;
        uint8_t phase_cal;

        if (status == VL53L0X_ERROR_NONE) {
            //printf("Call of VL53L0X_StaticInit\r\n");
            status = VL53L0X_static_init(_device);   // Device Initialization
        }

        if (status == VL53L0X_ERROR_NONE) {
            //printf("Call of VL53L0X_PerformRefCalibration\r\n");
            status = VL53L0X_perform_ref_calibration(_device,
                     &vhv_settings, &phase_cal);  // Device Initialization
        }

        if (status == VL53L0X_ERROR_NONE) {
            //printf("Call of VL53L0X_PerformRefSpadManagement\r\n");
            status = VL53L0X_perform_ref_spad_management(_device,
                     &ref_spad_count, &is_aperture_spads);  // Device Initialization
//            printf ("refSpadCount = %d, isApertureSpads = %d\r\n", refSpadCount, isApertureSpads);
        }

        return status;
    }

    /**
    * @brief Start continuous ranging mode
    *
    * @details End user should ensure device is in idle state and not already running
    * @return  "0" on success
    */
    int range_start_continuous_mode()
    {
        int status;
        status = VL53L0X_set_device_mode(_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

        if (status == VL53L0X_ERROR_NONE) {
            //printf ("Call of VL53L0X_StartMeasurement\r\n");
            status = VL53L0X_start_measurement(_device);
        }

        return status;
    }

    /**
     * @brief Get ranging result and only that
     *
     * @par Function Description
     * Unlike @a VL53L0X_get_ranging_measurement_data() this function only retrieves the range in millimeter \n
     * It does any required up-scale translation\n
     * It can be called after success status polling or in interrupt mode \n
     * @warning these function is not doing wrap around filtering \n
     * This function doesn't perform any data ready check!
     *
     * @param p_data  Pointer to range distance
     * @return        "0" on success
     */
    virtual int get_distance(uint32_t *p_data)
    {
        int status = 0;
        VL53L0X_RangingMeasurementData_t p_ranging_measurement_data;

        status = start_measurement(range_single_shot_polling, NULL);
        if (!status) {
            status = get_measurement(range_single_shot_polling, &p_ranging_measurement_data);
        }
        if (p_ranging_measurement_data.RangeStatus == 0) {
            // we have a valid range.
            *p_data = p_ranging_measurement_data.RangeMilliMeter;
        } else {
            *p_data = 0;
            status = VL53L0X_ERROR_RANGE_ERROR;
        }
        stop_measurement(range_single_shot_polling);
        return status;
    }

    /** @}  */

    /**
     * @brief Set new device i2c address
     *
     * After completion the device will answer to the new address programmed.
     *
     * @param new_addr  The new i2c address (7bit)
     * @return          "0" on success
     */
    int set_device_address(int new_addr)
    {
        int status;

        status = VL53L0X_set_device_address(_device, new_addr);
        if (!status) {
            _device->I2cDevAddr = new_addr;
        }
        return status;

    }

    /**
     * @brief Clear given system interrupt condition
     *
     * @par Function Description
     * Clear given interrupt cause by writing into register #SYSTEM_INTERRUPT_CLEAR register.
     * @param dev        The device
     * @param int_clear  Which interrupt source to clear. Use any combinations of #INTERRUPT_CLEAR_RANGING , #INTERRUPT_CLEAR_ALS , #INTERRUPT_CLEAR_ERROR.
     * @return           "0" on success
     */
    int clear_interrupt(uint8_t int_clear)
    {
        return VL53L0X_clear_interrupt_mask(_device, int_clear);
    }

    /**
     *
     * @brief Get the 53L0 device
     *
     * To be called to retrive the internal device descriptor to allow usage of 
     * low level API having device as parameter. To be called  after set_device_address()
     * (if any).
     *
     * @par Function Description
     * To be called if low level API usage is needed as those functions requires
     * device as a parameter.TICINIT.
     *
     * @note This function return a pointer to an object internal structure
     *
     * @param   dev                   ptr to ptr to Device Handle
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error vl53l0x_get_device(VL53L0X_DEV *dev)
{
   *dev = _device;
   return VL53L0X_ERROR_NONE;
}             

    /**
     *
     * @brief One time device initialization
     *
     * To be called once and only once after device is brought out of reset
     * (Chip enable) and booted see @a VL53L0X_WaitDeviceBooted()
     *
     * @par Function Description
     * When not used after a fresh device "power up" or reset, it may return
     * @a #VL53L0X_ERROR_CALIBRATION_WARNING meaning wrong calibration data
     * may have been fetched from device that can result in ranging offset error\n
     * If application cannot execute device reset or need to run VL53L0X_DataInit
     * multiple time then it  must ensure proper offset calibration saving and
     * restore on its own by using @a VL53L0X_GetOffsetCalibrationData() on first
     * power up and then @a VL53L0X_SetOffsetCalibrationData() in all subsequent init
     * This function will change the VL53L0X_State from VL53L0X_STATE_POWERDOWN to
     * VL53L0X_STATE_WAIT_STATICINIT.
     *
     * @note This function accesses to the device
     *
     * @param   dev                   Device Handle
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_data_init(VL53L0X_DEV dev);

    /**
     * @brief Do basic device init (and eventually patch loading)
     * This function will change the VL53L0X_State from
     * VL53L0X_STATE_WAIT_STATICINIT to VL53L0X_STATE_IDLE.
     * In this stage all default setting will be applied.
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_static_init(VL53L0X_DEV dev);

    /**
     * @brief Perform Reference Calibration
     *
     * @details Perform a reference calibration of the Device.
     * This function should be run from time to time before doing
     * a ranging measurement.
     * This function will launch a special ranging measurement, so
     * if interrupt are enable an interrupt will be done.
     * This function will clear the interrupt generated automatically.
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   dev                  Device Handle
     * @param   p_vhv_settings       Pointer to vhv settings parameter.
     * @param   p_phase_cal          Pointer to PhaseCal parameter.
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV dev, uint8_t *p_vhv_settings,
            uint8_t *p_phase_cal);

    /**
     * @brief Get Reference Calibration Parameters
     *
     * @par Function Description
     * Get Reference Calibration Parameters.
     *
     * @note This function Access to the device
     *
     * @param   dev                            Device Handle
     * @param   p_vhv_settings                 Pointer to VHV parameter
     * @param   p_phase_cal                    Pointer to PhaseCal Parameter
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"             See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_ref_calibration(VL53L0X_DEV dev,
            uint8_t *p_vhv_settings, uint8_t *p_phase_cal);

    VL53L0X_Error VL53L0X_set_ref_calibration(VL53L0X_DEV dev,
            uint8_t vhv_settings, uint8_t phase_cal);

    /**
     * @brief Performs Reference Spad Management
     *
     * @par Function Description
     * The reference SPAD initialization procedure determines the minimum amount
     * of reference spads to be enables to achieve a target reference signal rate
     * and should be performed once during initialization.
     *
     * @note This function Access to the device
     *
     * @note This function change the device mode to
     * VL53L0X_DEVICEMODE_SINGLE_RANGING
     *
     * @param   dev                          Device Handle
     * @param   ref_spad_count               Reports ref Spad Count
     * @param   is_aperture_spads            Reports if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE           Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT  Error in the Ref Spad procedure.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_perform_ref_spad_management(VL53L0X_DEV dev,
            uint32_t *ref_spad_count, uint8_t *is_aperture_spads);

    /**
     * @brief Applies Reference SPAD configuration
     *
     * @par Function Description
     * This function applies a given number of reference spads, identified as
     * either Aperture or Non-Aperture.
     * The requested spad count and type are stored within the device specific
     * parameters data for access by the host.
     *
     * @note This function Access to the device
     *
     * @param   dev                          Device Handle
     * @param   refSpadCount                 Number of ref spads.
     * @param   is_aperture_spads            Defines if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE           Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT  Error in the in the reference
     *                                       spad configuration.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_reference_spads(VL53L0X_DEV dev,
            uint32_t refSpadCount, uint8_t is_aperture_spads);

    /**
     * @brief Retrieves SPAD configuration
     *
     * @par Function Description
     * This function retrieves the current number of applied reference spads
     * and also their type : Aperture or Non-Aperture.
     *
     * @note This function Access to the device
     *
     * @param   dev                          Device Handle
     * @param   p_spad_count                 Number ref Spad Count
     * @param   p_is_aperture_spads          Reports if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE           Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT  Error in the in the reference
     *                                       spad configuration.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_reference_spads(VL53L0X_DEV dev,
            uint32_t *p_spad_count, uint8_t *p_is_aperture_spads);

    /**
     * @brief Get part to part calibration offset
     *
     * @par Function Description
     * Should only be used after a successful call to @a VL53L0X_DataInit to backup
     * device NVM value
     *
     * @note This function Access to the device
     *
     * @param   dev                                     Device Handle
     * @param   p_offset_calibration_data_micro_meter   Return part to part
     * calibration offset from device (microns)
     * @return  VL53L0X_ERROR_NONE                      Success
     * @return  "Other error code"                      See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
            int32_t *p_offset_calibration_data_micro_meter);
    /**
     * Set or over-hide part to part calibration offset
     * \sa VL53L0X_DataInit()   VL53L0X_GetOffsetCalibrationDataMicroMeter()
     *
     * @note This function Access to the device
     *
     * @param   dev                                      Device Handle
     * @param   p_offset_calibration_data_micro_meter    Offset (microns)
     * @return  VL53L0X_ERROR_NONE                       Success
     * @return  "Other error code"                       See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
            int32_t offset_calibration_data_micro_meter);

    VL53L0X_Error VL53L0X_perform_offset_calibration(VL53L0X_DEV dev,
            FixPoint1616_t cal_distance_milli_meter,
            int32_t *p_offset_micro_meter);

    VL53L0X_Error VL53L0X_perform_xtalk_calibration(VL53L0X_DEV dev,
            FixPoint1616_t xtalk_cal_distance,
            FixPoint1616_t *p_xtalk_compensation_rate_mega_cps);

    /**
     * @brief Perform XTalk Measurement
     *
     * @details Measures the current cross talk from glass in front
     * of the sensor.
     * This functions performs a histogram measurement and uses the results
     * to measure the crosstalk. For the function to be successful, there
     * must be no target in front of the sensor.
     *
     * @warning This function is a blocking function
     *
     * @warning This function is not supported when the final range
     * vcsel clock period is set below 10 PCLKS.
     *
     * @note This function Access to the device
     *
     * @param   dev                    Device Handle
     * @param   timeout_ms             Histogram measurement duration.
     * @param   p_xtalk_per_spad       Output parameter containing the crosstalk
     *                                 measurement result, in MCPS/Spad.
     *                                 Format fixpoint 16:16.
     * @param   p_ambient_too_high     Output parameter which indicate that
     *                                 pXtalkPerSpad is not good if the Ambient
     *                                 is too high.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS vcsel clock period not supported
     *                                 for this operation.
     *                                 Must not be less than 10PCLKS.
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_perform_xtalk_measurement(VL53L0X_DEV dev,
            uint32_t timeout_ms, FixPoint1616_t *p_xtalk_per_spad,
            uint8_t *p_ambient_too_high);

    /**
     * @brief Enable/Disable Cross talk compensation feature
     *
     * @note This function is not Implemented.
     * Enable/Disable Cross Talk by set to zero the Cross Talk value
     * by using @a VL53L0X_SetXTalkCompensationRateMegaCps().
     *
     * @param  dev                           Device Handle
     * @param  x_talk_compensation_enable    Cross talk compensation
     *                                       to be set 0=disabled else = enabled
     * @return VL53L0X_ERROR_NOT_IMPLEMENTED Not implemented
     */
    VL53L0X_Error VL53L0X_set_x_talk_compensation_enable(VL53L0X_DEV dev,
            uint8_t x_talk_compensation_enable);

    /**
     * @brief Get Cross talk compensation rate
     *
     * @note This function is not Implemented.
     * Enable/Disable Cross Talk by set to zero the Cross Talk value by
     * using @a VL53L0X_SetXTalkCompensationRateMegaCps().
     *
     * @param   dev                           Device Handle
     * @param   p_x_talk_compensation_enable  Pointer to the Cross talk compensation
     *                                        state 0=disabled or 1 = enabled
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED Not implemented
     */
    VL53L0X_Error VL53L0X_get_x_talk_compensation_enable(VL53L0X_DEV dev,
            uint8_t *p_x_talk_compensation_enable);
    /**
     * @brief Set Cross talk compensation rate
     *
     * @par Function Description
     * Set Cross talk compensation rate.
     *
     * @note This function Access to the device
     *
     * @param   dev                                 Device Handle
     * @param   x_talk_compensation_rate_mega_cps   Compensation rate in
     *                                              Mega counts per second
     *                                              (16.16 fix point) see
     *                                              datasheet for details
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_x_talk_compensation_rate_mega_cps(VL53L0X_DEV dev,
            FixPoint1616_t x_talk_compensation_rate_mega_cps);

    /**
     * @brief Get Cross talk compensation rate
     *
     * @par Function Description
     * Get Cross talk compensation rate.
     *
     * @note This function Access to the device
     *
     * @param   dev                                 Device Handle
     * @param   p_xtalk_compensation_rate_mega_cps  Pointer to Compensation rate
     *                                              in Mega counts per second
     *                                              (16.16 fix point) see
     *                                              datasheet for details
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_x_talk_compensation_rate_mega_cps(VL53L0X_DEV dev,
            FixPoint1616_t *p_xtalk_compensation_rate_mega_cps);

    /**
     * @brief  Set a new device mode
     * @par Function Description
     * Set device to a new mode (ranging, histogram ...)
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                   Device Handle
     * @param   device_mode           New device mode to apply
     *                                Valid values are:
     *                                VL53L0X_DEVICEMODE_SINGLE_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     *
     *
     * @return  VL53L0X_ERROR_NONE               Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED This error occurs when
     *                                           DeviceMode is not in the
     *                                           supported list
     */
    VL53L0X_Error VL53L0X_set_device_mode(VL53L0X_DEV dev, VL53L0X_DeviceModes device_mode);

    /**
     * @brief  Get current new device mode
     * @par Function Description
     * Get actual mode of the device(ranging, histogram ...)
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                   Device Handle
     * @param   p_device_mode         Pointer to current apply mode value
     *                                Valid values are:
     *                                VL53L0X_DEVICEMODE_SINGLE_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_RANGING
     *                                VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     *
     * @return  VL53L0X_ERROR_NONE                   Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED     This error occurs when
     *                                               DeviceMode is not in the
     *                                               supported list
     */
    VL53L0X_Error VL53L0X_get_device_mode(VL53L0X_DEV dev,
                                          VL53L0X_DeviceModes *p_device_mode);

    /**
    * @brief Get current configuration for GPIO pin for a given device
    *
    * @note This function Access to the device
    *
    * @param   dev                   Device Handle
    * @param   pin                   ID of the GPIO Pin
    * @param   p_device_mode         Pointer to Device Mode associated to the Gpio.
    * @param   p_functionality       Pointer to Pin functionality.
    *                                Refer to ::VL53L0X_GpioFunctionality
    * @param   p_polarity            Pointer to interrupt polarity.
    *                                Active high or active low see
    *                                ::VL53L0X_InterruptPolarity
    * @return  VL53L0X_ERROR_NONE    Success
    * @return  VL53L0X_ERROR_GPIO_NOT_EXISTING           Only Pin=0 is accepted.
    * @return  VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED
    *          This error occurs
    *          when Funcionality programmed is not in the supported list:
    *                      Supported value are:
    *                      VL53L0X_GPIOFUNCTIONALITY_OFF,
    *                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
    *                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
    *                      VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
    *                      VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY
    * @return  "Other error code"    See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_gpio_config(VL53L0X_DEV dev, uint8_t pin,
                                          VL53L0X_DeviceModes *p_device_mode,
                                          VL53L0X_GpioFunctionality *p_functionality,
                                          VL53L0X_InterruptPolarity *p_polarity);

    /**
     * @brief Set the configuration of GPIO pin for a given device
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @param   pin                   ID of the GPIO Pin
     * @param   functionality         Select Pin functionality.
     *  Refer to ::VL53L0X_GpioFunctionality
     * @param   device_mode            Device Mode associated to the Gpio.
     * @param   polarity              Set interrupt polarity. Active high
     *   or active low see ::VL53L0X_InterruptPolarity
     * @return  VL53L0X_ERROR_NONE                            Success
     * @return  VL53L0X_ERROR_GPIO_NOT_EXISTING               Only Pin=0 is accepted.
     * @return  VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED    This error occurs
     * when Functionality programmed is not in the supported list:
     *                             Supported value are:
     *                             VL53L0X_GPIOFUNCTIONALITY_OFF,
     *                             VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW,
     *                             VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH,
     VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT,
     *                               VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_gpio_config(VL53L0X_DEV dev, uint8_t pin,
                                          VL53L0X_DeviceModes device_mode, VL53L0X_GpioFunctionality functionality,
                                          VL53L0X_InterruptPolarity polarity);

    /**
     * @brief Start device measurement
     *
     * @details Started measurement will depend on device parameters set through
     * @a VL53L0X_SetParameters()
     * This is a non-blocking function.
     * This function will change the VL53L0X_State from VL53L0X_STATE_IDLE to
     * VL53L0X_STATE_RUNNING.
     *
     * @note This function Access to the device
     *

     * @param   dev                  Device Handle
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED    This error occurs when
     * DeviceMode programmed with @a VL53L0X_SetDeviceMode is not in the supported
     * list:
     *                                   Supported mode are:
     *                                   VL53L0X_DEVICEMODE_SINGLE_RANGING,
     *                                   VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
     *                                   VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING
     * @return  VL53L0X_ERROR_TIME_OUT    Time out on start measurement
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_start_measurement(VL53L0X_DEV dev);

    /**
     * @brief Stop device measurement
     *
     * @details Will set the device in standby mode at end of current measurement\n
     *          Not necessary in single mode as device shall return automatically
     *          in standby mode at end of measurement.
     *          This function will change the VL53L0X_State from VL53L0X_STATE_RUNNING
     *          to VL53L0X_STATE_IDLE.
     *
     * @note This function Access to the device
     *
     * @param   dev                  Device Handle
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_stop_measurement(VL53L0X_DEV dev);

    /**
     * @brief Return device stop completion status
     *
     * @par Function Description
     * Returns stop completiob status.
     * User shall call this function after a stop command
     *
     * @note This function Access to the device
     *
     * @param   dev                    Device Handle
     * @param   p_stop_status            Pointer to status variable to update
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_stop_completed_status(VL53L0X_DEV dev,
            uint32_t *p_stop_status);

    /**
     * @brief Return Measurement Data Ready
     *
     * @par Function Description
     * This function indicate that a measurement data is ready.
     * This function check if interrupt mode is used then check is done accordingly.
     * If perform function clear the interrupt, this function will not work,
     * like in case of @a VL53L0X_PerformSingleRangingMeasurement().
     * The previous function is blocking function, VL53L0X_GetMeasurementDataReady
     * is used for non-blocking capture.
     *
     * @note This function Access to the device
     *
     * @param   dev                    Device Handle
     * @param   p_measurement_data_ready  Pointer to Measurement Data Ready.
     *  0=data not ready, 1 = data ready
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_measurement_data_ready(VL53L0X_DEV dev,
            uint8_t *p_measurement_data_ready);

    /**
     * @brief Retrieve the measurements from device for a given setup
     *
     * @par Function Description
     * Get data from last successful Ranging measurement
     * @warning USER should take care about  @a VL53L0X_GetNumberOfROIZones()
     * before get data.
     * PAL will fill a NumberOfROIZones times the corresponding data
     * structure used in the measurement function.
     *
     * @note This function Access to the device
     *
     * @param   dev                      Device Handle
     * @param   p_ranging_measurement_data  Pointer to the data structure to fill up.
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_ranging_measurement_data(VL53L0X_DEV dev,
            VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data);

    /**
     * @brief Clear given system interrupt condition
     *
     * @par Function Description
     * Clear given interrupt(s).
     *
     * @note This function Access to the device
     *
     * @param   dev                  Device Handle
     * @param   interrupt_mask        Mask of interrupts to clear
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  VL53L0X_ERROR_INTERRUPT_NOT_CLEARED    Cannot clear interrupts
     *
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_clear_interrupt_mask(VL53L0X_DEV dev, uint32_t interrupt_mask);

    /**
     * @brief Return device interrupt status
     *
     * @par Function Description
     * Returns currently raised interrupts by the device.
     * User shall be able to activate/deactivate interrupts through
     * @a VL53L0X_SetGpioConfig()
     *
     * @note This function Access to the device
     *
     * @param   dev                    Device Handle
     * @param   p_interrupt_mask_status   Pointer to status variable to update
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_interrupt_mask_status(VL53L0X_DEV dev,
            uint32_t *p_interrupt_mask_status);

    /**
     * @brief Performs a single ranging measurement and retrieve the ranging
     * measurement data
     *
     * @par Function Description
     * This function will change the device mode to VL53L0X_DEVICEMODE_SINGLE_RANGING
     * with @a VL53L0X_SetDeviceMode(),
     * It performs measurement with @a VL53L0X_PerformSingleMeasurement()
     * It get data from last successful Ranging measurement with
     * @a VL53L0X_GetRangingMeasurementData.
     * Finally it clear the interrupt with @a VL53L0X_ClearInterruptMask().
     *
     * @note This function Access to the device
     *
     * @note This function change the device mode to
     * VL53L0X_DEVICEMODE_SINGLE_RANGING
     *
     * @param   dev                       Device Handle
     * @param   p_ranging_measurement_data   Pointer to the data structure to fill up.
     * @return  VL53L0X_ERROR_NONE         Success
     * @return  "Other error code"        See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_perform_single_ranging_measurement(VL53L0X_DEV dev,
            VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data);

    /**
     * @brief Single shot measurement.
     *
     * @par Function Description
     * Perform simple measurement sequence (Start measure, Wait measure to end,
     * and returns when measurement is done).
     * Once function returns, user can get valid data by calling
     * VL53L0X_GetRangingMeasurement or VL53L0X_GetHistogramMeasurement
     * depending on defined measurement mode
     * User should Clear the interrupt in case this are enabled by using the
     * function VL53L0X_ClearInterruptMask().
     *
     * @warning This function is a blocking function
     *
     * @note This function Access to the device
     *
     * @param   dev                  Device Handle
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_perform_single_measurement(VL53L0X_DEV dev);

    /**
    * @brief Read current status of the error register for the selected device
    *
    * @note This function Access to the device
    *
    * @param   dev                   Device Handle
    * @param   p_device_error_status    Pointer to current error code of the device
    * @return  VL53L0X_ERROR_NONE     Success
    * @return  "Other error code"    See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_device_error_status(VL53L0X_DEV dev,
            VL53L0X_DeviceError *p_device_error_status);

    /**
    * @brief Human readable error string for a given Error Code
    *
    * @note This function doesn't access to the device
    *
    * @param   error_code           The error code as stored on ::VL53L0X_DeviceError
    * @param   p_device_error_string  The error string corresponding to the ErrorCode
    * @return  VL53L0X_ERROR_NONE   Success
    * @return  "Other error code"  See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_device_error_string(
        VL53L0X_DeviceError error_code, char *p_device_error_string);

    /**
     * @brief Human readable Range Status string for a given RangeStatus
     *
     * @note This function doesn't access to the device
     *
     * @param   range_status         The RangeStatus code as stored on
     * @a VL53L0X_RangingMeasurementData_t
     * @param   p_range_status_string  The returned RangeStatus string.
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_range_status_string(uint8_t range_status,
            char *p_range_status_string);

    VL53L0X_Error VL53L0X_get_total_signal_rate(VL53L0X_DEV dev,
            VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
            FixPoint1616_t *p_total_signal_rate_mcps);

    VL53L0X_Error VL53L0X_get_total_xtalk_rate(VL53L0X_DEV dev,
            VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
            FixPoint1616_t *p_total_xtalk_rate_mcps);

    /**
     * @brief Get Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Returns the programmed the maximum time allowed by the user to the
     * device to run a full ranging sequence for the current mode
     * (ranging, histogram, ASL ...)
     *
     * @note This function Access to the device
     *
     * @param   dev                                    Device Handle
     * @param   p_measurement_timing_budget_micro_seconds   Max measurement time in
     * microseconds.
     *                                   Valid values are:
     *                                   >= 17000 microsecs when wraparound enabled
     *                                   >= 12000 microsecs when wraparound disabled
     * @return  VL53L0X_ERROR_NONE                      Success
     * @return  "Other error code"                     See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
            uint32_t *p_measurement_timing_budget_micro_seconds);

    /**
     * @brief Set Ranging Timing Budget in microseconds
     *
     * @par Function Description
     * Defines the maximum time allowed by the user to the device to run a
     * full ranging sequence for the current mode (ranging, histogram, ASL ...)
     *
     * @note This function Access to the device
     *
     * @param   dev                                Device Handle
     * @param measurement_timing_budget_micro_seconds  Max measurement time in
     * microseconds.
     *                                   Valid values are:
     *                                   >= 17000 microsecs when wraparound enabled
     *                                   >= 12000 microsecs when wraparound disabled
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned if
     MeasurementTimingBudgetMicroSeconds out of range
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
            uint32_t measurement_timing_budget_micro_seconds);

    /**
     * @brief  Get specific limit check enable state
     *
     * @par Function Description
     * This function get the enable state of a specific limit check.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   p_limit_check_enable             Pointer to the check limit enable
     * value.
     *  if 1 the check limit
     *        corresponding to LimitCheckId is Enabled
     *  if 0 the check limit
     *        corresponding to LimitCheckId is disabled
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned
     *  when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_limit_check_enable(VL53L0X_DEV dev, uint16_t limit_check_id,
            uint8_t *p_limit_check_enable);

    /**
     * @brief  Enable/Disable a specific limit check
     *
     * @par Function Description
     * This function Enable/Disable a specific limit check.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   limit_check_enable              if 1 the check limit
     *  corresponding to LimitCheckId is Enabled
     *                                        if 0 the check limit
     *  corresponding to LimitCheckId is disabled
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned
     *  when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_limit_check_enable(VL53L0X_DEV dev, uint16_t limit_check_id,
            uint8_t limit_check_enable);

    /**
     * @brief  Get a specific limit check value
     *
     * @par Function Description
     * This function get a specific limit check value from device then it updates
     * internal values and check enables.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   p_limit_check_value              Pointer to Limit
     *  check Value for a given LimitCheckId.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned
     *  when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_limit_check_value(VL53L0X_DEV dev, uint16_t limit_check_id,
            FixPoint1616_t *p_limit_check_value);

    /**
     * @brief  Set a specific limit check value
     *
     * @par Function Description
     * This function set a specific limit check value.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   limit_check_value               Limit check Value for a given
     * LimitCheckId
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned when either
     *  LimitCheckId or LimitCheckValue value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_limit_check_value(VL53L0X_DEV dev, uint16_t limit_check_id,
            FixPoint1616_t limit_check_value);

    /**
     * @brief  Get the current value of the signal used for the limit check
     *
     * @par Function Description
     * This function get a the current value of the signal used for the limit check.
     * To obtain the latest value you should run a ranging before.
     * The value reported is linked to the limit check identified with the
     * LimitCheckId.
     *
     * @note This function Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     *  (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   p_limit_check_current            Pointer to current Value for a
     * given LimitCheckId.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned when
     * LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_limit_check_current(VL53L0X_DEV dev, uint16_t limit_check_id,
            FixPoint1616_t *p_limit_check_current);

    /**
     * @brief  Return a the Status of the specified check limit
     *
     * @par Function Description
     * This function returns the Status of the specified check limit.
     * The value indicate if the check is fail or not.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   p_limit_check_status             Pointer to the
     Limit Check Status of the given check limit.
     * LimitCheckStatus :
     * 0 the check is not fail
     * 1 the check if fail or not enabled
     *
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is
     returned when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_limit_check_status(VL53L0X_DEV dev,
            uint16_t limit_check_id, uint8_t *p_limit_check_status);

    /**
     * Get continuous mode Inter-Measurement period in milliseconds
     *
     * @par Function Description
     * When trying to set too short time return  INVALID_PARAMS minimal value
     *
     * @note This function Access to the device
     *
     * @param   dev                                  Device Handle
     * @param   p_inter_measurement_period_milli_seconds  Pointer to programmed
     *  Inter-Measurement Period in milliseconds.
     * @return  VL53L0X_ERROR_NONE                    Success
     * @return  "Other error code"                   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_inter_measurement_period_milli_seconds(VL53L0X_DEV dev,
            uint32_t *p_inter_measurement_period_milli_seconds);

    /**
     * Program continuous mode Inter-Measurement period in milliseconds
     *
     * @par Function Description
     * When trying to set too short time return  INVALID_PARAMS minimal value
     *
     * @note This function Access to the device
     *
     * @param   dev                                  Device Handle
     * @param   inter_measurement_period_milli_seconds   Inter-Measurement Period in ms.
     * @return  VL53L0X_ERROR_NONE                    Success
     * @return  "Other error code"                   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_inter_measurement_period_milli_seconds(
        VL53L0X_DEV dev, uint32_t inter_measurement_period_milli_seconds);

    /**
     * @brief Set new device address
     *
     * After completion the device will answer to the new address programmed.
     * This function should be called when several devices are used in parallel
     * before start programming the sensor.
     * When a single device us used, there is no need to call this function.
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @param   device_address         The new Device address
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_device_address(VL53L0X_DEV dev, uint8_t device_address);

    /**
     * @brief Do an hard reset or soft reset (depending on implementation) of the
     * device \nAfter call of this function, device must be in same state as right
     * after a power-up sequence.This function will change the VL53L0X_State to
     * VL53L0X_STATE_POWERDOWN.
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_reset_device(VL53L0X_DEV dev);

    /**
     * @brief  Get setup of Wrap around Check
     *
     * @par Function Description
     * This function get the wrapAround check enable parameters
     *
     * @note This function Access to the device
     *
     * @param   dev                     Device Handle
     * @param   p_wrap_around_check_enable  Pointer to the Wrap around Check state
     *                                  0=disabled or 1 = enabled
     * @return  VL53L0X_ERROR_NONE       Success
     * @return  "Other error code"      See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_wrap_around_check_enable(VL53L0X_DEV dev,
            uint8_t *p_wrap_around_check_enable);

    /**
     * @brief  Enable (or disable) Wrap around Check
     *
     * @note This function Access to the device
     *
     * @param   dev                    Device Handle
     * @param   wrap_around_check_enable  Wrap around Check to be set
     *                                 0=disabled, other = enabled
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"     See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_wrap_around_check_enable(VL53L0X_DEV dev,
            uint8_t wrap_around_check_enable);

    /**
     * @brief Gets the VCSEL pulse period.
     *
     * @par Function Description
     * This function retrieves the VCSEL pulse period for the given period type.
     *
     * @note This function Accesses the device
     *
     * @param   dev                      Device Handle
     * @param   vcsel_period_type          VCSEL period identifier (pre-range|final).
     * @param   p_vcsel_pulse_period_pclk        Pointer to VCSEL period value.
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error VcselPeriodType parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV dev,
            VL53L0X_VcselPeriod vcsel_period_type, uint8_t *p_vcsel_pulse_period_pclk);

    /**
     * @brief Sets the VCSEL pulse period.
     *
     * @par Function Description
     * This function retrieves the VCSEL pulse period for the given period type.
     *
     * @note This function Accesses the device
     *
     * @param   dev                       Device Handle
     * @param   vcsel_period_type	      VCSEL period identifier (pre-range|final).
     * @param   vcsel_pulse_period          VCSEL period value
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error VcselPeriodType parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV dev,
            VL53L0X_VcselPeriod vcsel_period_type, uint8_t vcsel_pulse_period);

    /**
     * @brief Set low and high Interrupt thresholds for a given mode
     * (ranging, ALS, ...) for a given device
     *
     * @par Function Description
     * Set low and high Interrupt thresholds for a given mode (ranging, ALS, ...)
     * for a given device
     *
     * @note This function Access to the device
     *
     * @note DeviceMode is ignored for the current device
     *
     * @param   dev              Device Handle
     * @param   device_mode       Device Mode for which change thresholds
     * @param   threshold_low     Low threshold (mm, lux ..., depending on the mode)
     * @param   threshold_high    High threshold (mm, lux ..., depending on the mode)
     * @return  VL53L0X_ERROR_NONE    Success
     * @return  "Other error code"   See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_interrupt_thresholds(VL53L0X_DEV dev,
            VL53L0X_DeviceModes device_mode, FixPoint1616_t threshold_low,
            FixPoint1616_t threshold_high);

    /**
     * @brief  Get high and low Interrupt thresholds for a given mode
     *  (ranging, ALS, ...) for a given device
     *
     * @par Function Description
     * Get high and low Interrupt thresholds for a given mode (ranging, ALS, ...)
     * for a given device
     *
     * @note This function Access to the device
     *
     * @note DeviceMode is ignored for the current device
     *
     * @param   dev              Device Handle
     * @param   device_mode       Device Mode from which read thresholds
     * @param   p_threshold_low    Low threshold (mm, lux ..., depending on the mode)
     * @param   p_threshold_high   High threshold (mm, lux ..., depending on the mode)
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_interrupt_thresholds(VL53L0X_DEV dev,
            VL53L0X_DeviceModes device_mode, FixPoint1616_t *p_threshold_low,
            FixPoint1616_t *p_threshold_high);

    /**
     * @brief Reads the Device information for given Device
     *
     * @note This function Access to the device
     *
     * @param   dev                 Device Handle
     * @param   p_VL53L0X_device_info  Pointer to current device info for a given
     *  Device
     * @return  VL53L0X_ERROR_NONE   Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_device_info(VL53L0X_DEV dev,
                                          VL53L0X_DeviceInfo_t *p_VL53L0X_device_info);

    /**
     * @brief Gets the (on/off) state of all sequence steps.
     *
     * @par Function Description
     * This function retrieves the state of all sequence step in the scheduler.
     *
     * @note This function Accesses the device
     *
     * @param   dev                          Device Handle
     * @param   p_scheduler_sequence_steps      Pointer to struct containing result.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_sequence_step_enables(VL53L0X_DEV dev,
            VL53L0X_SchedulerSequenceSteps_t *p_scheduler_sequence_steps);

    /**
     * @brief Sets the (on/off) state of a requested sequence step.
     *
     * @par Function Description
     * This function enables/disables a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   dev                          Device Handle
     * @param   sequence_step_id	         Sequence step identifier.
     * @param   sequence_step_enabled          Demanded state {0=Off,1=On}
     *                                       is enabled.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_sequence_step_enable(VL53L0X_DEV dev,
            VL53L0X_SequenceStepId sequence_step_id, uint8_t sequence_step_enabled);

    /**
     * @brief  Gets the fraction enable parameter indicating the resolution of
     * range measurements.
     *
     * @par Function Description
     * Gets the fraction enable state, which translates to the resolution of
     * range measurements as follows :Enabled:=0.25mm resolution,
     * Not Enabled:=1mm resolution.
     *
     * @note This function Accesses the device
     *
     * @param   dev               Device Handle
     * @param   p_enabled           Output Parameter reporting the fraction enable state.
     *
     * @return  VL53L0X_ERROR_NONE                   Success
     * @return  "Other error code"                  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_fraction_enable(VL53L0X_DEV dev, uint8_t *p_enabled);

    /**
     * @brief  Sets the resolution of range measurements.
     * @par Function Description
     * Set resolution of range measurements to either 0.25mm if
     * fraction enabled or 1mm if not enabled.
     *
     * @note This function Accesses the device
     *
     * @param   dev               Device Handle
     * @param   enable            Enable high resolution
     *
     * @return  VL53L0X_ERROR_NONE               Success
     * @return  "Other error code"              See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_range_fraction_enable(VL53L0X_DEV dev,
            uint8_t enable);

    /**
    * @brief Return the VL53L0X PAL Implementation Version
    *
    * @note This function doesn't access to the device
    *
    * @param   p_version              Pointer to current PAL Implementation Version
    * @return  VL53L0X_ERROR_NONE     Success
    * @return  "Other error code"    See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_version(VL53L0X_Version_t *p_version);

    /**
     * @brief Reads the Product Revision for a for given Device
     * This function can be used to distinguish cut1.0 from cut1.1.
     *
     * @note This function Access to the device
     *
     * @param   dev                 Device Handle
     * @param   p_product_revision_major  Pointer to Product Revision Major
     * for a given Device
     * @param   p_product_revision_minor  Pointer to Product Revision Minor
     * for a given Device
     * @return  VL53L0X_ERROR_NONE      Success
     * @return  "Other error code"  See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_product_revision(VL53L0X_DEV dev,
            uint8_t *p_product_revision_major, uint8_t *p_product_revision_minor);

    /**
     * @brief  Retrieve current device parameters
     * @par Function Description
     * Get actual parameters of the device
     * @li Then start ranging operation.
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @param   p_device_parameters     Pointer to store current device parameters.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_device_parameters(VL53L0X_DEV dev,
            VL53L0X_DeviceParameters_t *p_device_parameters);

    /**
     * @brief Human readable error string for current PAL error status
     *
     * @note This function doesn't access to the device
     *
     * @param   pal_error_code       The error code as stored on @a VL53L0X_Error
     * @param   p_pal_error_string    The error string corresponding to the
     * PalErrorCode
     * @return  VL53L0X_ERROR_NONE  Success
     * @return  "Other error code" See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_pal_error_string(VL53L0X_Error pal_error_code,
            char *p_pal_error_string);

    /**
     * @brief Return the PAL Specification Version used for the current
     * implementation.
     *
     * @note This function doesn't access to the device
     *
     * @param   p_pal_spec_version       Pointer to current PAL Specification Version
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_pal_spec_version(
        VL53L0X_Version_t *p_pal_spec_version);

    /**
     * @brief Reads the internal state of the PAL for a given Device
     *
     * @note This function doesn't access to the device
     *
     * @param   dev                   Device Handle
     * @param   p_pal_state             Pointer to current state of the PAL for a
     * given Device
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_pal_state(VL53L0X_DEV dev,
                                        VL53L0X_State *p_pal_state);

    /**
     * @brief Human readable PAL State string
     *
     * @note This function doesn't access to the device
     *
     * @param   pal_state_code          The State code as stored on @a VL53L0X_State
     * @param   p_pal_state_string       The State string corresponding to the
     * PalStateCode
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_pal_state_string(VL53L0X_State pal_state_code,
            char *p_pal_state_string);

    /*** End High level API ***/
private:
    /* api.h functions */

    /**
     * @brief Wait for device booted after chip enable (hardware standby)
     * This function can be run only when VL53L0X_State is VL53L0X_STATE_POWERDOWN.
     *
     * @note This function is not Implemented
     *
     * @param   dev      Device Handle
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED Not implemented
     *
     */
    VL53L0X_Error VL53L0X_wait_device_booted(VL53L0X_DEV dev);


    VL53L0X_Error sequence_step_enabled(VL53L0X_DEV dev,
                                        VL53L0X_SequenceStepId sequence_step_id, uint8_t sequence_config,
                                        uint8_t *p_sequence_step_enabled);

    VL53L0X_Error VL53L0X_check_and_load_interrupt_settings(VL53L0X_DEV dev,
            uint8_t start_not_stopflag);


    /* api_core.h functions */

    VL53L0X_Error VL53L0X_get_info_from_device(VL53L0X_DEV dev, uint8_t option);

    VL53L0X_Error VL53L0X_device_read_strobe(VL53L0X_DEV dev);

    VL53L0X_Error wrapped_VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
            uint32_t *p_measurement_timing_budget_micro_seconds);

    VL53L0X_Error wrapped_VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV dev,
            VL53L0X_VcselPeriod vcsel_period_type, uint8_t *p_vcsel_pulse_period_pclk);

    uint8_t VL53L0X_decode_vcsel_period(uint8_t vcsel_period_reg);

    uint32_t VL53L0X_decode_timeout(uint16_t encoded_timeout);

    uint32_t VL53L0X_calc_timeout_us(VL53L0X_DEV dev,
                                     uint16_t timeout_period_mclks,
                                     uint8_t vcsel_period_pclks);

    uint32_t VL53L0X_calc_macro_period_ps(VL53L0X_DEV dev, uint8_t vcsel_period_pclks);

    VL53L0X_Error VL53L0X_measurement_poll_for_completion(VL53L0X_DEV dev);

    VL53L0X_Error VL53L0X_load_tuning_settings(VL53L0X_DEV dev,
            uint8_t *p_tuning_setting_buffer);

    VL53L0X_Error VL53L0X_get_pal_range_status(VL53L0X_DEV dev,
            uint8_t device_range_status,
            FixPoint1616_t signal_rate,
            uint16_t effective_spad_rtn_count,
            VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
            uint8_t *p_pal_range_status);
    VL53L0X_Error VL53L0X_calc_sigma_estimate(VL53L0X_DEV dev,
            VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
            FixPoint1616_t *p_sigma_estimate,
            uint32_t *p_dmax_mm);
    uint32_t VL53L0X_calc_timeout_mclks(VL53L0X_DEV dev,
                                        uint32_t timeout_period_us,
                                        uint8_t vcsel_period_pclks);
    uint32_t VL53L0X_isqrt(uint32_t num);

    uint32_t VL53L0X_quadrature_sum(uint32_t a, uint32_t b);

    VL53L0X_Error VL53L0X_calc_dmax(
        VL53L0X_DEV dev,
        FixPoint1616_t total_signal_rate_mcps,
        FixPoint1616_t total_corr_signal_rate_mcps,
        FixPoint1616_t pw_mult,
        uint32_t sigma_estimate_p1,
        FixPoint1616_t sigma_estimate_p2,
        uint32_t peak_vcsel_duration_us,
        uint32_t *pd_max_mm);
    VL53L0X_Error wrapped_VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
            uint32_t measurement_timing_budget_micro_seconds);
    VL53L0X_Error get_sequence_step_timeout(VL53L0X_DEV dev,
                                            VL53L0X_SequenceStepId sequence_step_id,
                                            uint32_t *p_time_out_micro_secs);
    VL53L0X_Error set_sequence_step_timeout(VL53L0X_DEV dev,
                                            VL53L0X_SequenceStepId sequence_step_id,
                                            uint32_t timeout_micro_secs);
    uint16_t VL53L0X_encode_timeout(uint32_t timeout_macro_clks);
    VL53L0X_Error wrapped_VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV dev,
            VL53L0X_VcselPeriod vcsel_period_type, uint8_t vcsel_pulse_period_pclk);
    uint8_t lv53l0x_encode_vcsel_period(uint8_t vcsel_period_pclks);

    /* api_calibration.h functions */
    VL53L0X_Error VL53L0X_apply_offset_adjustment(VL53L0X_DEV dev);
    VL53L0X_Error wrapped_VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
            int32_t *p_offset_calibration_data_micro_meter);
    VL53L0X_Error wrapped_VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
            int32_t offset_calibration_data_micro_meter);
    VL53L0X_Error wrapped_VL53L0X_perform_ref_spad_management(VL53L0X_DEV dev,
            uint32_t *ref_spad_count,
            uint8_t *is_aperture_spads);
    VL53L0X_Error VL53L0X_perform_ref_calibration(VL53L0X_DEV dev,
            uint8_t *p_vhv_settings, uint8_t *p_phase_cal, uint8_t get_data_enable);
    VL53L0X_Error VL53L0X_perform_vhv_calibration(VL53L0X_DEV dev,
            uint8_t *p_vhv_settings, const uint8_t get_data_enable,
            const uint8_t restore_config);
    VL53L0X_Error VL53L0X_perform_single_ref_calibration(VL53L0X_DEV dev,
            uint8_t vhv_init_byte);
    VL53L0X_Error VL53L0X_ref_calibration_io(VL53L0X_DEV dev, uint8_t read_not_write,
            uint8_t vhv_settings, uint8_t phase_cal,
            uint8_t *p_vhv_settings, uint8_t *p_phase_cal,
            const uint8_t vhv_enable, const uint8_t phase_enable);
    VL53L0X_Error VL53L0X_perform_phase_calibration(VL53L0X_DEV dev,
            uint8_t *p_phase_cal, const uint8_t get_data_enable,
            const uint8_t restore_config);
    VL53L0X_Error enable_ref_spads(VL53L0X_DEV dev,
                                   uint8_t aperture_spads,
                                   uint8_t good_spad_array[],
                                   uint8_t spad_array[],
                                   uint32_t size,
                                   uint32_t start,
                                   uint32_t offset,
                                   uint32_t spad_count,
                                   uint32_t *p_last_spad);
    void get_next_good_spad(uint8_t good_spad_array[], uint32_t size,
                            uint32_t curr, int32_t *p_next);
    uint8_t is_aperture(uint32_t spad_index);
    VL53L0X_Error enable_spad_bit(uint8_t spad_array[], uint32_t size,
                                  uint32_t spad_index);
    VL53L0X_Error set_ref_spad_map(VL53L0X_DEV dev, uint8_t *p_ref_spad_array);
    VL53L0X_Error get_ref_spad_map(VL53L0X_DEV dev, uint8_t *p_ref_spad_array);
    VL53L0X_Error perform_ref_signal_measurement(VL53L0X_DEV dev,
            uint16_t *p_ref_signal_rate);
    VL53L0X_Error wrapped_VL53L0X_set_reference_spads(VL53L0X_DEV dev,
            uint32_t count, uint8_t is_aperture_spads);

    /* api_strings.h functions */
    VL53L0X_Error wrapped_VL53L0X_get_device_info(VL53L0X_DEV dev,
            VL53L0X_DeviceInfo_t *p_VL53L0X_device_info);
    VL53L0X_Error VL53L0X_check_part_used(VL53L0X_DEV dev,
                                          uint8_t *revision,
                                          VL53L0X_DeviceInfo_t *p_VL53L0X_device_info);

    /* Read function of the ID device */
    //   virtual int read_id();
    virtual int read_id(uint8_t *id);

    VL53L0X_Error wait_measurement_data_ready(VL53L0X_DEV dev);

    VL53L0X_Error wait_stop_completed(VL53L0X_DEV dev);

    /* Write and read functions from I2C */
    /**
     * Write single byte register
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   data      8 bit register data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_write_byte(VL53L0X_DEV dev, uint8_t index, uint8_t data);
    /**
     * Write word register
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   data      16 bit register data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_write_word(VL53L0X_DEV dev, uint8_t index, uint16_t data);
    /**
     * Write double word (4 byte) register
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   data      32 bit register data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_write_dword(VL53L0X_DEV dev, uint8_t index, uint32_t data);
    /**
     * Read single byte register
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   data      pointer to 8 bit data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_read_byte(VL53L0X_DEV dev, uint8_t index, uint8_t *p_data);
    /**
     * Read word (2byte) register
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   data      pointer to 16 bit data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_read_word(VL53L0X_DEV dev, uint8_t index, uint16_t *p_data);
    /**
     * Read dword (4byte) register
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   data      pointer to 32 bit data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_read_dword(VL53L0X_DEV dev, uint8_t index, uint32_t *p_data);
    /**
     * Threat safe Update (read/modify/write) single byte register
     *
     * Final_reg = (Initial_reg & and_data) |or_data
     *
     * @param   dev        Device Handle
     * @param   index      The register index
     * @param   and_data    8 bit and data
     * @param   or_data     8 bit or data
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_update_byte(VL53L0X_DEV dev, uint8_t index, uint8_t and_data, uint8_t or_data);
    /**
     * Writes the supplied byte buffer to the device
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   p_data     Pointer to uint8_t buffer containing the data to be written
     * @param   count     Number of bytes in the supplied byte buffer
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_write_multi(VL53L0X_DEV dev, uint8_t index, uint8_t *p_data, uint32_t count);
    /**
     * Reads the requested number of bytes from the device
     * @param   dev       Device Handle
     * @param   index     The register index
     * @param   p_data     Pointer to the uint8_t buffer to store read data
     * @param   count     Number of uint8_t's to read
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_read_multi(VL53L0X_DEV dev, uint8_t index, uint8_t *p_data, uint32_t count);

    /**
     * @brief  Writes a buffer towards the I2C peripheral device.
     * @param  dev       Device Handle
     * @param  p_data pointer to the byte-array data to send
     * @param  number_of_bytes number of bytes to be written.
     * @retval 0 if ok,
     * @retval -1 if an I2C error has occured
     * @note   On some devices if NumByteToWrite is greater
     *         than one, the RegisterAddr must be masked correctly!
     */
    VL53L0X_Error VL53L0X_i2c_write(uint8_t dev, uint8_t index, uint8_t *p_data, uint16_t number_of_bytes);

    /**
     * @brief  Reads a buffer from the I2C peripheral device.
     * @param  dev       Device Handle
     * @param  p_data pointer to the byte-array to read data in to
     * @param  number_of_bytes number of bytes to be read.
     * @retval 0 if ok,
     * @retval -1 if an I2C error has occured
     * @note   On some devices if NumByteToWrite is greater
     *         than one, the RegisterAddr must be masked correctly!
     */
    VL53L0X_Error VL53L0X_i2c_read(uint8_t dev, uint8_t index, uint8_t *p_data, uint16_t number_of_bytes);

    /**
     * @brief execute delay in all polling API call
     *
     * A typical multi-thread or RTOs implementation is to sleep the task for some 5ms (with 100Hz max rate faster polling is not needed)
     * if nothing specific is need you can define it as an empty/void macro
     * @code
     * #define VL53L0X_PollingDelay(...) (void)0
     * @endcode
     * @param dev       Device Handle
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_polling_delay(VL53L0X_DEV dev);   /* usually best implemented as a real function */

    int is_present()
    {
        int status;
        uint8_t id = 0;

        status = read_id(&id);
        if (status) {
            VL53L0X_ErrLog("Failed to read ID device. Device not present!\n\r");
        }
        return status;
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //Added functions                                                                                    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////

    /**
     * @brief  Cycle Power to Device
     *
     * @return status - status 0 = ok, 1 = error
     *
     */
    int32_t VL53L0X_cycle_power(void);

    uint8_t VL53L0X_encode_vcsel_period(uint8_t vcsel_period_pclks);

    VL53L0X_Error wrapped_VL53L0X_get_device_error_string(VL53L0X_DeviceError error_code,
            char *p_device_error_string);

    VL53L0X_Error wrapped_VL53L0X_get_limit_check_info(VL53L0X_DEV dev, uint16_t limit_check_id,
            char *p_limit_check_string);

    VL53L0X_Error wrapped_VL53L0X_get_pal_error_string(VL53L0X_Error pal_error_code,
            char *p_pal_error_string);

    VL53L0X_Error wrapped_VL53L0X_get_pal_state_string(VL53L0X_State pal_state_code,
            char *p_pal_state_string);

    VL53L0X_Error wrapped_VL53L0X_get_range_status_string(uint8_t range_status,
            char *p_range_status_string);

    VL53L0X_Error wrapped_VL53L0X_get_ref_calibration(VL53L0X_DEV dev,
            uint8_t *p_vhv_settings, uint8_t *p_phase_cal);


    VL53L0X_Error count_enabled_spads(uint8_t spad_array[],
                                      uint32_t byte_count, uint32_t max_spads,
                                      uint32_t *p_total_spads_enabled, uint8_t *p_is_aperture);

    VL53L0X_Error wrapped_VL53L0X_get_sequence_steps_info(VL53L0X_SequenceStepId sequence_step_id,
            char *p_sequence_steps_string);


    /**
     * @brief Gets the name of a given sequence step.
     *
     * @par Function Description
     * This function retrieves the name of sequence steps corresponding to
     * SequenceStepId.
     *
     * @note This function doesn't Accesses the device
     *
     * @param   sequence_step_id               Sequence step identifier.
     * @param   p_sequence_steps_string         Pointer to Info string
     *
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_sequence_steps_info(VL53L0X_SequenceStepId sequence_step_id,
            char *p_sequence_steps_string);

    /**
    * @brief Get the frequency of the timer used for ranging results time stamps
    *
    * @param[out] p_timer_freq_hz : pointer for timer frequency
    *
    * @return status : 0 = ok, 1 = error
    *
    */
    int32_t VL53L0X_get_timer_frequency(int32_t *p_timer_freq_hz);

    /**
    * @brief Get the timer value in units of timer_freq_hz (see VL53L0X_get_timestamp_frequency())
    *
    * @param[out] p_timer_count : pointer for timer count value
    *
    * @return status : 0 = ok, 1 = error
    *
    */
    int32_t VL53L0X_get_timer_value(int32_t *p_timer_count);

    /**
    * @brief Configure ranging interrupt reported to system
    *
    * @note This function is not Implemented
    *
    * @param   dev                  Device Handle
    * @param   interrupt_mask         Mask of interrupt to Enable/disable
    *  (0:interrupt disabled or 1: interrupt enabled)
    * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
    */
    VL53L0X_Error VL53L0X_enable_interrupt_mask(VL53L0X_DEV dev,
            uint32_t interrupt_mask);

    /**
     * @brief  Get Dmax Calibration Parameters for a given device
     *
     *
     * @note This function Access to the device
     *
     * @param   dev                     Device Handle
     * @param   p_range_milli_meter        Pointer to Calibration Distance
     * @param   p_signal_rate_rtn_mega_cps   Pointer to Signal rate return
     * @return  VL53L0X_ERROR_NONE       Success
     * @return  "Other error code"      See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_dmax_cal_parameters(VL53L0X_DEV dev,
            uint16_t *p_range_milli_meter, FixPoint1616_t *p_signal_rate_rtn_mega_cps);

    /**
    * @brief   Set Dmax Calibration Parameters for a given device
    * When one of the parameter is zero, this function will get parameter
    * from NVM.
    * @note This function doesn't Access to the device
    *
    * @param   dev                    Device Handle
    * @param   range_milli_meter        Calibration Distance
    * @param   signal_rate_rtn_mega_cps   Signal rate return read at CalDistance
    * @return  VL53L0X_ERROR_NONE      Success
    * @return  "Other error code"     See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_dmax_cal_parameters(VL53L0X_DEV dev,
            uint16_t range_milli_meter, FixPoint1616_t signal_rate_rtn_mega_cps);

    /**
    * @brief Retrieve the measurements from device for a given setup
    *
    * @par Function Description
    * Get data from last successful Histogram measurement
    * @warning USER should take care about  @a VL53L0X_GetNumberOfROIZones()
    * before get data.
    * PAL will fill a NumberOfROIZones times the corresponding data structure
    * used in the measurement function.
    *
    * @note This function is not Implemented
    *
    * @param   dev                         Device Handle
    * @param   p_histogram_measurement_data   Pointer to the histogram data structure.
    * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
    */
    VL53L0X_Error VL53L0X_get_histogram_measurement_data(VL53L0X_DEV dev,
            VL53L0X_HistogramMeasurementData_t *p_histogram_measurement_data);

    /**
    * @brief  Get current new device mode
    * @par Function Description
    * Get current Histogram mode of a Device
    *
    * @note This function doesn't Access to the device
    *
    * @param   dev                   Device Handle
    * @param   p_histogram_mode        Pointer to current Histogram Mode value
    *                                Valid values are:
    *                                VL53L0X_HISTOGRAMMODE_DISABLED
    *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
    *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
    *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
    *                                VL53L0X_HISTOGRAMMODE_BOTH
    * @return  VL53L0X_ERROR_NONE     Success
    * @return  "Other error code"    See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_histogram_mode(VL53L0X_DEV dev,
            VL53L0X_HistogramModes *p_histogram_mode);

    /**
     * @brief  Set a new Histogram mode
     * @par Function Description
     * Set device to a new Histogram mode
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                   Device Handle
     * @param   histogram_mode         New device mode to apply
     *                                Valid values are:
     *                                VL53L0X_HISTOGRAMMODE_DISABLED
     *                                VL53L0X_DEVICEMODE_SINGLE_HISTOGRAM
     *                                VL53L0X_HISTOGRAMMODE_REFERENCE_ONLY
     *                                VL53L0X_HISTOGRAMMODE_RETURN_ONLY
     *                                VL53L0X_HISTOGRAMMODE_BOTH
     *
     * @return  VL53L0X_ERROR_NONE                   Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED     This error occurs when
     * HistogramMode is not in the supported list
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_histogram_mode(VL53L0X_DEV dev,
            VL53L0X_HistogramModes histogram_mode);

    /**
     * @brief  Return a description string for a given limit check number
     *
     * @par Function Description
     * This function returns a description string for a given limit check number.
     * The limit check is identified with the LimitCheckId.
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                           Device Handle
     * @param   limit_check_id                  Limit Check ID
     (0<= LimitCheckId < VL53L0X_GetNumberOfLimitCheck() ).
     * @param   p_limit_check_string             Pointer to the
     description string of the given check limit.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is
     returned when LimitCheckId value is out of range.
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_limit_check_info(VL53L0X_DEV dev,
            uint16_t limit_check_id, char *p_limit_check_string);

    /**
     * @brief Get the linearity corrective gain
     *
     * @par Function Description
     * Should only be used after a successful call to @a VL53L0X_DataInit to backup
     * device NVM value
     *
     * @note This function Access to the device
     *
     * @param   dev                                Device Handle
     * @param   p_linearity_corrective_gain           Pointer to the linearity
     * corrective gain in x1000
     * if value is 1000 then no modification is applied.
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                 See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_linearity_corrective_gain(VL53L0X_DEV dev,
            uint16_t *p_linearity_corrective_gain);

    /**
     * Set the linearity corrective gain
     *
     * @note This function Access to the device
     *
     * @param   dev                                Device Handle
     * @param   linearity_corrective_gain            Linearity corrective
     * gain in x1000
     * if value is 1000 then no modification is applied.
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  "Other error code"                 See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_linearity_corrective_gain(VL53L0X_DEV dev,
            int16_t linearity_corrective_gain);

    /**
     * @brief Get the Maximum number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get Maximum number of ROI Zones managed by the Device.
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                    Device Handle
     * @param   p_max_number_of_roi_zones   Pointer to the Maximum Number
     *  of ROI Zones value.
     * @return  VL53L0X_ERROR_NONE      Success
     */
    VL53L0X_Error VL53L0X_get_max_number_of_roi_zones(VL53L0X_DEV dev,
            uint8_t *p_max_number_of_roi_zones);

    /**
     * @brief Retrieve the Reference Signal after a measurements
     *
     * @par Function Description
     * Get Reference Signal from last successful Ranging measurement
     * This function return a valid value after that you call the
     * @a VL53L0X_GetRangingMeasurementData().
     *
     * @note This function Access to the device
     *
     * @param   dev                      Device Handle
     * @param   p_measurement_ref_signal    Pointer to the Ref Signal to fill up.
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"       See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_measurement_ref_signal(VL53L0X_DEV dev,
            FixPoint1616_t *p_measurement_ref_signal);

    /**
     * @brief  Get the number of the check limit managed by a given Device
     *
     * @par Function Description
     * This function give the number of the check limit managed by the Device
     *
     * @note This function doesn't Access to the device
     *
     * @param   p_number_of_limit_check           Pointer to the number of check limit.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_number_of_limit_check(
        uint16_t *p_number_of_limit_check);

    /**
     * @brief Get the number of ROI Zones managed by the Device
     *
     * @par Function Description
     * Get number of ROI Zones managed by the Device
     * USER should take care about  @a VL53L0X_GetNumberOfROIZones()
     * before get data after a perform measurement.
     * PAL will fill a NumberOfROIZones times the corresponding data
     * structure used in the measurement function.
     *
     * @note This function doesn't Access to the device
     *
     * @param   dev                   Device Handle
     * @param   p_number_of_roi_zones     Pointer to the Number of ROI Zones value.
     * @return  VL53L0X_ERROR_NONE     Success
     */
    VL53L0X_Error VL53L0X_get_number_of_roi_zones(VL53L0X_DEV dev,
            uint8_t *p_number_of_roi_zones);

    /**
     * @brief Set the number of ROI Zones to be used for a specific Device
     *
     * @par Function Description
     * Set the number of ROI Zones to be used for a specific Device.
     * The programmed value should be less than the max number of ROI Zones given
     * with @a VL53L0X_GetMaxNumberOfROIZones().
     * This version of API manage only one zone.
     *
     * @param   dev                           Device Handle
     * @param   number_of_roi_zones              Number of ROI Zones to be used for a
     *  specific Device.
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS   This error is returned if
     * NumberOfROIZones != 1
     */
    VL53L0X_Error VL53L0X_set_number_of_roi_zones(VL53L0X_DEV dev,
            uint8_t number_of_roi_zones);

    /**
     * @brief Gets number of sequence steps managed by the API.
     *
     * @par Function Description
     * This function retrieves the number of sequence steps currently managed
     * by the API
     *
     * @note This function Accesses the device
     *
     * @param   dev                          Device Handle
     * @param   p_number_of_sequence_steps       Out parameter reporting the number of
     *                                       sequence steps.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_number_of_sequence_steps(VL53L0X_DEV dev,
            uint8_t *p_number_of_sequence_steps);
    /**
     * @brief Get the power mode for a given Device
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @param   p_power_mode            Pointer to the current value of the power
     * mode. see ::VL53L0X_PowerModes
     *                                Valid values are:
     *                                VL53L0X_POWERMODE_STANDBY_LEVEL1,
     *                                VL53L0X_POWERMODE_IDLE_LEVEL1
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_power_mode(VL53L0X_DEV dev,
                                         VL53L0X_PowerModes *p_power_mode);

    /**
     * @brief Set the power mode for a given Device
     * The power mode can be Standby or Idle. Different level of both Standby and
     * Idle can exists.
     * This function should not be used when device is in Ranging state.
     *
     * @note This function Access to the device
     *
     * @param   dev                   Device Handle
     * @param   power_mode             The value of the power mode to set.
     * see ::VL53L0X_PowerModes
     *                                Valid values are:
     *                                VL53L0X_POWERMODE_STANDBY_LEVEL1,
     *                                VL53L0X_POWERMODE_IDLE_LEVEL1
     * @return  VL53L0X_ERROR_NONE                  Success
     * @return  VL53L0X_ERROR_MODE_NOT_SUPPORTED    This error occurs when PowerMode
     * is not in the supported list
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_power_mode(VL53L0X_DEV dev,
                                         VL53L0X_PowerModes power_mode);

    /**
     * @brief Retrieves SPAD configuration
     *
     * @par Function Description
     * This function retrieves the current number of applied reference spads
     * and also their type : Aperture or Non-Aperture.
     *
     * @note This function Access to the device
     *
     * @param   dev                          Device Handle
     * @param   p_spad_count                 Number ref Spad Count
     * @param   p_is_aperture_spads              Reports if spads are of type
     *                                       aperture or non-aperture.
     *                                       1:=aperture, 0:=Non-Aperture
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_REF_SPAD_INIT   Error in the in the reference
     *                                       spad configuration.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error wrapped_VL53L0X_get_reference_spads(VL53L0X_DEV dev,
            uint32_t *p_spad_count, uint8_t *p_is_aperture_spads);

    /**
     * @brief Gets the (on/off) state of a requested sequence step.
     *
     * @par Function Description
     * This function retrieves the state of a requested sequence step, i.e. on/off.
     *
     * @note This function Accesses the device
     *
     * @param   dev                    Device Handle
     * @param   sequence_step_id         Sequence step identifier.
     * @param   p_sequence_step_enabled   Out parameter reporting if the sequence step
     *                                 is enabled {0=Off,1=On}.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_sequence_step_enable(VL53L0X_DEV dev,
            VL53L0X_SequenceStepId sequence_step_id, uint8_t *p_sequence_step_enabled);


    /**
     * @brief Gets the timeout of a requested sequence step.
     *
     * @par Function Description
     * This function retrieves the timeout of a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   dev                          Device Handle
     * @param   sequence_step_id               Sequence step identifier.
     * @param   p_time_out_milli_secs            Timeout value.
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_sequence_step_timeout(VL53L0X_DEV dev,
            VL53L0X_SequenceStepId sequence_step_id,
            FixPoint1616_t *p_time_out_milli_secs);

    /**
     * @brief Sets the timeout of a requested sequence step.
     *
     * @par Function Description
     * This function sets the timeout of a requested sequence step.
     *
     * @note This function Accesses the device
     *
     * @param   dev                          Device Handle
     * @param   sequence_step_id               Sequence step identifier.
     * @param   time_out_milli_secs             Demanded timeout
     * @return  VL53L0X_ERROR_NONE            Success
     * @return  VL53L0X_ERROR_INVALID_PARAMS  Error SequenceStepId parameter not
     *                                       supported.
     * @return  "Other error code"           See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_sequence_step_timeout(VL53L0X_DEV dev,
            VL53L0X_SequenceStepId sequence_step_id, FixPoint1616_t time_out_milli_secs);

    /**
    * @brief  Get the current SPAD Ambient Damper Factor value
    *
    * @par Function Description
    * This function get the SPAD Ambient Damper Factor value
    *
    * @note This function Access to the device
    *
    * @param   dev                           Device Handle
    * @param   p_spad_ambient_damper_factor      Pointer to programmed SPAD Ambient
    * Damper Factor value
    * @return  VL53L0X_ERROR_NONE             Success
    * @return  "Other error code"            See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_spad_ambient_damper_factor(VL53L0X_DEV dev,
            uint16_t *p_spad_ambient_damper_factor);
    /**
    * @brief  Set the SPAD Ambient Damper Factor value
    *
    * @par Function Description
    * This function set the SPAD Ambient Damper Factor value
    *
    * @note This function Access to the device
    *
    * @param   dev                           Device Handle
    * @param   spad_ambient_damper_factor       SPAD Ambient Damper Factor value
    * @return  VL53L0X_ERROR_NONE             Success
    * @return  "Other error code"            See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_set_spad_ambient_damper_factor(VL53L0X_DEV dev,
            uint16_t spad_ambient_damper_factor);

    /**
     * @brief  Get the current SPAD Ambient Damper Threshold value
     *
     * @par Function Description
     * This function get the SPAD Ambient Damper Threshold value
     *
     * @note This function Access to the device
     *
     * @param   dev                           Device Handle
     * @param   p_spad_ambient_damper_threshold   Pointer to programmed
     *                                        SPAD Ambient Damper Threshold value
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_get_spad_ambient_damper_threshold(VL53L0X_DEV dev,
            uint16_t *p_spad_ambient_damper_threshold);

    /**
     * @brief  Set the SPAD Ambient Damper Threshold value
     *
     * @par Function Description
     * This function set the SPAD Ambient Damper Threshold value
     *
     * @note This function Access to the device
     *
     * @param   dev                           Device Handle
     * @param   spad_ambient_damper_threshold    SPAD Ambient Damper Threshold value
     * @return  VL53L0X_ERROR_NONE             Success
     * @return  "Other error code"            See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_spad_ambient_damper_threshold(VL53L0X_DEV dev,
            uint16_t spad_ambient_damper_threshold);

    /**
     * @brief Get the maximal distance for actual setup
     * @par Function Description
     * Device must be initialized through @a VL53L0X_SetParameters() prior calling
     * this function.
     *
     * Any range value more than the value returned is to be considered as
     * "no target detected" or
     * "no target in detectable range"\n
     * @warning The maximal distance depends on the setup
     *
     * @note This function is not Implemented
     *
     * @param   dev      Device Handle
     * @param   p_upper_limit_milli_meter   The maximal range limit for actual setup
     * (in millimeter)
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED        Not implemented
     */
    VL53L0X_Error VL53L0X_get_upper_limit_milli_meter(VL53L0X_DEV dev,
            uint16_t *p_upper_limit_milli_meter);

    /**
    * @brief Get the tuning settings pointer and the internal external switch
    * value.
    *
    * This function is used to get the Tuning settings buffer pointer and the
    * value.
    * of the switch to select either external or internal tuning settings.
    *
    * @note This function Access to the device
    *
    * @param   dev                        Device Handle
    * @param   pp_tuning_setting_buffer      Pointer to tuning settings buffer.
    * @param   p_use_internal_tuning_settings Pointer to store Use internal tuning
    *                                     settings value.
    * @return  VL53L0X_ERROR_NONE          Success
    * @return  "Other error code"         See ::VL53L0X_Error
    */
    VL53L0X_Error VL53L0X_get_tuning_setting_buffer(VL53L0X_DEV dev,
            uint8_t **pp_tuning_setting_buffer, uint8_t *p_use_internal_tuning_settings);

    /**
     * @brief Set the tuning settings pointer
     *
     * This function is used to specify the Tuning settings buffer to be used
     * for a given device. The buffer contains all the necessary data to permit
     * the API to write tuning settings.
     * This function permit to force the usage of either external or internal
     * tuning settings.
     *
     * @note This function Access to the device
     *
     * @param   dev                             Device Handle
     * @param   p_tuning_setting_buffer            Pointer to tuning settings buffer.
     * @param   use_internal_tuning_settings       Use internal tuning settings value.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_set_tuning_setting_buffer(VL53L0X_DEV dev,
            uint8_t *p_tuning_setting_buffer, uint8_t use_internal_tuning_settings);

    /**
     * @defgroup VL53L0X_registerAccess_group PAL Register Access Functions
     * @brief    PAL Register Access Functions
     *  @{
     */

    /**
     * Lock comms interface to serialize all commands to a shared I2C interface for a specific device
     * @param   dev       Device Handle
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_lock_sequence_access(VL53L0X_DEV dev);

    /**
     * Unlock comms interface to serialize all commands to a shared I2C interface for a specific device
     * @param   dev       Device Handle
     * @return  VL53L0X_ERROR_NONE        Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error VL53L0X_unlock_sequence_access(VL53L0X_DEV dev);

    /**
     * @brief  Prepare device for operation
     * @par Function Description
     * Update device with provided parameters
     * @li Then start ranging operation.
     *
     * @note This function Access to the device
     *
     * @param   Dev                   Device Handle
     * @param   pDeviceParameters     Pointer to store current device parameters.
     * @return  VL53L0X_ERROR_NONE     Success
     * @return  "Other error code"    See ::VL53L0X_Error
     */
    VL53L0X_Error vl53L0x_set_device_parameters(VL53L0X_DEV Dev,
            const VL53L0X_DeviceParameters_t *pDeviceParameters);

    /**
     * Set Group parameter Hold state
     *
     * @par Function Description
     * Set or remove device internal group parameter hold
     *
     * @note This function is not Implemented
     *
     * @param   dev      Device Handle
     * @param   group_param_hold   Group parameter Hold state to be set (on/off)
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED        Not implemented
     */
    VL53L0X_Error VL53L0X_set_group_param_hold(VL53L0X_DEV dev,
            uint8_t group_param_hold);


    /**
     * @brief Wait for device ready for a new measurement command.
     * Blocking function.
     *
     * @note This function is not Implemented
     *
     * @param   dev      Device Handle
     * @param   max_loop    Max Number of polling loop (timeout).
     * @return  VL53L0X_ERROR_NOT_IMPLEMENTED   Not implemented
     */
    VL53L0X_Error VL53L0X_wait_device_ready_for_new_measurement(VL53L0X_DEV dev,
            uint32_t max_loop);

    VL53L0X_Error VL53L0X_reverse_bytes(uint8_t *data, uint32_t size);

    int range_meas_int_continuous_mode(void (*fptr)(void));


    VL53L0X_DeviceInfo_t _device_info;

    /* IO Device */
    DevI2C *_dev_i2c;
    /* Digital out pin */
    DigitalOut *_gpio0;
    /* GPIO expander */
    Stmpe1600DigiOut *_expgpio0;
    /* Measure detection IRQ */
    InterruptIn *_gpio1Int;
    /* Device data */
    VL53L0X_Dev_t _my_device;
    VL53L0X_DEV _device;
};


#endif /* _VL53L0X_CLASS_H_ */
