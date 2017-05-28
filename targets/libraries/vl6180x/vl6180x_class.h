/**
 ******************************************************************************
 * @file    vl6180x_class.h
 * @author  AST / EST
 * @version V0.0.1
 * @date    9-November-2015
 * @brief   Header file for component VL6180X
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
 *       without specific prior written permission.
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
//aus VL6180X_platform.h
#ifndef VL6180x_PLATFORM
#define VL6180x_PLATFORM

#ifndef __VL6180X_CLASS_H
#define __VL6180X_CLASS_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "vl6180x_def.h"
#include "mbed.h"

//aus vl6180x_cfg.h
#ifndef VL6180x_CFG_H_
#define VL6180x_CFG_H_

#define VL6180x_ALS_SUPPORT      0
#define VL6180x_HAVE_DMAX_RANGING   1
#define VL6180x_WRAP_AROUND_FILTER_SUPPORT   1
#define EXTENDED_RANGE_50CM     0

#if EXTENDED_RANGE_50CM
#define VL6180x_UPSCALE_SUPPORT -3
#define VL6180x_EXTENDED_RANGE  1
#else
#define VL6180x_UPSCALE_SUPPORT -1
#define VL6180x_EXTENDED_RANGE  0
#endif

#if (VL6180x_EXTENDED_RANGE) && (VL6180x_ALS_SUPPORT)
#warning "Als support should be OFF for extended range"
#endif

#endif

//ab hier nicht mehr aus VL6180X_cfg.h



//ab hier nicht mehr aus VL6180X_def.h

//aus VL6180X_platform.h
 
 #define VL6180x_SINGLE_DEVICE_DRIVER 	0
#define VL6180x_RANGE_STATUS_ERRSTRING  1
#define VL6180X_SAFE_POLLING_ENTER 	0
#define VL6180X_LOG_ENABLE              0
#define MY_LOG                          1

#define VL6180x_DEV_DATA_ATTR
#define ROMABLE_DATA


#if VL6180X_LOG_ENABLE
/*  dot not include non ansi here trace was a case */
#ifdef TRACE
#include "diag/trace.h"
extern volatile uint32_t g_TickCnt;
#define LOG_GET_TIME()  g_TickCnt
//#define LOG_GET_TIME()  HAL_GetTick()
#else
/* these is nto stm32 vl6180x GNuArm eclpse build*/
#define trace_printf(...) (void)0
#define LOG_GET_TIME() (int)0 /* add your code here expect to be an integer native (%d) type  value  */
#endif

#define LOG_FUNCTION_START(fmt, ... ) \
    trace_printf("beg %s start @%d\t" fmt "\n", __func__, LOG_GET_TIME(), ##__VA_ARGS__)

#define LOG_FUNCTION_END(status)\
        trace_printf("end %s @%d %d\n", __func__, LOG_GET_TIME(), (int)status)

#define LOG_FUNCTION_END_FMT(status, fmt, ... )\
        trace_printf("End %s @%d %d\t"fmt"\n" , __func__, LOG_GET_TIME(), (int)status, ##__VA_ARGS__)

#define VL6180x_ErrLog(msg, ... )\
    do{\
        trace_printf("ERR in %s line %d\n" msg, __func__, __LINE__, ##__VA_ARGS__);\
    }while(0)

#else /* VL6180X_LOG_ENABLE no logging */
  //void OnErrLog(void);
  #define LOG_FUNCTION_START(...) (void)0
  #define LOG_FUNCTION_END(...) (void)0
  #define LOG_FUNCTION_END_FMT(...) (void)0
  #define VL6180x_ErrLog(... ) //OnErrLog() //(void)0
#endif

  
#ifdef MY_LOG  /* define printf as pc.printf in order to change the baudrate */
 // extern Serial pc;
 // #define printf(...) pc.printf(__VA_ARGS__)
#endif


#if  VL6180x_SINGLE_DEVICE_DRIVER
    #error "VL6180x_SINGLE_DEVICE_DRIVER must be set"
#endif
	
struct MyVL6180Dev_t {
    struct VL6180xDevData_t Data;
    uint8_t I2cAddr;
    //uint8_t DevID;
    
    //misc flags for application 	
    unsigned Present:1;
    unsigned Ready:1;
};
typedef struct MyVL6180Dev_t *VL6180xDev_t;

#define VL6180xDevDataGet(dev, field) (dev->Data.field)
#define VL6180xDevDataSet(dev, field, data) (dev->Data.field)=(data)


#endif  /* VL6180x_PLATFORM */

//ab hier nicht mehr aus vl6180x_platform.h
 
/* data struct containing range measure, light measure and type of error provided to the user
   in case of invalid data range_mm=0xFFFFFFFF and lux=0xFFFFFFFF */	
typedef struct MeasureData 
{
   uint32_t range_mm;
   uint32_t lux;
   uint32_t range_error;
   uint32_t als_error;
   uint32_t int_error;
}MeasureData_t;

/* sensor operating modes */ 
typedef enum
{
   range_single_shot_polling=1,
   als_single_shot_polling,
   range_continuous_polling,
   als_continuous_polling,
   range_continuous_interrupt,
   als_continuous_interrupt,
   interleaved_mode_interrupt,
   range_continuous_polling_low_threshold,
   range_continuous_polling_high_threshold,
   range_continuous_polling_out_of_window,
   als_continuous_polling_low_threshold,
   als_continuous_polling_high_threshold,
   als_continuous_polling_out_of_window,
   range_continuous_interrupt_low_threshold,
   range_continuous_interrupt_high_threshold,
   range_continuous_interrupt_out_of_window,
   als_continuous_interrupt_low_threshold,
   als_continuous_interrupt_high_threshold,
   als_continuous_interrupt_out_of_window,
   range_continuous_als_single_shot,
   range_single_shot_als_continuous,
}OperatingMode;

/** default device address */
#define DEFAULT_DEVICE_ADDRESS		0x29

/* Classes -------------------------------------------------------------------*/
/** Class representing a VL6180X sensor component
 */
class VL6180X //: public RangeSensor//, public LightSensor
{
 public:
    /** Constructor 1 (DigitalOut)
     * @param[in] &i2c device I2C to be used for communication
     * @param[in] &pin Mbed DigitalOut pin to be used as component GPIO_0 CE
     * @param[in] &pin_gpio1 pin Mbed InterruptIn PinName to be used as component GPIO_1 INT
     * @param[in] DevAddr device address, 0x29 by default  
     */
 VL6180X(I2C &i2c, DigitalInOut &pin, PinName pin_gpio1, uint8_t DevAddr=DEFAULT_DEVICE_ADDRESS) :  _i2c(i2c), gpio0(&pin)
    {
       MyDevice.I2cAddr=DevAddr;		 
       MyDevice.Present=0;
       MyDevice.Ready=0;
       Device=&MyDevice;;
       //expgpio0=NULL;
       if (pin_gpio1 != NC) { gpio1Int = new InterruptIn(pin_gpio1); }
       	else { gpio1Int = NULL; }
    }  
    /** Constructor 2 (STMPE1600DigiOut)
     * @param[in] i2c device I2C to be used for communication
     * @param[in] &pin Gpio Expander STMPE1600DigiOut pin to be used as component GPIO_0 CE
     * @param[in] pin_gpio1 pin Mbed InterruptIn PinName to be used as component GPIO_1 INT
     * @param[in] device address, 0x29 by default  
     */		
   VL6180X(I2C &i2c, PinName pin_gpio1, uint8_t DevAddr=DEFAULT_DEVICE_ADDRESS) :  _i2c(i2c)
    {
       MyDevice.I2cAddr=DevAddr;		 
       MyDevice.Present=0;
       MyDevice.Ready=0;
       Device=&MyDevice;
       gpio0=NULL;		
       if (pin_gpio1 != NC) { gpio1Int = new InterruptIn(pin_gpio1); }
       	else { gpio1Int = NULL; } 		 
    }  	 
    
   /** Destructor
    */
     ~VL6180X(){ 
    	if (gpio1Int != NULL) delete gpio1Int;
    }  
    
    /* warning: VL6180X class inherits from GenericSensor, RangeSensor and LightSensor, that haven`t a destructor.
       The warning should request to introduce a virtual destructor to make sure to delete the object */

	/*** Interface Methods ***/	
	/*** High level API ***/		
	/**
	 * @brief       PowerOn the sensor
	 * @return      void
	 */		
    /* turns on the sensor */		 
    void VL6180x_On(void)
    {
       if(gpio0) 
	  *gpio0=1;
/*       else if(expgpio0) 
	  *expgpio0=1;*/
       
       MyDevice.I2cAddr=DEFAULT_DEVICE_ADDRESS;
       MyDevice.Ready=0;       
    } 

	/**
	 * @brief       PowerOff the sensor
	 * @return      void
	 */		
    /* turns off the sensor */
    void VL6180x_Off(void) 
    {
       if(gpio0) 
	  *gpio0=0;
/*       else if(expgpio0) 
	  *expgpio0=0;	*/
       
       MyDevice.I2cAddr=DEFAULT_DEVICE_ADDRESS;
       MyDevice.Ready=0;       
    }   
    
	/**
	 * @brief       Start the measure indicated by operating mode
	 * @param[in]   operating_mode specifies requested measure 
	 * @param[in]   fptr specifies call back function must be !NULL in case of interrupt measure	 
	 * @param[in]   low specifies measure low threashold in Lux or in mm according to measure
	 * @param[in]   high specifies measure high threashold in Lux or in mm according to measure
	 * @return      0 on Success
	 */					     
    int StartMeasurement(OperatingMode operating_mode, void (*fptr)(void), uint16_t low, uint16_t high);

	/**
	 * @brief       Get results for the measure indicated by operating mode
	 * @param[in]   operating_mode specifies requested measure results
	 * @param[out]  Data pointer to the MeasureData_t structure to read data in to
	 * @return      0 on Success
	 */					         
    int GetMeasurement(OperatingMode operating_mode, MeasureData_t *Data);		

	/**
	 * @brief       Stop the currently running measure indicate by operating_mode
	 * @param[in]   operating_mode specifies requested measure to stop
	 * @return      0 on Success
	 */					             
    int StopMeasurement(OperatingMode operating_mode);
     
 	/**
	 * @brief       Interrupt handling func to be called by user after an INT is occourred
	 * @param[in]   opeating_mode indicating the in progress measure
	 * @param[out]  Data pointer to the MeasureData_t structure to read data in to
	 * @return      0 on Success
	 */					          		
    int HandleIRQ(OperatingMode operating_mode, MeasureData_t *Data);    

	/**
	 * @brief       Enable interrupt measure IRQ
	 * @return      0 on Success
	 */					     
    void EnableInterruptMeasureDetectionIRQ(void) 
    {
       if (gpio1Int != NULL) gpio1Int->enable_irq();
    }

	/**
	 * @brief       Disable interrupt measure IRQ
	 * @return      0 on Success
	 */					          
    void DisableInterruptMeasureDetectionIRQ(void) 
    {
       if (gpio1Int != NULL) gpio1Int->disable_irq();
    }
	/*** End High level API ***/	          
	
	/**
	 * @brief       Attach a function to call when an interrupt is detected, i.e. measurement is ready
	 * @param[in]   fptr pointer to call back function to be called whenever an interrupt occours
	 * @return      0 on Success
	 */					                  
    void AttachInterruptMeasureDetectionIRQ(void (*fptr)(void))
    {
       if (gpio1Int != NULL) gpio1Int->rise(fptr);
    }
    
	/**
	 * @brief       Check the sensor presence
	 * @return      1 when device is present
	 */						
    unsigned Present()
    {
       return Device->Present;
    }
		
    /** Wrapper functions */	
/** @defgroup api_init Init functions
 *  @brief    API init functions
 *  @ingroup api_hl
 *  @{  
 */
/**
 * @brief Wait for device booted after chip enable (hardware standby)
 * @par Function Description
 * After Chip enable Application you can also simply wait at least 1ms to ensure device is ready
 * @warning After device chip enable (gpio0) de-asserted  user must wait gpio1 to get asserted (hardware standby).
 * or wait at least 400usec prior to do any low level access or api call .
 *
 * This function implements polling for standby but you must ensure 400usec from chip enable passed\n
 * @warning if device get prepared @a VL6180x_Prepare() re-using these function can hold indefinitely\n
 *
 * @param 		void
 * @return     0 on success
 */
    int WaitDeviceBooted()
    {
       return VL6180x_WaitDeviceBooted(Device);
    }

/**
 *
 * @brief One time device initialization
 *
 * To be called once and only once after device is brought out of reset (Chip enable) and booted see @a VL6180x_WaitDeviceBooted()
 *
 * @par Function Description
 * When not used after a fresh device "power up" or reset, it may return @a #CALIBRATION_WARNING
 * meaning wrong calibration data may have been fetched from device that can result in ranging offset error\n
 * If application cannot execute device reset or need to run VL6180x_InitData  multiple time
 * then it  must ensure proper offset calibration saving and restore on its own
 * by using @a VL6180x_GetOffsetCalibrationData() on first power up and then @a VL6180x_SetOffsetCalibrationData() all all subsequent init
 *
 * @param void
 * @return     0 on success,  @a #CALIBRATION_WARNING if failed
*/
int Init(void * NewAddr)
{
   int status;
 
   VL6180x_Off();
   VL6180x_On();
   
   status=VL6180x_WaitDeviceBooted(Device);
   if(status) {
      VL6180x_ErrLog("WaitDeviceBooted fail\n\r");      
   }  
   status=IsPresent();
   if(!status)
   {
      Device->Present=1;
      VL6180x_InitData(Device);
      if(status)
      {
 //        printf("Failed to init VL6180X sensor!\n\r");
         return status;
      }
      status=Prepare();
      if(status)
      {
//         printf("Failed to prepare VL6180X!\n\r");
         return status;
      }
      if(*(uint8_t*)NewAddr!=DEFAULT_DEVICE_ADDRESS)
      {
         status=SetI2CAddress(*(uint8_t*)NewAddr);
         if(status)
         {
//            printf("Failed to change I2C address!\n\r");
            return status;
         }
      }
      Device->Ready=1;
   }
   return status; 
}


/**
 * @brief Configure GPIO1 function and set polarity.
 * @par Function Description
 * To be used prior to arm single shot measure or start  continuous mode.
 *
 * The function uses @a VL6180x_SetupGPIOx() for setting gpio 1.
 * @warning  changing polarity can generate a spurious interrupt on pins.
 * It sets an interrupt flags condition that must be cleared to avoid polling hangs. \n
 * It is safe to run VL6180x_ClearAllInterrupt() just after.
 *
 * @param IntFunction   The interrupt functionality to use one of :\n
 *  @a #GPIOx_SELECT_OFF \n
 *  @a #GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT
 * @param ActiveHigh  The interrupt line polarity see ::IntrPol_e
 *      use @a #INTR_POL_LOW (falling edge) or @a #INTR_POL_HIGH (rising edge)
 * @return 0 on success
 */		
    int SetupGPIO1(uint8_t InitFunction, int ActiveHigh)
    {
       return VL6180x_SetupGPIO1(Device, InitFunction, ActiveHigh);
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
  * @return      0 on success
  */		
    int Prepare()
    {
       return VL6180x_Prepare(Device);
    }

 /**
 * @brief Start continuous ranging mode
 *
 * @details End user should ensure device is in idle state and not already running
 * @return      0 on success
 */		
    int RangeStartContinuousMode()
    {
       return VL6180x_RangeStartContinuousMode(Device);
    }

/**
 * @brief Start single shot ranging measure
 *
 * @details End user should ensure device is in idle state and not already running
 * @return      0 on success 
 */		
    int RangeStartSingleShot()
    {
       return VL6180x_RangeStartSingleShot(Device);
    }

/**
 * @brief Set maximum convergence time
 *
 * @par Function Description
 * Setting a low convergence time can impact maximal detectable distance.
 * Refer to VL6180x Datasheet Table 7 : Typical range convergence time.
 * A typical value for up to x3 scaling is 50 ms
 *
 * @param MaxConTime_msec
 * @return 0 on success. <0 on error. >0 for calibration warning status
 */		
    int RangeSetMaxConvergenceTime(uint8_t MaxConTime_msec)
    {
       return VL6180x_RangeSetMaxConvergenceTime(Device, MaxConTime_msec);
    }

/**
  * @brief Single shot Range measurement in polling mode.
  *
  * @par Function Description
  * Kick off a new single shot range  then wait for ready to retrieve it by polling interrupt status \n
  * Ranging must be prepared by a first call to  @a VL6180x_Prepare() and it is safer to clear  very first poll call \n
  * This function reference VL6180x_PollDelay(dev) porting macro/call on each polling loop,
  * but PollDelay(dev) may never be called if measure in ready on first poll loop \n
  * Should not be use in continuous mode operation as it will stop it and cause stop/start misbehaviour \n
  * \n This function clears Range Interrupt status , but not error one. For that uses  @a VL6180x_ClearErrorInterrupt() \n
  * This range error is not related VL6180x_RangeData_t::errorStatus that refer measure status \n
  * 
  * @param pRangeData   Will be populated with the result ranging data @a  VL6180x_RangeData_t
  * @return 0 on success , @a #RANGE_ERROR if device reports an error case in it status (not cleared) use
  *
  * \sa ::VL6180x_RangeData_t
  */		
    int RangePollMeasurement(VL6180x_RangeData_t *pRangeData)
    {
       return VL6180x_RangePollMeasurement(Device, pRangeData);
    }

/**
 * @brief Check for measure readiness and get it if ready
 *
 * @par Function Description
 * Using this function is an alternative to @a VL6180x_RangePollMeasurement() to avoid polling operation. This is suitable for applications
 * where host CPU is triggered on a interrupt (not from VL6180X) to perform ranging operation. In this scenario, we assume that the very first ranging
 * operation is triggered by a call to @a VL6180x_RangeStartSingleShot(). Then, host CPU regularly calls @a VL6180x_RangeGetMeasurementIfReady() to
 * get a distance measure if ready. In case the distance is not ready, host may get it at the next call.\n
 *
 * @warning 
 * This function does not re-start a new measurement : this is up to the host CPU to do it.\n 
 * This function clears Range Interrupt for measure ready , but not error interrupts. For that, uses  @a VL6180x_ClearErrorInterrupt() \n
 *
 * @param pRangeData  Will be populated with the result ranging data if available
 * @return  0 when measure is ready pRange data is updated (untouched when not ready),  >0 for warning and @a #NOT_READY if measurement not yet ready, <0 for error @a #RANGE_ERROR if device report an error,
 */		
    int RangeGetMeasurementIfReady(VL6180x_RangeData_t *pRangeData)
    {
       return VL6180x_RangeGetMeasurementIfReady(Device, pRangeData);
    }

/**
 * @brief Retrieve range measurements set  from device
 *
 * @par Function Description
 * The measurement is made of range_mm status and error code @a VL6180x_RangeData_t \n
 * Based on configuration selected extra measures are included.
 *
 * @warning should not be used in continuous if wrap around filter is active \n
 * Does not perform any wait nor check for result availability or validity.
 *\sa VL6180x_RangeGetResult for "range only" measurement
 *
 * @param pRangeData  Pointer to the data structure to fill up
 * @return            0 on success
 */		
    int RangeGetMeasurement(VL6180x_RangeData_t *pRangeData)
    {
       return VL6180x_RangeGetMeasurement(Device, pRangeData);
    }

/**
 * @brief Get ranging result and only that
 *
 * @par Function Description
 * Unlike @a VL6180x_RangeGetMeasurement() this function only retrieves the range in millimeter \n
 * It does any required up-scale translation\n
 * It can be called after success status polling or in interrupt mode \n
 * @warning these function is not doing wrap around filtering \n
 * This function doesn't perform any data ready check!
 *
 * @param pRange_mm  Pointer to range distance
 * @return           0 on success
 */		
    int GetRange(int32_t *piData)
    {
       return VL6180x_RangeGetResult(Device, piData);
    }
		
/**
 * @brief Configure ranging interrupt reported to application
 *
 * @param ConfigGpioInt  Select ranging report\n select one (and only one) of:\n
 *   @a #CONFIG_GPIO_INTERRUPT_DISABLED \n
 *   @a #CONFIG_GPIO_INTERRUPT_LEVEL_LOW \n
 *   @a #CONFIG_GPIO_INTERRUPT_LEVEL_HIGH \n
 *   @a #CONFIG_GPIO_INTERRUPT_OUT_OF_WINDOW \n
 *   @a #CONFIG_GPIO_INTERRUPT_NEW_SAMPLE_READY
 * @return   0 on success
 */
    int RangeConfigInterrupt(uint8_t ConfigGpioInt)
    {
       return VL6180x_RangeConfigInterrupt(Device, ConfigGpioInt);
    }

/**
 * @brief Return ranging error interrupt status
 *
 * @par Function Description
 * Appropriate Interrupt report must have been selected first by @a VL6180x_RangeConfigInterrupt() or @a  VL6180x_Prepare() \n
 *
 * Can be used in polling loop to wait for a given ranging event or in interrupt to read the trigger \n
 * Events triggers are : \n
 * @a #RES_INT_STAT_GPIO_LOW_LEVEL_THRESHOLD \n
 * @a #RES_INT_STAT_GPIO_HIGH_LEVEL_THRESHOLD \n
 * @a #RES_INT_STAT_GPIO_OUT_OF_WINDOW \n (RES_INT_STAT_GPIO_LOW_LEVEL_THRESHOLD|RES_INT_STAT_GPIO_HIGH_LEVEL_THRESHOLD)
 * @a #RES_INT_STAT_GPIO_NEW_SAMPLE_READY \n
 *
 * @sa IntrStatus_t
 * @param pIntStatus Pointer to status variable to update
 * @return           0 on success
 */		
    int RangeGetInterruptStatus(uint8_t *pIntStatus)
    {
       return VL6180x_RangeGetInterruptStatus(Device, pIntStatus);
    }


/**
 * @brief Low level ranging and ALS register static settings (you should call @a VL6180x_Prepare() function instead)
 *
 * @return 0 on success
 */
    int StaticInit()
    {
       return VL6180x_StaticInit(Device);
    }

/**
 * @brief Wait for device to be ready (before a new ranging command can be issued by application)
 * @param MaxLoop    Max Number of i2c polling loop see @a #msec_2_i2cloop
 * @return           0 on success. <0 when fail \n
 *                   @ref VL6180x_ErrCode_t::TIME_OUT for time out \n
 *                   @ref VL6180x_ErrCode_t::INVALID_PARAMS if MaxLop<1
 */		
    int RangeWaitDeviceReady(int MaxLoop )
    {
       return VL6180x_RangeWaitDeviceReady(Device, MaxLoop);
    }

/**
 * @brief Program Inter measurement period (used only in continuous mode)
 *
 * @par Function Description
 * When trying to set too long time, it returns #INVALID_PARAMS
 *
 * @param InterMeasTime_msec Requires inter-measurement time in msec
 * @return 0 on success
 */		
    int RangeSetInterMeasPeriod(uint32_t  InterMeasTime_msec)
    {
       return VL6180x_RangeSetInterMeasPeriod(Device, InterMeasTime_msec);
    }

/**
 * @brief Set device ranging scaling factor
 *
 * @par Function Description
 * The ranging scaling factor is applied on the raw distance measured by the device to increase operating ranging at the price of the precision.
 * Changing the scaling factor when device is not in f/w standby state (free running) is not safe.
 * It can be source of spurious interrupt, wrongly scaled range etc ...
 * @warning __This  function doesns't update high/low threshold and other programmed settings linked to scaling factor__.
 *  To ensure proper operation, threshold and scaling changes should be done following this procedure: \n
 *  @li Set Group hold  : @a VL6180x_SetGroupParamHold() \n
 *  @li Get Threshold @a VL6180x_RangeGetThresholds() \n
 *  @li Change scaling : @a VL6180x_UpscaleSetScaling() \n
 *  @li Set Threshold : @a VL6180x_RangeSetThresholds() \n
 *  @li Unset Group Hold : @a VL6180x_SetGroupParamHold()
 *
 * @param scaling  Scaling factor to apply (1,2 or 3)
 * @return          0 on success when up-scale support is not configured it fail for any
 *                  scaling than the one statically configured.
 */
    int UpscaleSetScaling(uint8_t scaling)
    {
       return VL6180x_UpscaleSetScaling(Device, scaling);
    }

/**
 * @brief Get current ranging scaling factor
 *
 * @return    The current scaling factor
 */				
    int UpscaleGetScaling()
    {
       return VL6180x_UpscaleGetScaling(Device);
    }

/**
 * @brief Get the maximal distance for actual scaling
 * @par Function Description
 * Do not use prior to @a VL6180x_Prepare() or at least @a VL6180x_InitData()
 *
 * Any range value more than the value returned by this function is to be considered as "no target detected"
 * or "no target in detectable range" \n
 * @warning The maximal distance depends on the scaling
 *
 * @return    The maximal range limit for actual mode and scaling
 */		
    uint16_t GetUpperLimit()
    {
       return VL6180x_GetUpperLimit(Device);
    }

/**
 * @brief Apply low and high ranging thresholds that are considered only in continuous mode
 *
 * @par Function Description
 * This function programs low and high ranging thresholds that are considered in continuous mode : 
 * interrupt will be raised only when an object is detected at a distance inside this [low:high] range.  
 * The function takes care of applying current scaling factor if any.\n
 * To be safe, in continuous operation, thresholds must be changed under "group parameter hold" cover.
 * Group hold can be activated/deactivated directly in the function or externally (then set 0)
 * using /a VL6180x_SetGroupParamHold() function.
 *
 * @param low      Low threshold in mm
 * @param high     High threshold in mm
 * @param SafeHold  Use of group parameters hold to surround threshold programming.
 * @return  0 On success
 */		
    int RangeSetThresholds(uint16_t low, uint16_t high, int SafeHold)
    {
       return VL6180x_RangeSetThresholds(Device, low, high, SafeHold);
    }

/**
 * @brief  Get scaled high and low threshold from device
 *
 * @par Function Description
 * Due to scaling factor, the returned value may be different from what has been programmed first (precision lost).
 * For instance VL6180x_RangeSetThresholds(dev,11,22) with scale 3
 * will read back 9 ((11/3)x3) and 21 ((22/3)x3).
 *
 * @param low  scaled low Threshold ptr  can be NULL if not needed
 * @param high scaled High Threshold ptr can be NULL if not needed
 * @return 0 on success, return value is undefined if both low and high are NULL
 * @warning return value is undefined if both low and high are NULL
 */
    int RangeGetThresholds(uint16_t *low, uint16_t *high)
    {
       return VL6180x_RangeGetThresholds(Device, low, high);
    }

/**
 * @brief Set ranging raw thresholds (scaling not considered so not recommended to use it)
 *
 * @param low  raw low threshold set to raw register
 * @param high raw high threshold set to raw  register
 * @return 0 on success
 */			
    int RangeSetRawThresholds(uint8_t low, uint8_t high)
    {
       return VL6180x_RangeSetRawThresholds(Device, low, high);
    }

/**
 * @brief Set Early Convergence Estimate ratio
 * @par Function Description
 * For more information on ECE check datasheet
 * @warning May return a calibration warning in some use cases
 *
 * @param FactorM    ECE factor M in M/D
 * @param FactorD    ECE factor D in M/D
 * @return           0 on success. <0 on error. >0 on warning
 */		
    int RangeSetEceFactor(uint16_t  FactorM, uint16_t FactorD)
    {
       return VL6180x_RangeSetEceFactor(Device, FactorM, FactorD);
    }

/**
 * @brief Set Early Convergence Estimate state (See #SYSRANGE_RANGE_CHECK_ENABLES register)
 * @param enable    State to be set 0=disabled, otherwise enabled
 * @return          0 on success
 */		
    int RangeSetEceState(int enable)
    {
       return VL6180x_RangeSetEceState(Device, enable);
    }

/**
 * @brief Set activation state of the wrap around filter
 * @param state New activation state (0=off,  otherwise on)
 * @return      0 on success
 */			
    int FilterSetState(int state)
    {
       return VL6180x_FilterSetState(Device, state);
    }

/**
 * Get activation state of the wrap around filter
 * @return     Filter enabled or not, when filter is not supported it always returns 0S
 */			
    int FilterGetState()
    {
       return VL6180x_FilterGetState(Device);
    }

/**
 * @brief Set activation state of  DMax computation
 * @param state New activation state (0=off,  otherwise on)
 * @return      0 on success
 */		
    int DMaxSetState(int state)
    {
       return VL6180x_DMaxSetState(Device, state);
    }

/**
 * Get activation state of DMax computation
 * @return     Filter enabled or not, when filter is not supported it always returns 0S
 */		
    int DMaxGetState()
    {
       return VL6180x_DMaxGetState(Device);
    }

/**
 * @brief Set ranging mode and start/stop measure (use high level functions instead : @a VL6180x_RangeStartSingleShot() or @a VL6180x_RangeStartContinuousMode())
 *
 * @par Function Description
 * When used outside scope of known polling single shot stopped state, \n
 * user must ensure the device state is "idle" before to issue a new command.
 *
 * @param mode  A combination of working mode (#MODE_SINGLESHOT or #MODE_CONTINUOUS) and start/stop condition (#MODE_START_STOP) \n
 * @return      0 on success
 */		
    int RangeSetSystemMode(uint8_t mode)
    {
       return VL6180x_RangeSetSystemMode(Device, mode);
    }

/** @}  */ 

/** @defgroup api_ll_range_calibration Ranging calibration functions
 *  @brief    Ranging calibration functions
 *  @ingroup api_ll
 *  @{  
 */
/**
 * @brief Get part to part calibration offset
 *
 * @par Function Description
 * Should only be used after a successful call to @a VL6180x_InitData to backup device nvm value
 *
 * @return part to part calibration offset from device
 */		
    int8_t GetOffsetCalibrationData()
    {
       return VL6180x_GetOffsetCalibrationData(Device);
    }

/**
 * Set or over-write part to part calibration offset
 * \sa VL6180x_InitData(), VL6180x_GetOffsetCalibrationData()
 * @param offset   Offset
 */		
    void SetOffsetCalibrationData(int8_t offset)
    {
       return VL6180x_SetOffsetCalibrationData(Device, offset);
    }

/**
 * @brief Set Cross talk compensation rate
 *
 * @par Function Description
 * It programs register @a #SYSRANGE_CROSSTALK_COMPENSATION_RATE
 *
 * @param Rate Compensation rate (9.7 fix point) see datasheet for details
 * @return     0 on success
 */		
    int SetXTalkCompensationRate(FixPoint97_t Rate)
    {
       return VL6180x_SetXTalkCompensationRate(Device, Rate);
    }
/** @}  */

/** @defgroup api_ll_als ALS functions
 *  @brief    ALS functions
 *  @ingroup api_ll
 *  @{  
 */



/** @defgroup api_ll_misc Misc functions
 *  @brief    Misc functions
 *  @ingroup api_ll
 *  @{  
 */

/**
 * Set Group parameter Hold state
 *
 * @par Function Description
 * Group parameter holds @a #SYSTEM_GROUPED_PARAMETER_HOLD enable safe update (non atomic across multiple measure) by host
 * \n The critical register group is composed of: \n
 * #SYSTEM_INTERRUPT_CONFIG_GPIO \n
 * #SYSRANGE_THRESH_HIGH \n
 * #SYSRANGE_THRESH_LOW \n
 * #SYSALS_INTEGRATION_PERIOD \n
 * #SYSALS_ANALOGUE_GAIN \n
 * #SYSALS_THRESH_HIGH \n
 * #SYSALS_THRESH_LOW
 *
 *
 * @param Hold  Group parameter Hold state to be set (on/off)
 * @return      0 on success
 */
    int SetGroupParamHold(int Hold)
    {
       return VL6180x_SetGroupParamHold(Device, Hold);
    }		

/**
 * @brief Set new device i2c address
 *
 * After completion the device will answer to the new address programmed.
 *
 * @sa AN4478: Using multiple VL6180X's in a single design
 * @param NewAddr   The new i2c address (7bit)
 * @return          0 on success
 */		
    int SetI2CAddress(int NewAddr)
    {
       int status;
			
       status=VL6180x_SetI2CAddress(Device, NewAddr);
       if(!status)
          Device->I2cAddr=NewAddr;
       return status;
    }

/**
 * @brief Fully configure gpio 0/1 pin : polarity and functionality
 *
 * @param pin          gpio pin 0 or 1
 * @param IntFunction  Pin functionality : either #GPIOx_SELECT_OFF or #GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT (refer to #SYSTEM_MODE_GPIO1 register definition)
 * @param ActiveHigh   Set active high polarity, or active low see @a ::IntrPol_e
 * @return             0 on success
 */		
    int SetupGPIOx(int pin, uint8_t IntFunction, int ActiveHigh)
    {
       return VL6180x_SetupGPIOx(Device, pin, IntFunction, ActiveHigh);
    }

/**
 * @brief Set interrupt pin polarity for the given GPIO
 *
 * @param pin          Pin 0 or 1
 * @param active_high  select active high or low polarity using @ref IntrPol_e
 * @return             0 on success
 */		
    int SetGPIOxPolarity(int pin, int active_high)
    {
       return VL6180x_SetGPIOxPolarity(Device, pin, active_high);
    }

/**
 * Select interrupt functionality for the given GPIO
 *
 * @par Function Description
 * Functionality refer to @a SYSTEM_MODE_GPIO0
 *
 * @param pin            Pin to configure 0 or 1 (gpio0 or gpio1)\nNote that gpio0 is chip enable at power up !
 * @param functionality  Pin functionality : either #GPIOx_SELECT_OFF or #GPIOx_SELECT_GPIO_INTERRUPT_OUTPUT (refer to #SYSTEM_MODE_GPIO1 register definition)
 * @return              0 on success
 */		 
    int SetGPIOxFunctionality(int pin, uint8_t functionality)
    {
       return VL6180x_SetGPIOxFunctionality(Device, pin, functionality);
    }

/**
 * #brief Disable and turn to Hi-Z gpio output pin
 *
 * @param pin  The pin number to disable 0 or 1
 * @return     0 on success
 */	
    int DisableGPIOxOut(int pin)
    {
       return VL6180x_DisableGPIOxOut(Device, pin);
    }

/** @}  */

/** @defgroup api_ll_intr Interrupts management functions
 *  @brief    Interrupts management functions
 *  @ingroup api_ll
 *  @{  
 */

/**
 * @brief     Get all interrupts cause
 *
 * @param status Ptr to interrupt status. You can use @a IntrStatus_t::val
 * @return 0 on success
 */		
    int GetInterruptStatus(uint8_t *status)
    {
       return VL6180x_GetInterruptStatus(Device, status);
    }

/**
 * @brief Clear given system interrupt condition
 *
 * @par Function Description
 * Clear given interrupt cause by writing into register #SYSTEM_INTERRUPT_CLEAR register.
 * @param dev       The device 
 * @param IntClear  Which interrupt source to clear. Use any combinations of #INTERRUPT_CLEAR_RANGING , #INTERRUPT_CLEAR_ALS , #INTERRUPT_CLEAR_ERROR.
 * @return  0       On success
 */		
    int ClearInterrupt(uint8_t IntClear)
    {
       return VL6180x_ClearInterrupt(Device, IntClear );
    }

/**
 * @brief Clear error interrupt
 *
 * @param dev    The device
 * @return  0    On success
 */
 #define VL6180x_ClearErrorInterrupt(dev) VL6180x_ClearInterrupt(dev, INTERRUPT_CLEAR_ERROR)

/**
 * @brief Clear All interrupt causes (als+range+error)
 *
 * @param dev    The device
 * @return  0    On success
 */
#define VL6180x_ClearAllInterrupt(dev) VL6180x_ClearInterrupt(dev, INTERRUPT_CLEAR_ERROR|INTERRUPT_CLEAR_RANGING|INTERRUPT_CLEAR_ALS)
	
/** @}  */

/**
 * @brief Get the ALS (light in Lux) level
 *
 * @par Function Description
 * Get the ALS (light in Lux) level 
 * @param *piData The pointer to variable to write in the measure in Lux
 * @return  0       On success
 */				
    int GetLight(uint32_t *piData)
    {
       return VL6180x_AlsGetLux(Device, piData);
    }

/**
 * @brief Start the ALS (light) measure in continous mode
 *
 * @par Function Description
 * Start the ALS (light) measure in continous mode
 * @return  0       On success
 */						
    int AlsStartContinuousMode()
    {
       return VL6180x_AlsSetSystemMode(Device, MODE_START_STOP|MODE_CONTINUOUS);
    }

/**
 * @brief Start the ALS (light) measure in single shot mode
 *
 * @par Function Description
 * Start the ALS (light) measure in single shot mode
 * @return  0       On success
 */						    
    int AlsStartSingleShot()
    {
       return VL6180x_AlsSetSystemMode(Device, MODE_START_STOP|MODE_SINGLESHOT);
    }
		
 private:		
    /* api.h functions */
    int VL6180x_WaitDeviceBooted(VL6180xDev_t dev);
    int VL6180x_InitData(VL6180xDev_t dev );
    int VL6180x_SetupGPIO1(VL6180xDev_t dev, uint8_t IntFunction, int ActiveHigh);
    int VL6180x_Prepare(VL6180xDev_t dev);
    int VL6180x_RangeStartContinuousMode(VL6180xDev_t dev);
    int VL6180x_RangeStartSingleShot(VL6180xDev_t dev);
    int VL6180x_RangeSetMaxConvergenceTime(VL6180xDev_t dev, uint8_t  MaxConTime_msec);
    int VL6180x_RangePollMeasurement(VL6180xDev_t dev, VL6180x_RangeData_t *pRangeData);
    int VL6180x_RangeGetMeasurementIfReady(VL6180xDev_t dev, VL6180x_RangeData_t *pRangeData);
    int VL6180x_RangeGetMeasurement(VL6180xDev_t dev, VL6180x_RangeData_t *pRangeData);
    int VL6180x_RangeGetResult(VL6180xDev_t dev, int32_t *pRange_mm);
    int VL6180x_RangeConfigInterrupt(VL6180xDev_t dev, uint8_t ConfigGpioInt);
    int VL6180x_RangeGetInterruptStatus(VL6180xDev_t dev, uint8_t *pIntStatus);
    int VL6180x_AlsPollMeasurement(VL6180xDev_t dev, VL6180x_AlsData_t *pAlsData);
    int VL6180x_AlsGetMeasurement(VL6180xDev_t dev, VL6180x_AlsData_t *pAlsData);
    int VL6180x_AlsConfigInterrupt(VL6180xDev_t dev, uint8_t ConfigGpioInt);
    int VL6180x_AlsSetIntegrationPeriod(VL6180xDev_t dev, uint16_t period_ms);
    int VL6180x_AlsSetInterMeasurementPeriod(VL6180xDev_t dev,  uint16_t intermeasurement_period_ms);
    int VL6180x_AlsSetAnalogueGain(VL6180xDev_t dev, uint8_t gain);
    int VL6180x_AlsSetThresholds(VL6180xDev_t dev, uint16_t low, uint16_t high);
    int VL6180x_AlsGetInterruptStatus(VL6180xDev_t dev, uint8_t *pIntStatus);
    int VL6180x_StaticInit(VL6180xDev_t dev);
    int VL6180x_RangeWaitDeviceReady(VL6180xDev_t dev, int MaxLoop );
    int VL6180x_RangeSetInterMeasPeriod(VL6180xDev_t dev, uint32_t  InterMeasTime_msec);
    int VL6180x_UpscaleSetScaling(VL6180xDev_t dev, uint8_t scaling);
    int VL6180x_UpscaleGetScaling(VL6180xDev_t dev);
    uint16_t VL6180x_GetUpperLimit(VL6180xDev_t dev);
    int VL6180x_RangeSetThresholds(VL6180xDev_t dev, uint16_t low, uint16_t high, int SafeHold);
    int VL6180x_RangeGetThresholds(VL6180xDev_t dev, uint16_t *low, uint16_t *high);
    int VL6180x_RangeSetRawThresholds(VL6180xDev_t dev, uint8_t low, uint8_t high);
    int VL6180x_RangeSetEceFactor(VL6180xDev_t dev, uint16_t  FactorM, uint16_t FactorD);
    int VL6180x_RangeSetEceState(VL6180xDev_t dev, int enable );
    int VL6180x_FilterSetState(VL6180xDev_t dev, int state);
    int VL6180x_FilterGetState(VL6180xDev_t dev);
    int VL6180x_DMaxSetState(VL6180xDev_t dev, int state);
    int VL6180x_DMaxGetState(VL6180xDev_t dev);
    int VL6180x_RangeSetSystemMode(VL6180xDev_t dev, uint8_t mode);
    int8_t VL6180x_GetOffsetCalibrationData(VL6180xDev_t dev);
    void VL6180x_SetOffsetCalibrationData(VL6180xDev_t dev, int8_t offset);
    int VL6180x_SetXTalkCompensationRate(VL6180xDev_t dev, FixPoint97_t Rate);
    int VL6180x_AlsWaitDeviceReady(VL6180xDev_t dev, int MaxLoop );
    int VL6180x_AlsSetSystemMode(VL6180xDev_t dev, uint8_t mode); 
    int VL6180x_SetGroupParamHold(VL6180xDev_t dev, int Hold);
    int VL6180x_SetI2CAddress(VL6180xDev_t dev, uint8_t NewAddr);
    int VL6180x_SetupGPIOx(VL6180xDev_t dev, int pin, uint8_t IntFunction, int ActiveHigh);
    int VL6180x_SetGPIOxPolarity(VL6180xDev_t dev, int pin, int active_high);
    int VL6180x_SetGPIOxFunctionality(VL6180xDev_t dev, int pin, uint8_t functionality);
    int VL6180x_DisableGPIOxOut(VL6180xDev_t dev, int pin);
    int VL6180x_GetInterruptStatus(VL6180xDev_t dev, uint8_t *status);
    int VL6180x_ClearInterrupt(VL6180xDev_t dev, uint8_t IntClear );
		
    /*  Other functions defined in api.c */
    int VL6180x_RangeStaticInit(VL6180xDev_t dev); 
    int VL6180x_UpscaleRegInit(VL6180xDev_t dev);
    int VL6180x_UpscaleStaticInit(VL6180xDev_t dev); 
    int VL6180x_AlsGetLux(VL6180xDev_t dev, lux_t *pLux);
    int _UpscaleInitPatch0(VL6180xDev_t dev); 
    int VL6180x_RangeGetDeviceReady(VL6180xDev_t dev, int * Ready);
    int VL6180x_RangeSetEarlyConvergenceEestimateThreshold(VL6180xDev_t dev);
    int32_t _GetAveTotalTime(VL6180xDev_t dev); 
    int32_t _filter_Start(VL6180xDev_t dev, uint16_t m_trueRange_mm, uint16_t m_rawRange_mm, uint32_t m_rtnSignalRate, uint32_t m_rtnAmbientRate, uint16_t errorCode);
    int _filter_GetResult(VL6180xDev_t dev, VL6180x_RangeData_t *pRangeData);
    int _GetRateResult(VL6180xDev_t dev, VL6180x_RangeData_t *pRangeData); 
    int _DMax_InitData(VL6180xDev_t dev);
		
    /* Read function of the ID device */
    int ReadID(uint8_t *id);
    
    /* Write and read functions from I2C */
    int VL6180x_WrByte(VL6180xDev_t dev, uint16_t index, uint8_t data);
    int VL6180x_WrWord(VL6180xDev_t dev, uint16_t index, uint16_t data);
    int VL6180x_WrDWord(VL6180xDev_t dev, uint16_t index, uint32_t data);
    int VL6180x_RdByte(VL6180xDev_t dev, uint16_t index, uint8_t *data);
    int VL6180x_RdWord(VL6180xDev_t dev, uint16_t index, uint16_t *data);
    int VL6180x_RdDWord(VL6180xDev_t dev, uint16_t index, uint32_t *data);
    int VL6180x_UpdateByte(VL6180xDev_t dev, uint16_t index, uint8_t AndData, uint8_t OrData);
    int VL6180x_I2CWrite(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToWrite);
    int VL6180x_I2CRead(uint8_t DeviceAddr, uint16_t RegisterAddr, uint8_t *pBuffer, uint16_t NumByteToRead);
		
    int IsPresent();
    int StopRangeMeasurement(OperatingMode operating_mode);
    int StopAlsMeasurement(OperatingMode operating_mode);
    int GetRangeMeas(OperatingMode operating_mode, MeasureData_t *Data);	
    int GetAlsMeas(OperatingMode operating_mode, MeasureData_t *Data);
    int GetRangeAlsMeas(MeasureData_t *Data);
    int RangeSetLowThreshold(uint16_t threshold);
    int RangeSetHighThreshold(uint16_t threshold);
    int AlsSetLowThreshold(uint16_t threshold);	
    int AlsSetHighThreshold(uint16_t threshold);
    int GetRangeError(MeasureData_t *Data, VL6180x_RangeData_t RangeData);
    int GetAlsError(MeasureData_t *Data, VL6180x_AlsData_t AlsData);
    int RangeMeasPollSingleShot();
    int AlsMeasPollSingleShot();		
    int RangeMeasPollContinuousMode();	
    int AlsMeasPollContinuousMode();
    int AlsGetMeasurementIfReady(VL6180xDev_t dev, VL6180x_AlsData_t *pAlsData);
    int RangeMeasIntContinuousMode(void (*fptr)(void));
    int AlsMeasIntContinuousMode(void (*fptr)(void));
    int InterleavedMode(void (*fptr)(void));
    int StartInterleavedMode();
    int AlsGetThresholds(VL6180xDev_t dev, lux_t *low, lux_t *high);

		
    /* IO Device */
    I2C &_i2c;
    /* Digital out pin */
    DigitalInOut *gpio0;
    /* GPIO expander */
   // STMPE1600DigiOut *expgpio0;
    /* Measure detection IRQ */
    InterruptIn *gpio1Int;
    /* Device data */
    MyVL6180Dev_t MyDevice;
    VL6180xDev_t Device;  
};

#endif // __VL6180X_CLASS_H
