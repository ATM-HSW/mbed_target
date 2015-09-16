/******************************************************************//**
* @file ds3231.h
*
* @author Justin Jordan
*
* @version 1.0
*
* Started: 11NOV14
*
* Updated: 
*
* @brief Header file for DS3231 class
*
***********************************************************************
*
* @copyright 
* Copyright (C) 2015 Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
**********************************************************************/


#ifndef DS3231_H
#define DS3231_H


#include "mbed.h"


#define DS3231_I2C_ADRS 0x68 
#define I2C_WRITE 0
#define I2C_READ  1

#define AM_PM     (1 << 5) 
#define MODE      (1 << 6)
#define DY_DT     (1 << 6)
#define ALRM_MASK (1 << 7)

//control register bit masks
#define A1IE  (1 << 0)
#define A2IE  (1 << 1)
#define INTCN (1 << 2)
#define RS1   (1 << 3)
#define RS2   (1 << 4)
#define CONV  (1 << 5)
#define BBSQW (1 << 6)
#define EOSC  (1 << 7)

//status register bit masks
#define A1F     (1 << 0)
#define A2F     (1 << 1)
#define BSY     (1 << 2)
#define EN32KHZ (1 << 3)
#define OSF     (1 << 7)
  

/**
* ds3231_time_t - Struct for containing time data.
* 
* Members:
*
* - uint32_t seconds - Use decimal value. Member fx's convert to BCD
*
* - uint32_t minutes - Use decimal value. Member fx's convert to BCD
*
* - uint32_t hours   - Use decimal value. Member fx's convert to BCD
*
* - bool am_pm      - TRUE for PM, same logic as datasheet
*
* - bool mode       - TRUE for 12 hour, same logic as datasheet
*/
typedef struct
{
    uint32_t seconds; 
    uint32_t minutes; 
    uint32_t hours; 
    bool am_pm; 
    bool mode; 
}ds3231_time_t;


/**
* ds3231_calendar_t - Struct for containing calendar data.
* 
* Members:
*
* - uint32_t day   - Use decimal value. Member fx's convert to BCD
*
* - uint32_t date  - Use decimal value. Member fx's convert to BCD
*
* - uint32_t month - Use decimal value. Member fx's convert to BCD
*
* - uint32_t year  - Use decimal value. Member fx's convert to BCD
*/
typedef struct
{
    uint32_t day; 
    uint32_t date; 
    uint32_t month; 
    uint32_t year;
}ds3231_calendar_t;


/**
* ds3231_alrm_t - Struct for containing alarm data.
* 
* Members:
*
* - uint32_t seconds - Use decimal value. Member fx's convert to BCD 
*
* - uint32_t minutes - Use decimal value. Member fx's convert to BCD 
*
* - uint32_t hours   - Use decimal value. Member fx's convert to BCD 
*
* - uint32_t day     - Use decimal value. Member fx's convert to BCD 
*
* - uint32_t date    - Use decimal value. Member fx's convert to BCD 
*
* - bool am1        - Flag for setting alarm rate
*
* - bool am2        - Flag for setting alarm rate
*
* - bool am3        - Flag for setting alarm rate
*
* - bool am4        - Flag for setting alarm rate
*
* - bool am_pm      - TRUE for PM, same logic as datasheet
*
* - bool mode       - TRUE for 12 hour, same logic as datasheet
*
* - bool dy_dt      - TRUE for Day, same logic as datasheet
*/
typedef struct
{
    //Seconds and am1 not used for alarm2
    uint32_t seconds; 
    uint32_t minutes; 
    uint32_t hours; 
    uint32_t day; 
    uint32_t date; 
    bool am1; 
    bool am2;
    bool am3;
    bool am4;
    bool am_pm; 
    bool mode; 
    bool dy_dt;
}ds3231_alrm_t;


/**
* ds3231_cntl_stat_t - Struct for containing control and status 
* register data.
* 
* Members:
*
* - uint8_t control - Binary data for read/write of control register 
*
* - uint8_t status  - Binary data  for read/write of status register 
*/
typedef struct
{
    uint8_t control; 
    uint8_t status; 
}ds3231_cntl_stat_t;
        
                
/******************************************************************//**
* Ds3231 Class
**********************************************************************/
class Ds3231
{
    uint8_t w_adrs, r_adrs;
    I2C *_i2c;
    
    /**********************************************************//**
    * Private mmber fx, converts unsigned char to BCD
    *
    * On Entry:
    *     @param[in] data - 0-255
    *
    * On Exit:
    *     @return bcd_result = BCD representation of data
    *
    **************************************************************/
    uint16_t uchar_2_bcd(uint8_t data);
    
    
    /**********************************************************//**
    * Private mmber fx, converts BCD to a uint8_t
    *
    * On Entry:
    *     @param[in] bcd - 0-99
    *
    * On Exit:
    *     @return rtn_val = integer rep. of BCD
    *
    **************************************************************/
    uint8_t bcd_2_uchar(uint8_t bcd);
    
    public:
    
        /**
        * ds3231_regs_t - enumerated DS3231 registers 
        */
        typedef enum
        {
            SECONDS,
            MINUTES,
            HOURS,
            DAY,
            DATE,
            MONTH,
            YEAR,
            ALRM1_SECONDS,
            ALRM1_MINUTES,
            ALRM1_HOURS,
            ALRM1_DAY_DATE,
            ALRM2_MINUTES,
            ALRM2_HOURS,
            ALRM2_DAY_DATE,
            CONTROL,
            STATUS,
            AGING_OFFSET, //don't touch this register
            MSB_TEMP,
            LSB_TEMP
        }ds3231_regs_t;
    
        
        /**********************************************************//**
        * Constructor for Ds3231 Class
        *
        * On Entry:
        *     @param[in] sda - sda pin of I2C bus
        *     @param[in] scl - scl pin of I2C bus
        *
        * On Exit:
        *    @return none
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        *
        * @endcode
        **************************************************************/
        //Ds3231(PinName sda, PinName scl);
        Ds3231(I2C *i2c);
        
        
        /**********************************************************//**
        * Sets the time on DS3231
        * Struct data is in integrer format, not BCD.  Fx will convert
        * to BCD for you.
        *
        * On Entry:
        *     @param[in] time - struct cotaining time data; 
        *
        * On Exit:
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * //time = 12:00:00 AM 12hr mode
        * ds3231_time_t time = {12, 0, 0, 0, 1}
        * uint16_t rtn_val;
        *
        * rtn_val = rtc.set_time(time);
        *
        * @endcode
        **************************************************************/
        uint16_t set_time(ds3231_time_t time);
        
        
        /**********************************************************//**
        * Sets the calendar on DS3231
        *
        * On Entry:
        *     @param[in] calendar - struct cotaining calendar data 
        *
        * On Exit:
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * //see datasheet for calendar format
        * ds3231_calendar_t calendar = {1, 1, 1, 0}; 
        * uint16_t rtn_val;
        *
        * rtn_val = rtc.set_calendar(calendar);
        *
        * @endcode
        **************************************************************/
        uint16_t set_calendar(ds3231_calendar_t calendar);
        
        
        /**********************************************************//**
        * Set either Alarm1 or Alarm2 of DS3231
        *
        * On Entry:
        *     @param[in] alarm - struct cotaining alarm data 
        *                        
        *     @param[in] one_r_two - TRUE for Alarm1 and FALSE for 
        *                            Alarm2
        *
        * On Exit:
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * //see ds3231.h for .members and datasheet for alarm format
        * ds3231_alrm_t alarm; 
        * uint16_t rtn_val;
        *
        * rtn_val = rtc.set_alarm(alarm, FALSE);
        *
        * @endcode
        **************************************************************/
        uint16_t set_alarm(ds3231_alrm_t alarm, bool one_r_two);
        
        
        /**********************************************************//**
        * Set control and status registers of DS3231
        *
        * On Entry:
        *     @param[in] data - Struct containing control and status 
        *                       register data
        *
        * On Exit:
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15);  
        * 
        * //do not use 0xAA, see datasheet for appropriate data 
        * ds3231_cntl_stat_t data = {0xAA, 0xAA}; 
        *
        * rtn_val = rtc.set_cntl_stat_reg(data);
        *
        * @endcode
        **************************************************************/
        uint16_t set_cntl_stat_reg(ds3231_cntl_stat_t data);
        
        
        /**********************************************************//**
        * Gets the time on DS3231
        *
        * On Entry:
        *     @param[in] time - pointer to struct for storing time data
        *
        * On Exit:
        *     @param[out] time - contains current integrer rtc time 
        *                        data
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * //time = 12:00:00 AM 12hr mode
        * ds3231_time_t time = {12, 0, 0, 0, 1} 
        * uint16_t rtn_val;
        *
        * rtn_val = rtc.get_time(&time);
        *
        * @endcode
        **************************************************************/
        uint16_t get_time(ds3231_time_t* time);
        
        
        /**********************************************************//**
        * Gets the calendar on DS3231
        *
        * On Entry:
        *     @param[in] calendar - pointer to struct for storing 
        *                           calendar data
        *
        * On Exit:
        *     @param[out] calendar - contains current integer rtc 
        *                            calendar data
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * //see datasheet for calendar format
        * ds3231_calendar_t calendar = {1, 1, 1, 0}; 
        * uint16_t rtn_val;
        *
        * rtn_val = rtc.get_calendar(&calendar);
        *
        * @endcode
        **************************************************************/
        uint16_t get_calendar(ds3231_calendar_t* calendar);
        
        
        /**********************************************************//**
        * Get either Alarm1 or Alarm2 of DS3231
        *
        * On Entry:
        *     @param[in] alarm - pointer to struct for storing alarm 
        *                        data; 
        *
        *     @param[in] one_r_two - TRUE for Alarm1 and FALSE for 
        *                            Alarm2
        *
        * On Exit:
        *     @param[out] alarm - contains integer alarm data
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * //see ds3231.h for .members and datasheet for alarm format
        * ds3231_alrm_t alarm; 
        * uint16_t rtn_val;
        *
        * rtn_val = rtc.get_alarm(&alarm, FALSE);
        *
        * @endcode
        **************************************************************/
        uint16_t get_alarm(ds3231_alrm_t* alarm, bool one_r_two);
        
        
        /**********************************************************//**
        * Get control and status registers of DS3231
        *
        * On Entry:
        *     @param[in] data - pointer to struct for storing control 
        *                       and status register data
        *
        * On Exit:
        *     @param[out] data - contains control and status registers
        *                        data
        *     @return return value = 0 on success, non-0 on failure
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15);  
        * 
        * //do not use 0xAA, see datasheet for appropriate data 
        * ds3231_cntl_stat_t data = {0xAA, 0xAA}; 
        *
        * rtn_val = rtc.get_cntl_stat_reg(&data);
        *
        * @endcode
        **************************************************************/
        uint16_t get_cntl_stat_reg(ds3231_cntl_stat_t* data);
        
        
        /**********************************************************//**
        * Get temperature data of DS3231
        *
        * On Entry:
        *
        * On Exit:
        *     @return return value = raw temperature data
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * uint16_t temp; 
        *
        * temp = rtc.get_temperature();
        *
        * @endcode
        **************************************************************/
        uint16_t get_temperature(void);
        
        
        /**********************************************************//**
        * Get epoch time based on current RTC time and date.  
        * DS3231 must be configured and running before this fx is 
        * called
        *
        * On Entry:
        *
        * On Exit:
        *     @return return value = epoch time
        *
        * Example:
        * @code
        * 
        * //instantiate rtc object
        * Ds3231 rtc(D14, D15); 
        * 
        * time_t epoch_time; 
        *
        * epoch_time = rtc.get_epoch();
        *
        * @endcode
        **************************************************************/
        time_t get_epoch(void);
        
};
#endif /* DS3231_H*/
