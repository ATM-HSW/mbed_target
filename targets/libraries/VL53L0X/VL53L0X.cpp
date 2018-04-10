/**
 ******************************************************************************
 * @file    VL53L0X_class.cpp
 * @author  IMG
 * @version V0.0.1
 * @date    28-June-2016
 * @brief   Implementation file for the VL53L0X driver class
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

/* Includes */
#include <stdlib.h>

#include "VL53L0X.h"

//#include "VL53L0X_api_core.h"
//#include "VL53L0X_api_calibration.h"
//#include "VL53L0X_api_strings.h"
#include "VL53L0X_interrupt_threshold_settings.h"
#include "VL53L0X_tuning.h"
#include "VL53L0X_types.h"


/****************** define for i2c configuration *******************************/

/** Maximum buffer size to be used in i2c */
#define VL53L0X_MAX_I2C_XFER_SIZE   64 /* Maximum buffer size to be used in i2c */
#define VL53L0X_I2C_USER_VAR         /* none but could be for a flag var to get/pass to mutex interruptible  return flags and try again */


#define LOG_FUNCTION_START(fmt, ...) \
	_LOG_FUNCTION_START(TRACE_MODULE_API, fmt, ##__VA_ARGS__)
#define LOG_FUNCTION_END(status, ...) \
	_LOG_FUNCTION_END(TRACE_MODULE_API, status, ##__VA_ARGS__)
#define LOG_FUNCTION_END_FMT(status, fmt, ...) \
	_LOG_FUNCTION_END_FMT(TRACE_MODULE_API, status, fmt, ##__VA_ARGS__)

#ifdef VL53L0X_LOG_ENABLE
#define trace_print(level, ...) trace_print_module_function(TRACE_MODULE_API, \
	level, TRACE_FUNCTION_NONE, ##__VA_ARGS__)
#endif

#define REF_ARRAY_SPAD_0  0
#define REF_ARRAY_SPAD_5  5
#define REF_ARRAY_SPAD_10 10

uint32_t refArrayQuadrants[4] = {REF_ARRAY_SPAD_10, REF_ARRAY_SPAD_5,
                                 REF_ARRAY_SPAD_0, REF_ARRAY_SPAD_5
                                };




VL53L0X_Error VL53L0X::VL53L0X_device_read_strobe(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t strobe;
    uint32_t loop_nb;
    LOG_FUNCTION_START("");

    status |= VL53L0X_write_byte(dev, 0x83, 0x00);

    /* polling
     * use timeout to avoid deadlock*/
    if (status == VL53L0X_ERROR_NONE) {
        loop_nb = 0;
        do {
            status = VL53L0X_read_byte(dev, 0x83, &strobe);
            if ((strobe != 0x00) || status != VL53L0X_ERROR_NONE) {
                break;
            }

            loop_nb = loop_nb + 1;
        } while (loop_nb < VL53L0X_DEFAULT_MAX_LOOP);

        if (loop_nb >= VL53L0X_DEFAULT_MAX_LOOP) {
            status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    status |= VL53L0X_write_byte(dev, 0x83, 0x01);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_info_from_device(VL53L0X_DEV dev, uint8_t option)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t byte;
    uint32_t tmp_dword;
    uint8_t module_id;
    uint8_t revision;
    uint8_t reference_spad_count = 0;
    uint8_t reference_spad_type = 0;
    uint32_t part_uid_upper = 0;
    uint32_t part_uid_lower = 0;
    uint32_t offset_fixed1104_mm = 0;
    int16_t offset_micro_meters = 0;
    uint32_t dist_meas_tgt_fixed1104_mm = 400 << 4;
    uint32_t dist_meas_fixed1104_400_mm = 0;
    uint32_t signal_rate_meas_fixed1104_400_mm = 0;
    char product_id[19];
    char *product_id_tmp;
    uint8_t read_data_from_device_done;
    FixPoint1616_t signal_rate_meas_fixed400_mm_fix = 0;
    uint8_t nvm_ref_good_spad_map[VL53L0X_REF_SPAD_BUFFER_SIZE];
    int i;


    LOG_FUNCTION_START("");

    read_data_from_device_done = VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
                                 ReadDataFromDeviceDone);

    /* This access is done only once after that a GetDeviceInfo or
     * datainit is done*/
    if (read_data_from_device_done != 7) {

        status |= VL53L0X_write_byte(dev, 0x80, 0x01);
        status |= VL53L0X_write_byte(dev, 0xFF, 0x01);
        status |= VL53L0X_write_byte(dev, 0x00, 0x00);

        status |= VL53L0X_write_byte(dev, 0xFF, 0x06);
        status |= VL53L0X_read_byte(dev, 0x83, &byte);
        status |= VL53L0X_write_byte(dev, 0x83, byte | 4);
        status |= VL53L0X_write_byte(dev, 0xFF, 0x07);
        status |= VL53L0X_write_byte(dev, 0x81, 0x01);

        status |= VL53L0X_polling_delay(dev);

        status |= VL53L0X_write_byte(dev, 0x80, 0x01);

        if (((option & 1) == 1) &&
                ((read_data_from_device_done & 1) == 0)) {
            status |= VL53L0X_write_byte(dev, 0x94, 0x6b);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            reference_spad_count = (uint8_t)((tmp_dword >> 8) & 0x07f);
            reference_spad_type  = (uint8_t)((tmp_dword >> 15) & 0x01);

            status |= VL53L0X_write_byte(dev, 0x94, 0x24);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);


            nvm_ref_good_spad_map[0] = (uint8_t)((tmp_dword >> 24)
                                                 & 0xff);
            nvm_ref_good_spad_map[1] = (uint8_t)((tmp_dword >> 16)
                                                 & 0xff);
            nvm_ref_good_spad_map[2] = (uint8_t)((tmp_dword >> 8)
                                                 & 0xff);
            nvm_ref_good_spad_map[3] = (uint8_t)(tmp_dword & 0xff);

            status |= VL53L0X_write_byte(dev, 0x94, 0x25);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            nvm_ref_good_spad_map[4] = (uint8_t)((tmp_dword >> 24)
                                                 & 0xff);
            nvm_ref_good_spad_map[5] = (uint8_t)((tmp_dword >> 16)
                                                 & 0xff);
        }

        if (((option & 2) == 2) &&
                ((read_data_from_device_done & 2) == 0)) {

            status |= VL53L0X_write_byte(dev, 0x94, 0x02);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_byte(dev, 0x90, &module_id);

            status |= VL53L0X_write_byte(dev, 0x94, 0x7B);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_byte(dev, 0x90, &revision);

            status |= VL53L0X_write_byte(dev, 0x94, 0x77);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            product_id[0] = (char)((tmp_dword >> 25) & 0x07f);
            product_id[1] = (char)((tmp_dword >> 18) & 0x07f);
            product_id[2] = (char)((tmp_dword >> 11) & 0x07f);
            product_id[3] = (char)((tmp_dword >> 4) & 0x07f);

            byte = (uint8_t)((tmp_dword & 0x00f) << 3);

            status |= VL53L0X_write_byte(dev, 0x94, 0x78);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            product_id[4] = (char)(byte +
                                   ((tmp_dword >> 29) & 0x07f));
            product_id[5] = (char)((tmp_dword >> 22) & 0x07f);
            product_id[6] = (char)((tmp_dword >> 15) & 0x07f);
            product_id[7] = (char)((tmp_dword >> 8) & 0x07f);
            product_id[8] = (char)((tmp_dword >> 1) & 0x07f);

            byte = (uint8_t)((tmp_dword & 0x001) << 6);

            status |= VL53L0X_write_byte(dev, 0x94, 0x79);

            status |= VL53L0X_device_read_strobe(dev);

            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            product_id[9] = (char)(byte +
                                   ((tmp_dword >> 26) & 0x07f));
            product_id[10] = (char)((tmp_dword >> 19) & 0x07f);
            product_id[11] = (char)((tmp_dword >> 12) & 0x07f);
            product_id[12] = (char)((tmp_dword >> 5) & 0x07f);

            byte = (uint8_t)((tmp_dword & 0x01f) << 2);

            status |= VL53L0X_write_byte(dev, 0x94, 0x7A);

            status |= VL53L0X_device_read_strobe(dev);

            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            product_id[13] = (char)(byte +
                                    ((tmp_dword >> 30) & 0x07f));
            product_id[14] = (char)((tmp_dword >> 23) & 0x07f);
            product_id[15] = (char)((tmp_dword >> 16) & 0x07f);
            product_id[16] = (char)((tmp_dword >> 9) & 0x07f);
            product_id[17] = (char)((tmp_dword >> 2) & 0x07f);
            product_id[18] = '\0';

        }

        if (((option & 4) == 4) &&
                ((read_data_from_device_done & 4) == 0)) {

            status |= VL53L0X_write_byte(dev, 0x94, 0x7B);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &part_uid_upper);

            status |= VL53L0X_write_byte(dev, 0x94, 0x7C);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &part_uid_lower);

            status |= VL53L0X_write_byte(dev, 0x94, 0x73);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            signal_rate_meas_fixed1104_400_mm = (tmp_dword &
                                                 0x0000000ff) << 8;

            status |= VL53L0X_write_byte(dev, 0x94, 0x74);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            signal_rate_meas_fixed1104_400_mm |= ((tmp_dword &
                                                   0xff000000) >> 24);

            status |= VL53L0X_write_byte(dev, 0x94, 0x75);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            dist_meas_fixed1104_400_mm = (tmp_dword & 0x0000000ff)
                                         << 8;

            status |= VL53L0X_write_byte(dev, 0x94, 0x76);
            status |= VL53L0X_device_read_strobe(dev);
            status |= VL53L0X_read_dword(dev, 0x90, &tmp_dword);

            dist_meas_fixed1104_400_mm |= ((tmp_dword & 0xff000000)
                                           >> 24);
        }

        status |= VL53L0X_write_byte(dev, 0x81, 0x00);
        status |= VL53L0X_write_byte(dev, 0xFF, 0x06);
        status |= VL53L0X_read_byte(dev, 0x83, &byte);
        status |= VL53L0X_write_byte(dev, 0x83, byte & 0xfb);
        status |= VL53L0X_write_byte(dev, 0xFF, 0x01);
        status |= VL53L0X_write_byte(dev, 0x00, 0x01);

        status |= VL53L0X_write_byte(dev, 0xFF, 0x00);
        status |= VL53L0X_write_byte(dev, 0x80, 0x00);
    }

    if ((status == VL53L0X_ERROR_NONE) &&
            (read_data_from_device_done != 7)) {
        /* Assign to variable if status is ok */
        if (((option & 1) == 1) &&
                ((read_data_from_device_done & 1) == 0)) {
            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               ReferenceSpadCount, reference_spad_count);

            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               ReferenceSpadType, reference_spad_type);

            for (i = 0; i < VL53L0X_REF_SPAD_BUFFER_SIZE; i++) {
                dev->Data.SpadData.RefGoodSpadMap[i] =
                    nvm_ref_good_spad_map[i];
            }
        }

        if (((option & 2) == 2) &&
                ((read_data_from_device_done & 2) == 0)) {
            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               ModuleId, module_id);

            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               Revision, revision);

            product_id_tmp = VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
                             ProductId);
            VL53L0X_COPYSTRING(product_id_tmp, product_id);

        }

        if (((option & 4) == 4) &&
                ((read_data_from_device_done & 4) == 0)) {
            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               PartUIDUpper, part_uid_upper);

            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               PartUIDLower, part_uid_lower);

            signal_rate_meas_fixed400_mm_fix =
                VL53L0X_FIXPOINT97TOFIXPOINT1616(
                    signal_rate_meas_fixed1104_400_mm);

            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               SignalRateMeasFixed400mm,
                                               signal_rate_meas_fixed400_mm_fix);

            offset_micro_meters = 0;
            if (dist_meas_fixed1104_400_mm != 0) {
                offset_fixed1104_mm =
                    dist_meas_fixed1104_400_mm -
                    dist_meas_tgt_fixed1104_mm;
                offset_micro_meters = (offset_fixed1104_mm
                                       * 1000) >> 4;
                offset_micro_meters *= -1;
            }

            PALDevDataSet(dev,
                          Part2PartOffsetAdjustmentNVMMicroMeter,
                          offset_micro_meters);
        }
        byte = (uint8_t)(read_data_from_device_done | option);
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev, ReadDataFromDeviceDone,
                                           byte);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
        int32_t *p_offset_calibration_data_micro_meter)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint16_t range_offset_register;
    int16_t c_max_offset = 2047;
    int16_t c_offset_range = 4096;

    /* Note that offset has 10.2 format */

    status = VL53L0X_read_word(dev,
                               VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                               &range_offset_register);

    if (status == VL53L0X_ERROR_NONE) {
        range_offset_register = (range_offset_register & 0x0fff);

        /* Apply 12 bit 2's compliment conversion */
        if (range_offset_register > c_max_offset) {
            *p_offset_calibration_data_micro_meter =
                (int16_t)(range_offset_register - c_offset_range)
                * 250;
        } else {
            *p_offset_calibration_data_micro_meter =
                (int16_t)range_offset_register * 250;
        }

    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
        int32_t *p_offset_calibration_data_micro_meter)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_get_offset_calibration_data_micro_meter(dev,
             p_offset_calibration_data_micro_meter);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
        int32_t offset_calibration_data_micro_meter)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t c_max_offset_micro_meter = 511000;
    int32_t c_min_offset_micro_meter = -512000;
    int16_t c_offset_range = 4096;
    uint32_t encoded_offset_val;

    LOG_FUNCTION_START("");

    if (offset_calibration_data_micro_meter > c_max_offset_micro_meter) {
        offset_calibration_data_micro_meter = c_max_offset_micro_meter;
    } else {
        if (offset_calibration_data_micro_meter < c_min_offset_micro_meter) {
            offset_calibration_data_micro_meter = c_min_offset_micro_meter;
        }
    }

    /* The offset register is 10.2 format and units are mm
     * therefore conversion is applied by a division of
     * 250.
     */
    if (offset_calibration_data_micro_meter >= 0) {
        encoded_offset_val =
            offset_calibration_data_micro_meter / 250;
    } else {
        encoded_offset_val =
            c_offset_range +
            offset_calibration_data_micro_meter / 250;
    }

    status = VL53L0X_write_word(dev,
                                VL53L0X_REG_ALGO_PART_TO_PART_RANGE_OFFSET_MM,
                                encoded_offset_val);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_offset_calibration_data_micro_meter(VL53L0X_DEV dev,
        int32_t offset_calibration_data_micro_meter)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_set_offset_calibration_data_micro_meter(dev,
             offset_calibration_data_micro_meter);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_apply_offset_adjustment(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int32_t corrected_offset_micro_meters;
    int32_t current_offset_micro_meters;

    /* if we run on this function we can read all the NVM info
     * used by the API */
    status = VL53L0X_get_info_from_device(dev, 7);

    /* Read back current device offset */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_offset_calibration_data_micro_meter(dev,
                 &current_offset_micro_meters);
    }

    /* Apply Offset Adjustment derived from 400mm measurements */
    if (status == VL53L0X_ERROR_NONE) {

        /* Store initial device offset */
        PALDevDataSet(dev, Part2PartOffsetNVMMicroMeter,
                      current_offset_micro_meters);

        corrected_offset_micro_meters = current_offset_micro_meters +
                                        (int32_t)PALDevDataGet(dev,
                                                Part2PartOffsetAdjustmentNVMMicroMeter);

        status = VL53L0X_set_offset_calibration_data_micro_meter(dev,
                 corrected_offset_micro_meters);

        /* store current, adjusted offset */
        if (status == VL53L0X_ERROR_NONE) {
            VL53L0X_SETPARAMETERFIELD(dev, RangeOffsetMicroMeters,
                                      corrected_offset_micro_meters);
        }
    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_device_mode(VL53L0X_DEV dev,
        VL53L0X_DeviceModes *p_device_mode)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(dev, DeviceMode, *p_device_mode);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_inter_measurement_period_milli_seconds(VL53L0X_DEV dev,
        uint32_t *p_inter_measurement_period_milli_seconds)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint16_t osc_calibrate_val;
    uint32_t im_period_milli_seconds;

    LOG_FUNCTION_START("");

    status = VL53L0X_read_word(dev, VL53L0X_REG_OSC_CALIBRATE_VAL,
                               &osc_calibrate_val);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_dword(dev,
                                    VL53L0X_REG_SYSTEM_INTERMEASUREMENT_PERIOD,
                                    &im_period_milli_seconds);
    }

    if (status == VL53L0X_ERROR_NONE) {
        if (osc_calibrate_val != 0) {
            *p_inter_measurement_period_milli_seconds =
                im_period_milli_seconds / osc_calibrate_val;
        }
        VL53L0X_SETPARAMETERFIELD(dev,
                                  InterMeasurementPeriodMilliSeconds,
                                  *p_inter_measurement_period_milli_seconds);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_x_talk_compensation_rate_mega_cps(VL53L0X_DEV dev,
        FixPoint1616_t *p_xtalk_compensation_rate_mega_cps)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint16_t value;
    FixPoint1616_t temp_fix1616;

    LOG_FUNCTION_START("");

    status = VL53L0X_read_word(dev,
                               VL53L0X_REG_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, (uint16_t *)&value);
    if (status == VL53L0X_ERROR_NONE) {
        if (value == 0) {
            /* the Xtalk is disabled return value from memory */
            VL53L0X_GETPARAMETERFIELD(dev,
                                      XTalkCompensationRateMegaCps, temp_fix1616);
            *p_xtalk_compensation_rate_mega_cps = temp_fix1616;
            VL53L0X_SETPARAMETERFIELD(dev, XTalkCompensationEnable,
                                      0);
        } else {
            temp_fix1616 = VL53L0X_FIXPOINT313TOFIXPOINT1616(value);
            *p_xtalk_compensation_rate_mega_cps = temp_fix1616;
            VL53L0X_SETPARAMETERFIELD(dev,
                                      XTalkCompensationRateMegaCps, temp_fix1616);
            VL53L0X_SETPARAMETERFIELD(dev, XTalkCompensationEnable,
                                      1);
        }
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_limit_check_value(VL53L0X_DEV dev, uint16_t limit_check_id,
        FixPoint1616_t *p_limit_check_value)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t enable_zero_value = 0;
    uint16_t temp16;
    FixPoint1616_t temp_fix1616;

    LOG_FUNCTION_START("");

    switch (limit_check_id) {

        case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
            /* internal computation: */
            VL53L0X_GETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                           VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, temp_fix1616);
            enable_zero_value = 0;
            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:
            status = VL53L0X_read_word(dev,
                                       VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                                       &temp16);
            if (status == VL53L0X_ERROR_NONE) {
                temp_fix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(temp16);
            }


            enable_zero_value = 1;
            break;

        case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:
            /* internal computation: */
            VL53L0X_GETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                           VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, temp_fix1616);
            enable_zero_value = 0;
            break;

        case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:
            /* internal computation: */
            VL53L0X_GETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                           VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, temp_fix1616);
            enable_zero_value = 0;
            break;

        case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
        case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:
            status = VL53L0X_read_word(dev,
                                       VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT,
                                       &temp16);
            if (status == VL53L0X_ERROR_NONE) {
                temp_fix1616 = VL53L0X_FIXPOINT97TOFIXPOINT1616(temp16);
            }


            enable_zero_value = 0;
            break;

        default:
            status = VL53L0X_ERROR_INVALID_PARAMS;

    }

    if (status == VL53L0X_ERROR_NONE) {

        if (enable_zero_value == 1) {

            if (temp_fix1616 == 0) {
                /* disabled: return value from memory */
                VL53L0X_GETARRAYPARAMETERFIELD(dev,
                                               LimitChecksValue, limit_check_id,
                                               temp_fix1616);
                *p_limit_check_value = temp_fix1616;
                VL53L0X_SETARRAYPARAMETERFIELD(dev,
                                               LimitChecksEnable, limit_check_id, 0);
            } else {
                *p_limit_check_value = temp_fix1616;
                VL53L0X_SETARRAYPARAMETERFIELD(dev,
                                               LimitChecksValue, limit_check_id,
                                               temp_fix1616);
                VL53L0X_SETARRAYPARAMETERFIELD(dev,
                                               LimitChecksEnable, limit_check_id, 1);
            }
        } else {
            *p_limit_check_value = temp_fix1616;
        }
    }

    LOG_FUNCTION_END(status);
    return status;

}

VL53L0X_Error VL53L0X::VL53L0X_get_limit_check_enable(VL53L0X_DEV dev, uint16_t limit_check_id,
        uint8_t *p_limit_check_enable)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t temp8;

    LOG_FUNCTION_START("");

    if (limit_check_id >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
        status = VL53L0X_ERROR_INVALID_PARAMS;
        *p_limit_check_enable = 0;
    } else {
        VL53L0X_GETARRAYPARAMETERFIELD(dev, LimitChecksEnable,
                                       limit_check_id, temp8);
        *p_limit_check_enable = temp8;
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_wrap_around_check_enable(VL53L0X_DEV dev,
        uint8_t *p_wrap_around_check_enable)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t data;

    LOG_FUNCTION_START("");

    status = VL53L0X_read_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &data);
    if (status == VL53L0X_ERROR_NONE) {
        PALDevDataSet(dev, SequenceConfig, data);
        if (data & (0x01 << 7)) {
            *p_wrap_around_check_enable = 0x01;
        } else {
            *p_wrap_around_check_enable = 0x00;
        }
    }
    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(dev, WrapAroundCheckEnable,
                                  *p_wrap_around_check_enable);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::sequence_step_enabled(VL53L0X_DEV dev,
        VL53L0X_SequenceStepId sequence_step_id, uint8_t sequence_config,
        uint8_t *p_sequence_step_enabled)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    *p_sequence_step_enabled = 0;
    LOG_FUNCTION_START("");

    switch (sequence_step_id) {
        case VL53L0X_SEQUENCESTEP_TCC:
            *p_sequence_step_enabled = (sequence_config & 0x10) >> 4;
            break;
        case VL53L0X_SEQUENCESTEP_DSS:
            *p_sequence_step_enabled = (sequence_config & 0x08) >> 3;
            break;
        case VL53L0X_SEQUENCESTEP_MSRC:
            *p_sequence_step_enabled = (sequence_config & 0x04) >> 2;
            break;
        case VL53L0X_SEQUENCESTEP_PRE_RANGE:
            *p_sequence_step_enabled = (sequence_config & 0x40) >> 6;
            break;
        case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
            *p_sequence_step_enabled = (sequence_config & 0x80) >> 7;
            break;
        default:
            Status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    LOG_FUNCTION_END(status);
    return Status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_sequence_step_enables(VL53L0X_DEV dev,
        VL53L0X_SchedulerSequenceSteps_t *p_scheduler_sequence_steps)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t sequence_config = 0;
    LOG_FUNCTION_START("");

    status = VL53L0X_read_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                               &sequence_config);

    if (status == VL53L0X_ERROR_NONE) {
        status = sequence_step_enabled(dev,
                                       VL53L0X_SEQUENCESTEP_TCC, sequence_config,
                                       &p_scheduler_sequence_steps->TccOn);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = sequence_step_enabled(dev,
                                       VL53L0X_SEQUENCESTEP_DSS, sequence_config,
                                       &p_scheduler_sequence_steps->DssOn);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = sequence_step_enabled(dev,
                                       VL53L0X_SEQUENCESTEP_MSRC, sequence_config,
                                       &p_scheduler_sequence_steps->MsrcOn);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = sequence_step_enabled(dev,
                                       VL53L0X_SEQUENCESTEP_PRE_RANGE, sequence_config,
                                       &p_scheduler_sequence_steps->PreRangeOn);
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = sequence_step_enabled(dev,
                                       VL53L0X_SEQUENCESTEP_FINAL_RANGE, sequence_config,
                                       &p_scheduler_sequence_steps->FinalRangeOn);
    }

    LOG_FUNCTION_END(status);
    return status;
}

uint8_t VL53L0X::VL53L0X_decode_vcsel_period(uint8_t vcsel_period_reg)
{
    /*!
     * Converts the encoded VCSEL period register value into the real
     * period in PLL clocks
     */

    uint8_t vcsel_period_pclks = 0;

    vcsel_period_pclks = (vcsel_period_reg + 1) << 1;

    return vcsel_period_pclks;
}

uint8_t VL53L0X::lv53l0x_encode_vcsel_period(uint8_t vcsel_period_pclks)
{
    /*!
     * Converts the encoded VCSEL period register value into the real period
     * in PLL clocks
     */

    uint8_t vcsel_period_reg = 0;

    vcsel_period_reg = (vcsel_period_pclks >> 1) - 1;

    return vcsel_period_reg;
}


VL53L0X_Error VL53L0X::wrapped_VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV dev,
        VL53L0X_VcselPeriod vcsel_period_type, uint8_t vcsel_pulse_period_pclk)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t vcsel_period_reg;
    uint8_t min_pre_vcsel_period_pclk = 12;
    uint8_t max_pre_vcsel_period_pclk = 18;
    uint8_t min_final_vcsel_period_pclk = 8;
    uint8_t max_final_vcsel_period_pclk = 14;
    uint32_t measurement_timing_budget_micro_seconds;
    uint32_t final_range_timeout_micro_seconds;
    uint32_t pre_range_timeout_micro_seconds;
    uint32_t msrc_timeout_micro_seconds;
    uint8_t phase_cal_int = 0;

    /* Check if valid clock period requested */

    if ((vcsel_pulse_period_pclk % 2) != 0) {
        /* Value must be an even number */
        status = VL53L0X_ERROR_INVALID_PARAMS;
    } else if (vcsel_period_type == VL53L0X_VCSEL_PERIOD_PRE_RANGE &&
               (vcsel_pulse_period_pclk < min_pre_vcsel_period_pclk ||
                vcsel_pulse_period_pclk > max_pre_vcsel_period_pclk)) {
        status = VL53L0X_ERROR_INVALID_PARAMS;
    } else if (vcsel_period_type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE &&
               (vcsel_pulse_period_pclk < min_final_vcsel_period_pclk ||
                vcsel_pulse_period_pclk > max_final_vcsel_period_pclk)) {

        status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    /* Apply specific settings for the requested clock period */

    if (status != VL53L0X_ERROR_NONE) {
        return status;
    }


    if (vcsel_period_type == VL53L0X_VCSEL_PERIOD_PRE_RANGE) {

        /* Set phase check limits */
        if (vcsel_pulse_period_pclk == 12) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x18);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);
        } else if (vcsel_pulse_period_pclk == 14) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x30);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);
        } else if (vcsel_pulse_period_pclk == 16) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x40);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);
        } else if (vcsel_pulse_period_pclk == 18) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x50);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_PRE_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);
        }
    } else if (vcsel_period_type == VL53L0X_VCSEL_PERIOD_FINAL_RANGE) {

        if (vcsel_pulse_period_pclk == 8) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x10);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);

            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);

            status |= VL53L0X_write_byte(dev, 0xff, 0x01);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_LIM,
                                         0x30);
            status |= VL53L0X_write_byte(dev, 0xff, 0x00);
        } else if (vcsel_pulse_period_pclk == 10) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x28);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);

            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);

            status |= VL53L0X_write_byte(dev, 0xff, 0x01);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_LIM,
                                         0x20);
            status |= VL53L0X_write_byte(dev, 0xff, 0x00);
        } else if (vcsel_pulse_period_pclk == 12) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x38);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);

            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);

            status |= VL53L0X_write_byte(dev, 0xff, 0x01);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_LIM,
                                         0x20);
            status |= VL53L0X_write_byte(dev, 0xff, 0x00);
        } else if (vcsel_pulse_period_pclk == 14) {

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH,
                                        0x048);
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_FINAL_RANGE_CONFIG_VALID_PHASE_LOW,
                                        0x08);

            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);

            status |= VL53L0X_write_byte(dev, 0xff, 0x01);
            status |= VL53L0X_write_byte(dev,
                                         VL53L0X_REG_ALGO_PHASECAL_LIM,
                                         0x20);
            status |= VL53L0X_write_byte(dev, 0xff, 0x00);
        }
    }


    /* Re-calculate and apply timeouts, in macro periods */

    if (status == VL53L0X_ERROR_NONE) {
        vcsel_period_reg = lv53l0x_encode_vcsel_period((uint8_t)
                           vcsel_pulse_period_pclk);

        /* When the VCSEL period for the pre or final range is changed,
        * the corresponding timeout must be read from the device using
        * the current VCSEL period, then the new VCSEL period can be
        * applied. The timeout then must be written back to the device
        * using the new VCSEL period.
        *
        * For the MSRC timeout, the same applies - this timeout being
        * dependant on the pre-range vcsel period.
        */
        switch (vcsel_period_type) {
            case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
                status = get_sequence_step_timeout(dev,
                                                   VL53L0X_SEQUENCESTEP_PRE_RANGE,
                                                   &pre_range_timeout_micro_seconds);

                if (status == VL53L0X_ERROR_NONE)
                    status = get_sequence_step_timeout(dev,
                                                       VL53L0X_SEQUENCESTEP_MSRC,
                                                       &msrc_timeout_micro_seconds);

                if (status == VL53L0X_ERROR_NONE)
                    status = VL53L0X_write_byte(dev,
                                                VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
                                                vcsel_period_reg);


                if (status == VL53L0X_ERROR_NONE)
                    status = set_sequence_step_timeout(dev,
                                                       VL53L0X_SEQUENCESTEP_PRE_RANGE,
                                                       pre_range_timeout_micro_seconds);


                if (status == VL53L0X_ERROR_NONE)
                    status = set_sequence_step_timeout(dev,
                                                       VL53L0X_SEQUENCESTEP_MSRC,
                                                       msrc_timeout_micro_seconds);

                VL53L0X_SETDEVICESPECIFICPARAMETER(
                    dev,
                    PreRangeVcselPulsePeriod,
                    vcsel_pulse_period_pclk);
                break;
            case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
                status = get_sequence_step_timeout(dev,
                                                   VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                                                   &final_range_timeout_micro_seconds);

                if (status == VL53L0X_ERROR_NONE)
                    status = VL53L0X_write_byte(dev,
                                                VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                                                vcsel_period_reg);


                if (status == VL53L0X_ERROR_NONE)
                    status = set_sequence_step_timeout(dev,
                                                       VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                                                       final_range_timeout_micro_seconds);

                VL53L0X_SETDEVICESPECIFICPARAMETER(
                    dev,
                    FinalRangeVcselPulsePeriod,
                    vcsel_pulse_period_pclk);
                break;
            default:
                status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }

    /* Finally, the timing budget must be re-applied */
    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_GETPARAMETERFIELD(dev,
                                  MeasurementTimingBudgetMicroSeconds,
                                  measurement_timing_budget_micro_seconds);

        status = VL53L0X_set_measurement_timing_budget_micro_seconds(dev,
                 measurement_timing_budget_micro_seconds);
    }

    /* Perform the phase calibration. This is needed after changing on
     * vcsel period.
     * get_data_enable = 0, restore_config = 1 */
    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_perform_phase_calibration(
                     dev, &phase_cal_int, 0, 1);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_vcsel_pulse_period(VL53L0X_DEV dev,
        VL53L0X_VcselPeriod vcsel_period_type, uint8_t vcsel_pulse_period)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_set_vcsel_pulse_period(dev, vcsel_period_type,
             vcsel_pulse_period);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV dev,
        VL53L0X_VcselPeriod vcsel_period_type, uint8_t *p_vcsel_pulse_period_pclk)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t vcsel_period_reg;

    switch (vcsel_period_type) {
        case VL53L0X_VCSEL_PERIOD_PRE_RANGE:
            status = VL53L0X_read_byte(dev,
                                       VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD,
                                       &vcsel_period_reg);
            break;
        case VL53L0X_VCSEL_PERIOD_FINAL_RANGE:
            status = VL53L0X_read_byte(dev,
                                       VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD,
                                       &vcsel_period_reg);
            break;
        default:
            status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    if (status == VL53L0X_ERROR_NONE)
        *p_vcsel_pulse_period_pclk =
            VL53L0X_decode_vcsel_period(vcsel_period_reg);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_vcsel_pulse_period(VL53L0X_DEV dev,
        VL53L0X_VcselPeriod vcsel_period_type, uint8_t *p_vcsel_pulse_period_pclk)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_get_vcsel_pulse_period(dev, vcsel_period_type,
             p_vcsel_pulse_period_pclk);

    LOG_FUNCTION_END(status);
    return status;
}

uint32_t VL53L0X::VL53L0X_decode_timeout(uint16_t encoded_timeout)
{
    /*!
     * Decode 16-bit timeout register value - format (LSByte * 2^MSByte) + 1
     */

    uint32_t timeout_macro_clks = 0;

    timeout_macro_clks = ((uint32_t)(encoded_timeout & 0x00FF)
                          << (uint32_t)((encoded_timeout & 0xFF00) >> 8)) + 1;

    return timeout_macro_clks;
}

uint32_t VL53L0X::VL53L0X_calc_macro_period_ps(VL53L0X_DEV dev, uint8_t vcsel_period_pclks)
{
    uint64_t pll_period_ps;
    uint32_t macro_period_vclks;
    uint32_t macro_period_ps;

    LOG_FUNCTION_START("");

    /* The above calculation will produce rounding errors,
       therefore set fixed value
    */
    pll_period_ps = 1655;

    macro_period_vclks = 2304;
    macro_period_ps = (uint32_t)(macro_period_vclks
                                 * vcsel_period_pclks * pll_period_ps);

    LOG_FUNCTION_END("");
    return macro_period_ps;
}

/* To convert register value into us */
uint32_t VL53L0X::VL53L0X_calc_timeout_us(VL53L0X_DEV dev,
        uint16_t timeout_period_mclks,
        uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t actual_timeout_period_us = 0;

    macro_period_ps = VL53L0X_calc_macro_period_ps(dev, vcsel_period_pclks);
    macro_period_ns = (macro_period_ps + 500) / 1000;

    actual_timeout_period_us =
        ((timeout_period_mclks * macro_period_ns) + 500) / 1000;

    return actual_timeout_period_us;
}

VL53L0X_Error VL53L0X::get_sequence_step_timeout(VL53L0X_DEV dev,
        VL53L0X_SequenceStepId sequence_step_id,
        uint32_t *p_time_out_micro_secs)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t current_vcsel_pulse_period_p_clk;
    uint8_t encoded_time_out_byte = 0;
    uint32_t timeout_micro_seconds = 0;
    uint16_t pre_range_encoded_time_out = 0;
    uint16_t msrc_time_out_m_clks;
    uint16_t pre_range_time_out_m_clks;
    uint16_t final_range_time_out_m_clks = 0;
    uint16_t final_range_encoded_time_out;
    VL53L0X_SchedulerSequenceSteps_t scheduler_sequence_steps;

    if ((sequence_step_id == VL53L0X_SEQUENCESTEP_TCC)	 ||
            (sequence_step_id == VL53L0X_SEQUENCESTEP_DSS)	 ||
            (sequence_step_id == VL53L0X_SEQUENCESTEP_MSRC)) {

        status = VL53L0X_get_vcsel_pulse_period(dev,
                                                VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                &current_vcsel_pulse_period_p_clk);
        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_read_byte(dev,
                                       VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                                       &encoded_time_out_byte);
        }
        msrc_time_out_m_clks = VL53L0X_decode_timeout(encoded_time_out_byte);

        timeout_micro_seconds = VL53L0X_calc_timeout_us(dev,
                                msrc_time_out_m_clks,
                                current_vcsel_pulse_period_p_clk);
    } else if (sequence_step_id == VL53L0X_SEQUENCESTEP_PRE_RANGE) {
        /* Retrieve PRE-RANGE VCSEL Period */
        status = VL53L0X_get_vcsel_pulse_period(dev,
                                                VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                &current_vcsel_pulse_period_p_clk);

        /* Retrieve PRE-RANGE Timeout in Macro periods (MCLKS) */
        if (status == VL53L0X_ERROR_NONE) {

            /* Retrieve PRE-RANGE VCSEL Period */
            status = VL53L0X_get_vcsel_pulse_period(dev,
                                                    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                    &current_vcsel_pulse_period_p_clk);

            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_read_word(dev,
                                           VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                           &pre_range_encoded_time_out);
            }

            pre_range_time_out_m_clks = VL53L0X_decode_timeout(
                                            pre_range_encoded_time_out);

            timeout_micro_seconds = VL53L0X_calc_timeout_us(dev,
                                    pre_range_time_out_m_clks,
                                    current_vcsel_pulse_period_p_clk);
        }
    } else if (sequence_step_id == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

        VL53L0X_get_sequence_step_enables(dev, &scheduler_sequence_steps);
        pre_range_time_out_m_clks = 0;

        if (scheduler_sequence_steps.PreRangeOn) {
            /* Retrieve PRE-RANGE VCSEL Period */
            status = VL53L0X_get_vcsel_pulse_period(dev,
                                                    VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                    &current_vcsel_pulse_period_p_clk);

            /* Retrieve PRE-RANGE Timeout in Macro periods
             * (MCLKS) */
            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_read_word(dev,
                                           VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                           &pre_range_encoded_time_out);
                pre_range_time_out_m_clks = VL53L0X_decode_timeout(
                                                pre_range_encoded_time_out);
            }
        }

        if (status == VL53L0X_ERROR_NONE) {
            /* Retrieve FINAL-RANGE VCSEL Period */
            status = VL53L0X_get_vcsel_pulse_period(dev,
                                                    VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                                    &current_vcsel_pulse_period_p_clk);
        }

        /* Retrieve FINAL-RANGE Timeout in Macro periods (MCLKS) */
        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_read_word(dev,
                                       VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                       &final_range_encoded_time_out);
            final_range_time_out_m_clks = VL53L0X_decode_timeout(
                                              final_range_encoded_time_out);
        }

        final_range_time_out_m_clks -= pre_range_time_out_m_clks;
        timeout_micro_seconds = VL53L0X_calc_timeout_us(dev,
                                final_range_time_out_m_clks,
                                current_vcsel_pulse_period_p_clk);
    }

    *p_time_out_micro_secs = timeout_micro_seconds;

    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
        uint32_t *p_measurement_timing_budget_micro_seconds)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_SchedulerSequenceSteps_t scheduler_sequence_steps;
    uint32_t final_range_timeout_micro_seconds;
    uint32_t msrc_dcc_tcc_timeout_micro_seconds	= 2000;
    uint32_t start_overhead_micro_seconds		= 1910;
    uint32_t end_overhead_micro_seconds		= 960;
    uint32_t msrc_overhead_micro_seconds		= 660;
    uint32_t tcc_overhead_micro_seconds		= 590;
    uint32_t dss_overhead_micro_seconds		= 690;
    uint32_t pre_range_overhead_micro_seconds	= 660;
    uint32_t final_range_overhead_micro_seconds = 550;
    uint32_t pre_range_timeout_micro_seconds	= 0;

    LOG_FUNCTION_START("");

    /* Start and end overhead times always present */
    *p_measurement_timing_budget_micro_seconds
        = start_overhead_micro_seconds + end_overhead_micro_seconds;

    status = VL53L0X_get_sequence_step_enables(dev, &scheduler_sequence_steps);

    if (status != VL53L0X_ERROR_NONE) {
        LOG_FUNCTION_END(status);
        return status;
    }


    if (scheduler_sequence_steps.TccOn  ||
            scheduler_sequence_steps.MsrcOn ||
            scheduler_sequence_steps.DssOn) {

        status = get_sequence_step_timeout(dev,
                                           VL53L0X_SEQUENCESTEP_MSRC,
                                           &msrc_dcc_tcc_timeout_micro_seconds);

        if (status == VL53L0X_ERROR_NONE) {
            if (scheduler_sequence_steps.TccOn) {
                *p_measurement_timing_budget_micro_seconds +=
                    msrc_dcc_tcc_timeout_micro_seconds +
                    tcc_overhead_micro_seconds;
            }

            if (scheduler_sequence_steps.DssOn) {
                *p_measurement_timing_budget_micro_seconds +=
                    2 * (msrc_dcc_tcc_timeout_micro_seconds +
                         dss_overhead_micro_seconds);
            } else if (scheduler_sequence_steps.MsrcOn) {
                *p_measurement_timing_budget_micro_seconds +=
                    msrc_dcc_tcc_timeout_micro_seconds +
                    msrc_overhead_micro_seconds;
            }
        }
    }

    if (status == VL53L0X_ERROR_NONE) {
        if (scheduler_sequence_steps.PreRangeOn) {
            status = get_sequence_step_timeout(dev,
                                               VL53L0X_SEQUENCESTEP_PRE_RANGE,
                                               &pre_range_timeout_micro_seconds);
            *p_measurement_timing_budget_micro_seconds +=
                pre_range_timeout_micro_seconds +
                pre_range_overhead_micro_seconds;
        }
    }

    if (status == VL53L0X_ERROR_NONE) {
        if (scheduler_sequence_steps.FinalRangeOn) {
            status = get_sequence_step_timeout(dev,
                                               VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                                               &final_range_timeout_micro_seconds);
            *p_measurement_timing_budget_micro_seconds +=
                (final_range_timeout_micro_seconds +
                 final_range_overhead_micro_seconds);
        }
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETPARAMETERFIELD(dev,
                                  MeasurementTimingBudgetMicroSeconds,
                                  *p_measurement_timing_budget_micro_seconds);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
        uint32_t *p_measurement_timing_budget_micro_seconds)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_get_measurement_timing_budget_micro_seconds(dev,
             p_measurement_timing_budget_micro_seconds);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_device_parameters(VL53L0X_DEV dev,
        VL53L0X_DeviceParameters_t *p_device_parameters)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int i;

    LOG_FUNCTION_START("");

    status = VL53L0X_get_device_mode(dev, &(p_device_parameters->DeviceMode));

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_get_inter_measurement_period_milli_seconds(dev,
                 &(p_device_parameters->InterMeasurementPeriodMilliSeconds));


    if (status == VL53L0X_ERROR_NONE) {
        p_device_parameters->XTalkCompensationEnable = 0;
    }

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_get_x_talk_compensation_rate_mega_cps(dev,
                 &(p_device_parameters->XTalkCompensationRateMegaCps));


    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_get_offset_calibration_data_micro_meter(dev,
                 &(p_device_parameters->RangeOffsetMicroMeters));


    if (status == VL53L0X_ERROR_NONE) {
        for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
            /* get first the values, then the enables.
             * VL53L0X_GetLimitCheckValue will modify the enable
             * flags
             */
            if (status == VL53L0X_ERROR_NONE) {
                status |= VL53L0X_get_limit_check_value(dev, i,
                                                        &(p_device_parameters->LimitChecksValue[i]));
            } else {
                break;
            }
            if (status == VL53L0X_ERROR_NONE) {
                status |= VL53L0X_get_limit_check_enable(dev, i,
                          &(p_device_parameters->LimitChecksEnable[i]));
            } else {
                break;
            }
        }
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_wrap_around_check_enable(dev,
                 &(p_device_parameters->WrapAroundCheckEnable));
    }

    /* Need to be done at the end as it uses VCSELPulsePeriod */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_measurement_timing_budget_micro_seconds(dev,
                 &(p_device_parameters->MeasurementTimingBudgetMicroSeconds));
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_limit_check_value(VL53L0X_DEV dev, uint16_t limit_check_id,
        FixPoint1616_t limit_check_value)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t temp8;

    LOG_FUNCTION_START("");

    VL53L0X_GETARRAYPARAMETERFIELD(dev, LimitChecksEnable, limit_check_id,
                                   temp8);

    if (temp8 == 0) { /* disabled write only internal value */
        VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                       limit_check_id, limit_check_value);
    } else {

        switch (limit_check_id) {

            case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
                /* internal computation: */
                VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                               VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                               limit_check_value);
                break;

            case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

                status = VL53L0X_write_word(dev,
                                            VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                                            VL53L0X_FIXPOINT1616TOFIXPOINT97(
                                                limit_check_value));

                break;

            case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:

                /* internal computation: */
                VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                               VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                                               limit_check_value);

                break;

            case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

                /* internal computation: */
                VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                               VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                               limit_check_value);

                break;

            case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:
            case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:

                status = VL53L0X_write_word(dev,
                                            VL53L0X_REG_PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT,
                                            VL53L0X_FIXPOINT1616TOFIXPOINT97(
                                                limit_check_value));

                break;

            default:
                status = VL53L0X_ERROR_INVALID_PARAMS;

        }

        if (status == VL53L0X_ERROR_NONE) {
            VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                           limit_check_id, limit_check_value);
        }
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_data_init(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceParameters_t CurrentParameters;
    int i;
    uint8_t StopVariable;

    LOG_FUNCTION_START("");

    /* by default the I2C is running at 1V8 if you want to change it you
     * need to include this define at compilation level. */
#ifdef USE_I2C_2V8
    Status = VL53L0X_UpdateByte(Dev,
                                VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
                                0xFE,
                                0x01);
#endif

    /* Set I2C standard mode */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0x88, 0x00);
    }

    VL53L0X_SETDEVICESPECIFICPARAMETER(dev, ReadDataFromDeviceDone, 0);

#ifdef USE_IQC_STATION
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_apply_offset_adjustment(Dev);
    }
#endif

    /* Default value is 1000 for Linearity Corrective Gain */
    PALDevDataSet(dev, LinearityCorrectiveGain, 1000);

    /* Dmax default Parameter */
    PALDevDataSet(dev, DmaxCalRangeMilliMeter, 400);
    PALDevDataSet(dev, DmaxCalSignalRateRtnMegaCps,
                  (FixPoint1616_t)((0x00016B85))); /* 1.42 No Cover Glass*/

    /* Set Default static parameters
     *set first temporary values 9.44MHz * 65536 = 618660 */
    VL53L0X_SETDEVICESPECIFICPARAMETER(dev, OscFrequencyMHz, 618660);

    /* Set Default XTalkCompensationRateMegaCps to 0  */
    VL53L0X_SETPARAMETERFIELD(dev, XTalkCompensationRateMegaCps, 0);

    /* Get default parameters */
    status = VL53L0X_get_device_parameters(dev, &CurrentParameters);
    if (status == VL53L0X_ERROR_NONE) {
        /* initialize PAL values */
        CurrentParameters.DeviceMode = VL53L0X_DEVICEMODE_SINGLE_RANGING;
        CurrentParameters.HistogramMode = VL53L0X_HISTOGRAMMODE_DISABLED;
        PALDevDataSet(dev, CurrentParameters, CurrentParameters);
    }

    /* Sigma estimator variable */
    PALDevDataSet(dev, SigmaEstRefArray, 100);
    PALDevDataSet(dev, SigmaEstEffPulseWidth, 900);
    PALDevDataSet(dev, SigmaEstEffAmbWidth, 500);
    PALDevDataSet(dev, targetRefRate, 0x0A00); /* 20 MCPS in 9:7 format */

    /* Use internal default settings */
    PALDevDataSet(dev, UseInternalTuningSettings, 1);

    status |= VL53L0X_write_byte(dev, 0x80, 0x01);
    status |= VL53L0X_write_byte(dev, 0xFF, 0x01);
    status |= VL53L0X_write_byte(dev, 0x00, 0x00);
    status |= VL53L0X_read_byte(dev, 0x91, &StopVariable);
    PALDevDataSet(dev, StopVariable, StopVariable);
    status |= VL53L0X_write_byte(dev, 0x00, 0x01);
    status |= VL53L0X_write_byte(dev, 0xFF, 0x00);
    status |= VL53L0X_write_byte(dev, 0x80, 0x00);

    /* Enable all check */
    for (i = 0; i < VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS; i++) {
        if (status == VL53L0X_ERROR_NONE) {
            status |= VL53L0X_set_limit_check_enable(dev, i, 1);
        } else {
            break;
        }

    }

    /* Disable the following checks */
    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_set_limit_check_enable(dev,
                                                VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, 0);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_set_limit_check_enable(dev,
                                                VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_set_limit_check_enable(dev,
                                                VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC, 0);

    if (status == VL53L0X_ERROR_NONE)
        status = VL53L0X_set_limit_check_enable(dev,
                                                VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE, 0);

    /* Limit default values */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_limit_check_value(dev,
                                               VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                               (FixPoint1616_t)(18 * 65536));
    }
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_limit_check_value(dev,
                                               VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                               (FixPoint1616_t)(25 * 65536 / 100));
        /* 0.25 * 65536 */
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_limit_check_value(dev,
                                               VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                                               (FixPoint1616_t)(35 * 65536));
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_limit_check_value(dev,
                                               VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                               (FixPoint1616_t)(0 * 65536));
    }

    if (status == VL53L0X_ERROR_NONE) {

        PALDevDataSet(dev, SequenceConfig, 0xFF);
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                                    0xFF);

        /* Set PAL state to tell that we are waiting for call to
         * VL53L0X_StaticInit */
        PALDevDataSet(dev, PalState, VL53L0X_STATE_WAIT_STATICINIT);
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev, RefSpadsInitialised, 0);
    }


    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_check_part_used(VL53L0X_DEV dev,
        uint8_t *revision,
        VL53L0X_DeviceInfo_t *p_VL53L0X_device_info)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t module_id_int;
    char *product_id_tmp;

    LOG_FUNCTION_START("");

    status = VL53L0X_get_info_from_device(dev, 2);

    if (status == VL53L0X_ERROR_NONE) {
        module_id_int = VL53L0X_GETDEVICESPECIFICPARAMETER(dev, ModuleId);

        if (module_id_int == 0) {
            *revision = 0;
            VL53L0X_COPYSTRING(p_VL53L0X_device_info->ProductId, "");
        } else {
            *revision = VL53L0X_GETDEVICESPECIFICPARAMETER(dev, Revision);
            product_id_tmp = VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
                             ProductId);
            VL53L0X_COPYSTRING(p_VL53L0X_device_info->ProductId, product_id_tmp);
        }
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_get_device_info(VL53L0X_DEV dev,
        VL53L0X_DeviceInfo_t *p_VL53L0X_device_info)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t revision_id;
    uint8_t revision;

    status = VL53L0X_check_part_used(dev, &revision, p_VL53L0X_device_info);

    if (status == VL53L0X_ERROR_NONE) {
        if (revision == 0) {
            VL53L0X_COPYSTRING(p_VL53L0X_device_info->Name,
                               VL53L0X_STRING_DEVICE_INFO_NAME_TS0);
        } else if ((revision <= 34) && (revision != 32)) {
            VL53L0X_COPYSTRING(p_VL53L0X_device_info->Name,
                               VL53L0X_STRING_DEVICE_INFO_NAME_TS1);
        } else if (revision < 39) {
            VL53L0X_COPYSTRING(p_VL53L0X_device_info->Name,
                               VL53L0X_STRING_DEVICE_INFO_NAME_TS2);
        } else {
            VL53L0X_COPYSTRING(p_VL53L0X_device_info->Name,
                               VL53L0X_STRING_DEVICE_INFO_NAME_ES1);
        }

        VL53L0X_COPYSTRING(p_VL53L0X_device_info->Type,
                           VL53L0X_STRING_DEVICE_INFO_TYPE);

    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_byte(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID,
                                   &p_VL53L0X_device_info->ProductType);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_byte(dev,
                                   VL53L0X_REG_IDENTIFICATION_REVISION_ID,
                                   &revision_id);
        p_VL53L0X_device_info->ProductRevisionMajor = 1;
        p_VL53L0X_device_info->ProductRevisionMinor =
            (revision_id & 0xF0) >> 4;
    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_device_info(VL53L0X_DEV dev,
        VL53L0X_DeviceInfo_t *p_VL53L0X_device_info)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_get_device_info(dev, p_VL53L0X_device_info);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_interrupt_mask_status(VL53L0X_DEV dev,
        uint32_t *p_interrupt_mask_status)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t byte;
    LOG_FUNCTION_START("");

    status = VL53L0X_read_byte(dev, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &byte);
    *p_interrupt_mask_status = byte & 0x07;

    if (byte & 0x18) {
        status = VL53L0X_ERROR_RANGE_ERROR;
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_measurement_data_ready(VL53L0X_DEV dev,
        uint8_t *p_measurement_data_ready)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t sys_range_status_register;
    uint8_t interrupt_config;
    uint32_t interrupt_mask;
    LOG_FUNCTION_START("");

    interrupt_config = VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
                       Pin0GpioFunctionality);

    if (interrupt_config ==
            VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
        status = VL53L0X_get_interrupt_mask_status(dev, &interrupt_mask);
        if (interrupt_mask ==
                VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY) {
            *p_measurement_data_ready = 1;
        } else {
            *p_measurement_data_ready = 0;
        }
    } else {
        status = VL53L0X_read_byte(dev, VL53L0X_REG_RESULT_RANGE_STATUS,
                                   &sys_range_status_register);
        if (status == VL53L0X_ERROR_NONE) {
            if (sys_range_status_register & 0x01) {
                *p_measurement_data_ready = 1;
            } else {
                *p_measurement_data_ready = 0;
            }
        }
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_polling_delay(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    // do nothing
    VL53L0X_OsDelay();
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_measurement_poll_for_completion(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t new_data_ready = 0;
    uint32_t loop_nb;

    LOG_FUNCTION_START("");

    loop_nb = 0;

    do {
        status = VL53L0X_get_measurement_data_ready(dev, &new_data_ready);
        if (status != 0) {
            break; /* the error is set */
        }

        if (new_data_ready == 1) {
            break; /* done note that status == 0 */
        }

        loop_nb++;
        if (loop_nb >= VL53L0X_DEFAULT_MAX_LOOP) {
            status = VL53L0X_ERROR_TIME_OUT;
            break;
        }

        VL53L0X_polling_delay(dev);
    } while (1);

    LOG_FUNCTION_END(status);

    return status;
}

/* Group PAL Interrupt Functions */
VL53L0X_Error VL53L0X::VL53L0X_clear_interrupt_mask(VL53L0X_DEV dev, uint32_t interrupt_mask)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t loop_count;
    uint8_t byte;
    LOG_FUNCTION_START("");

    /* clear bit 0 range interrupt, bit 1 error interrupt */
    loop_count = 0;
    do {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
        status |= VL53L0X_write_byte(dev,
                                     VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x00);
        status |= VL53L0X_read_byte(dev,
                                    VL53L0X_REG_RESULT_INTERRUPT_STATUS, &byte);
        loop_count++;
    } while (((byte & 0x07) != 0x00)
             && (loop_count < 3)
             && (status == VL53L0X_ERROR_NONE));


    if (loop_count >= 3) {
        status = VL53L0X_ERROR_INTERRUPT_NOT_CLEARED;
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_single_ref_calibration(VL53L0X_DEV dev,
        uint8_t vhv_init_byte)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSRANGE_START,
                                    VL53L0X_REG_SYSRANGE_MODE_START_STOP |
                                    vhv_init_byte);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_measurement_poll_for_completion(dev);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_clear_interrupt_mask(dev, 0);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSRANGE_START, 0x00);
    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_ref_calibration_io(VL53L0X_DEV dev, uint8_t read_not_write,
        uint8_t vhv_settings, uint8_t phase_cal,
        uint8_t *p_vhv_settings, uint8_t *p_phase_cal,
        const uint8_t vhv_enable, const uint8_t phase_enable)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t phase_calint = 0;

    /* Read VHV from device */
    status |= VL53L0X_write_byte(dev, 0xFF, 0x01);
    status |= VL53L0X_write_byte(dev, 0x00, 0x00);
    status |= VL53L0X_write_byte(dev, 0xFF, 0x00);

    if (read_not_write) {
        if (vhv_enable) {
            status |= VL53L0X_read_byte(dev, 0xCB, p_vhv_settings);
        }
        if (phase_enable) {
            status |= VL53L0X_read_byte(dev, 0xEE, &phase_calint);
        }
    } else {
        if (vhv_enable) {
            status |= VL53L0X_write_byte(dev, 0xCB, vhv_settings);
        }
        if (phase_enable) {
            status |= VL53L0X_update_byte(dev, 0xEE, 0x80, phase_cal);
        }
    }

    status |= VL53L0X_write_byte(dev, 0xFF, 0x01);
    status |= VL53L0X_write_byte(dev, 0x00, 0x01);
    status |= VL53L0X_write_byte(dev, 0xFF, 0x00);

    *p_phase_cal = (uint8_t)(phase_calint & 0xEF);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_vhv_calibration(VL53L0X_DEV dev,
        uint8_t *p_vhv_settings, const uint8_t get_data_enable,
        const uint8_t restore_config)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t sequence_config = 0;
    uint8_t vhv_settings = 0;
    uint8_t phase_cal = 0;
    uint8_t phase_cal_int = 0;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    if (restore_config) {
        sequence_config = PALDevDataGet(dev, SequenceConfig);
    }

    /* Run VHV */
    status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_perform_single_ref_calibration(dev, 0x40);
    }

    /* Read VHV from device */
    if ((status == VL53L0X_ERROR_NONE) && (get_data_enable == 1)) {
        status = VL53L0X_ref_calibration_io(dev, 1,
                                            vhv_settings, phase_cal, /* Not used here */
                                            p_vhv_settings, &phase_cal_int,
                                            1, 0);
    } else {
        *p_vhv_settings = 0;
    }


    if ((status == VL53L0X_ERROR_NONE) && restore_config) {
        /* restore the previous Sequence Config */
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                                    sequence_config);
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, SequenceConfig, sequence_config);
        }

    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_phase_calibration(VL53L0X_DEV dev,
        uint8_t *p_phase_cal, const uint8_t get_data_enable,
        const uint8_t restore_config)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t sequence_config = 0;
    uint8_t vhv_settings = 0;
    uint8_t phase_cal = 0;
    uint8_t vhv_settingsint;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    if (restore_config) {
        sequence_config = PALDevDataGet(dev, SequenceConfig);
    }

    /* Run PhaseCal */
    status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_perform_single_ref_calibration(dev, 0x0);
    }

    /* Read PhaseCal from device */
    if ((status == VL53L0X_ERROR_NONE) && (get_data_enable == 1)) {
        status = VL53L0X_ref_calibration_io(dev, 1,
                                            vhv_settings, phase_cal, /* Not used here */
                                            &vhv_settingsint, p_phase_cal,
                                            0, 1);
    } else {
        *p_phase_cal = 0;
    }


    if ((status == VL53L0X_ERROR_NONE) && restore_config) {
        /* restore the previous Sequence Config */
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                                    sequence_config);
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, SequenceConfig, sequence_config);
        }

    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_ref_calibration(VL53L0X_DEV dev,
        uint8_t *p_vhv_settings, uint8_t *p_phase_cal, uint8_t get_data_enable)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t sequence_config = 0;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    sequence_config = PALDevDataGet(dev, SequenceConfig);

    /* In the following function we don't save the config to optimize
     * writes on device. Config is saved and restored only once. */
    status = VL53L0X_perform_vhv_calibration(
                 dev, p_vhv_settings, get_data_enable, 0);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_perform_phase_calibration(
                     dev, p_phase_cal, get_data_enable, 0);
    }


    if (status == VL53L0X_ERROR_NONE) {
        /* restore the previous Sequence Config */
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                                    sequence_config);
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, SequenceConfig, sequence_config);
        }

    }

    return status;
}

void VL53L0X::get_next_good_spad(uint8_t good_spad_array[], uint32_t size,
                                 uint32_t curr, int32_t *p_next)
{
    uint32_t start_index;
    uint32_t fine_offset;
    uint32_t c_spads_per_byte = 8;
    uint32_t coarse_index;
    uint32_t fine_index;
    uint8_t data_byte;
    uint8_t success = 0;

    /*
     * Starting with the current good spad, loop through the array to find
     * the next. i.e. the next bit set in the sequence.
     *
     * The coarse index is the byte index of the array and the fine index is
     * the index of the bit within each byte.
     */

    *p_next = -1;

    start_index = curr / c_spads_per_byte;
    fine_offset = curr % c_spads_per_byte;

    for (coarse_index = start_index; ((coarse_index < size) && !success);
            coarse_index++) {
        fine_index = 0;
        data_byte = good_spad_array[coarse_index];

        if (coarse_index == start_index) {
            /* locate the bit position of the provided current
             * spad bit before iterating */
            data_byte >>= fine_offset;
            fine_index = fine_offset;
        }

        while (fine_index < c_spads_per_byte) {
            if ((data_byte & 0x1) == 1) {
                success = 1;
                *p_next = coarse_index * c_spads_per_byte + fine_index;
                break;
            }
            data_byte >>= 1;
            fine_index++;
        }
    }
}

uint8_t VL53L0X::is_aperture(uint32_t spad_index)
{
    /*
     * This function reports if a given spad index is an aperture SPAD by
     * deriving the quadrant.
     */
    uint32_t quadrant;
    uint8_t is_aperture = 1;
    quadrant = spad_index >> 6;
    if (refArrayQuadrants[quadrant] == REF_ARRAY_SPAD_0) {
        is_aperture = 0;
    }

    return is_aperture;
}

VL53L0X_Error VL53L0X::enable_spad_bit(uint8_t spad_array[], uint32_t size,
                                       uint32_t spad_index)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t c_spads_per_byte = 8;
    uint32_t coarse_index;
    uint32_t fine_index;

    coarse_index = spad_index / c_spads_per_byte;
    fine_index = spad_index % c_spads_per_byte;
    if (coarse_index >= size) {
        status = VL53L0X_ERROR_REF_SPAD_INIT;
    } else {
        spad_array[coarse_index] |= (1 << fine_index);
    }

    return status;
}

VL53L0X_Error VL53L0X::set_ref_spad_map(VL53L0X_DEV dev, uint8_t *p_ref_spad_array)
{
    VL53L0X_Error status = VL53L0X_write_multi(dev,
                           VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                           p_ref_spad_array, 6);

    return status;
}

VL53L0X_Error VL53L0X::get_ref_spad_map(VL53L0X_DEV dev, uint8_t *p_ref_spad_array)
{
    VL53L0X_Error status = VL53L0X_read_multi(dev,
                           VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                           p_ref_spad_array,
                           6);
//	VL53L0X_Error status = VL53L0X_ERROR_NONE;
//	uint8_t count=0;

//	for (count = 0; count < 6; count++)
//        status = VL53L0X_RdByte(Dev, (VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 + count), &refSpadArray[count]);
    return status;
}

VL53L0X_Error VL53L0X::enable_ref_spads(VL53L0X_DEV dev,
                                        uint8_t aperture_spads,
                                        uint8_t good_spad_array[],
                                        uint8_t spad_array[],
                                        uint32_t size,
                                        uint32_t start,
                                        uint32_t offset,
                                        uint32_t spad_count,
                                        uint32_t *p_last_spad)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t index;
    uint32_t i;
    int32_t next_good_spad = offset;
    uint32_t current_spad;
    uint8_t check_spad_array[6];

    /*
     * This function takes in a spad array which may or may not have SPADS
     * already enabled and appends from a given offset a requested number
     * of new SPAD enables. The 'good spad map' is applied to
     * determine the next SPADs to enable.
     *
     * This function applies to only aperture or only non-aperture spads.
     * Checks are performed to ensure this.
     */

    current_spad = offset;
    for (index = 0; index < spad_count; index++) {
        get_next_good_spad(good_spad_array, size, current_spad,
                           &next_good_spad);

        if (next_good_spad == -1) {
            status = VL53L0X_ERROR_REF_SPAD_INIT;
            break;
        }

        /* Confirm that the next good SPAD is non-aperture */
        if (is_aperture(start + next_good_spad) != aperture_spads) {
            /* if we can't get the required number of good aperture
             * spads from the current quadrant then this is an error
             */
            status = VL53L0X_ERROR_REF_SPAD_INIT;
            break;
        }
        current_spad = (uint32_t)next_good_spad;
        enable_spad_bit(spad_array, size, current_spad);
        current_spad++;
    }
    *p_last_spad = current_spad;

    if (status == VL53L0X_ERROR_NONE) {
        status = set_ref_spad_map(dev, spad_array);
    }


    if (status == VL53L0X_ERROR_NONE) {
        status = get_ref_spad_map(dev, check_spad_array);

        i = 0;

        /* Compare spad maps. If not equal report error. */
        while (i < size) {
            if (spad_array[i] != check_spad_array[i]) {
                status = VL53L0X_ERROR_REF_SPAD_INIT;
                break;
            }
            i++;
        }
    }
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_device_mode(VL53L0X_DEV dev, VL53L0X_DeviceModes device_mode)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("%d", (int)DeviceMode);

    switch (device_mode) {
        case VL53L0X_DEVICEMODE_SINGLE_RANGING:
        case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
        case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
        case VL53L0X_DEVICEMODE_GPIO_DRIVE:
        case VL53L0X_DEVICEMODE_GPIO_OSC:
            /* Supported modes */
            VL53L0X_SETPARAMETERFIELD(dev, DeviceMode, device_mode);
            break;
        default:
            /* Unsupported mode */
            status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_interrupt_thresholds(VL53L0X_DEV dev,
        VL53L0X_DeviceModes device_mode, FixPoint1616_t threshold_low,
        FixPoint1616_t threshold_high)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint16_t threshold16;
    LOG_FUNCTION_START("");

    /* no dependency on DeviceMode for Ewok */
    /* Need to divide by 2 because the FW will apply a x2 */
    threshold16 = (uint16_t)((threshold_low >> 17) & 0x00fff);
    status = VL53L0X_write_word(dev, VL53L0X_REG_SYSTEM_THRESH_LOW, threshold16);

    if (status == VL53L0X_ERROR_NONE) {
        /* Need to divide by 2 because the FW will apply a x2 */
        threshold16 = (uint16_t)((threshold_high >> 17) & 0x00fff);
        status = VL53L0X_write_word(dev, VL53L0X_REG_SYSTEM_THRESH_HIGH,
                                    threshold16);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_interrupt_thresholds(VL53L0X_DEV dev,
        VL53L0X_DeviceModes device_mode, FixPoint1616_t *p_threshold_low,
        FixPoint1616_t *p_threshold_high)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint16_t threshold16;
    LOG_FUNCTION_START("");

    /* no dependency on DeviceMode for Ewok */

    status = VL53L0X_read_word(dev, VL53L0X_REG_SYSTEM_THRESH_LOW, &threshold16);
    /* Need to multiply by 2 because the FW will apply a x2 */
    *p_threshold_low = (FixPoint1616_t)((0x00fff & threshold16) << 17);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_word(dev, VL53L0X_REG_SYSTEM_THRESH_HIGH,
                                   &threshold16);
        /* Need to multiply by 2 because the FW will apply a x2 */
        *p_threshold_high =
            (FixPoint1616_t)((0x00fff & threshold16) << 17);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_load_tuning_settings(VL53L0X_DEV dev,
        uint8_t *p_tuning_setting_buffer)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    int i;
    int index;
    uint8_t msb;
    uint8_t lsb;
    uint8_t select_param;
    uint8_t number_of_writes;
    uint8_t address;
    uint8_t local_buffer[4]; /* max */
    uint16_t temp16;

    LOG_FUNCTION_START("");

    index = 0;

    while ((*(p_tuning_setting_buffer + index) != 0) &&
            (status == VL53L0X_ERROR_NONE)) {
        number_of_writes = *(p_tuning_setting_buffer + index);
        index++;
        if (number_of_writes == 0xFF) {
            /* internal parameters */
            select_param = *(p_tuning_setting_buffer + index);
            index++;
            switch (select_param) {
                case 0: /* uint16_t SigmaEstRefArray -> 2 bytes */
                    msb = *(p_tuning_setting_buffer + index);
                    index++;
                    lsb = *(p_tuning_setting_buffer + index);
                    index++;
                    temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                    PALDevDataSet(dev, SigmaEstRefArray, temp16);
                    break;
                case 1: /* uint16_t SigmaEstEffPulseWidth -> 2 bytes */
                    msb = *(p_tuning_setting_buffer + index);
                    index++;
                    lsb = *(p_tuning_setting_buffer + index);
                    index++;
                    temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                    PALDevDataSet(dev, SigmaEstEffPulseWidth,
                                  temp16);
                    break;
                case 2: /* uint16_t SigmaEstEffAmbWidth -> 2 bytes */
                    msb = *(p_tuning_setting_buffer + index);
                    index++;
                    lsb = *(p_tuning_setting_buffer + index);
                    index++;
                    temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                    PALDevDataSet(dev, SigmaEstEffAmbWidth, temp16);
                    break;
                case 3: /* uint16_t targetRefRate -> 2 bytes */
                    msb = *(p_tuning_setting_buffer + index);
                    index++;
                    lsb = *(p_tuning_setting_buffer + index);
                    index++;
                    temp16 = VL53L0X_MAKEUINT16(lsb, msb);
                    PALDevDataSet(dev, targetRefRate, temp16);
                    break;
                default: /* invalid parameter */
                    status = VL53L0X_ERROR_INVALID_PARAMS;
            }

        } else if (number_of_writes <= 4) {
            address = *(p_tuning_setting_buffer + index);
            index++;

            for (i = 0; i < number_of_writes; i++) {
                local_buffer[i] = *(p_tuning_setting_buffer +
                                    index);
                index++;
            }

            status = VL53L0X_write_multi(dev, address, local_buffer,
                                         number_of_writes);

        } else {
            status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_check_and_load_interrupt_settings(VL53L0X_DEV dev,
        uint8_t start_not_stopflag)
{
    uint8_t interrupt_config;
    FixPoint1616_t threshold_low;
    FixPoint1616_t threshold_high;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    interrupt_config = VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
                       Pin0GpioFunctionality);

    if ((interrupt_config ==
            VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW) ||
            (interrupt_config ==
             VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH) ||
            (interrupt_config ==
             VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT)) {

        status = VL53L0X_get_interrupt_thresholds(dev,
                 VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                 &threshold_low, &threshold_high);

        if (((threshold_low > 255 * 65536) ||
                (threshold_high > 255 * 65536)) &&
                (status == VL53L0X_ERROR_NONE)) {

            if (start_not_stopflag != 0) {
                status = VL53L0X_load_tuning_settings(dev,
                                                      InterruptThresholdSettings);
            } else {
                status |= VL53L0X_write_byte(dev, 0xFF, 0x04);
                status |= VL53L0X_write_byte(dev, 0x70, 0x00);
                status |= VL53L0X_write_byte(dev, 0xFF, 0x00);
                status |= VL53L0X_write_byte(dev, 0x80, 0x00);
            }

        }


    }

    return status;

}

VL53L0X_Error VL53L0X::VL53L0X_start_measurement(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceModes device_mode;
    uint8_t byte;
    uint8_t start_stop_byte = VL53L0X_REG_SYSRANGE_MODE_START_STOP;
    uint32_t loop_nb;
    LOG_FUNCTION_START("");

    /* Get Current DeviceMode */
    VL53L0X_get_device_mode(dev, &device_mode);

    status = VL53L0X_write_byte(dev, 0x80, 0x01);
    status = VL53L0X_write_byte(dev, 0xFF, 0x01);
    status = VL53L0X_write_byte(dev, 0x00, 0x00);
    status = VL53L0X_write_byte(dev, 0x91, PALDevDataGet(dev, StopVariable));
    status = VL53L0X_write_byte(dev, 0x00, 0x01);
    status = VL53L0X_write_byte(dev, 0xFF, 0x00);
    status = VL53L0X_write_byte(dev, 0x80, 0x00);

    switch (device_mode) {
        case VL53L0X_DEVICEMODE_SINGLE_RANGING:
            status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSRANGE_START, 0x01);

            byte = start_stop_byte;
            if (status == VL53L0X_ERROR_NONE) {
                /* Wait until start bit has been cleared */
                loop_nb = 0;
                do {
                    if (loop_nb > 0)
                        status = VL53L0X_read_byte(dev,
                                                   VL53L0X_REG_SYSRANGE_START, &byte);
                    loop_nb = loop_nb + 1;
                } while (((byte & start_stop_byte) == start_stop_byte)
                         && (status == VL53L0X_ERROR_NONE)
                         && (loop_nb < VL53L0X_DEFAULT_MAX_LOOP));

                if (loop_nb >= VL53L0X_DEFAULT_MAX_LOOP) {
                    status = VL53L0X_ERROR_TIME_OUT;
                }

            }

            break;
        case VL53L0X_DEVICEMODE_CONTINUOUS_RANGING:
            /* Back-to-back mode */

            /* Check if need to apply interrupt settings */
            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_check_and_load_interrupt_settings(dev, 1);
            }

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_SYSRANGE_START,
                                        VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK);
            if (status == VL53L0X_ERROR_NONE) {
                /* Set PAL State to Running */
                PALDevDataSet(dev, PalState, VL53L0X_STATE_RUNNING);
            }
            break;
        case VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING:
            /* Continuous mode */
            /* Check if need to apply interrupt settings */
            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_check_and_load_interrupt_settings(dev, 1);
            }

            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_SYSRANGE_START,
                                        VL53L0X_REG_SYSRANGE_MODE_TIMED);

            if (status == VL53L0X_ERROR_NONE) {
                /* Set PAL State to Running */
                PALDevDataSet(dev, PalState, VL53L0X_STATE_RUNNING);
            }
            break;
        default:
            /* Selected mode not supported */
            status = VL53L0X_ERROR_MODE_NOT_SUPPORTED;
    }


    LOG_FUNCTION_END(status);
    return status;
}

/* Group PAL Measurement Functions */
VL53L0X_Error VL53L0X::VL53L0X_perform_single_measurement(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceModes device_mode;

    LOG_FUNCTION_START("");

    /* Get Current DeviceMode */
    status = VL53L0X_get_device_mode(dev, &device_mode);

    /* Start immediately to run a single ranging measurement in case of
     * single ranging or single histogram */
    if (status == VL53L0X_ERROR_NONE
            && device_mode == VL53L0X_DEVICEMODE_SINGLE_RANGING) {
        status = VL53L0X_start_measurement(dev);
    }


    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_measurement_poll_for_completion(dev);
    }


    /* Change PAL State in case of single ranging or single histogram */
    if (status == VL53L0X_ERROR_NONE
            && device_mode == VL53L0X_DEVICEMODE_SINGLE_RANGING) {
        PALDevDataSet(dev, PalState, VL53L0X_STATE_IDLE);
    }


    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_x_talk_compensation_enable(VL53L0X_DEV dev,
        uint8_t *p_x_talk_compensation_enable)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t temp8;
    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(dev, XTalkCompensationEnable, temp8);
    *p_x_talk_compensation_enable = temp8;

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_total_xtalk_rate(VL53L0X_DEV dev,
        VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
        FixPoint1616_t *p_total_xtalk_rate_mcps)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    uint8_t xtalk_comp_enable;
    FixPoint1616_t total_xtalk_mega_cps;
    FixPoint1616_t xtalk_per_spad_mega_cps;

    *p_total_xtalk_rate_mcps = 0;

    status = VL53L0X_get_x_talk_compensation_enable(dev, &xtalk_comp_enable);
    if (status == VL53L0X_ERROR_NONE) {

        if (xtalk_comp_enable) {

            VL53L0X_GETPARAMETERFIELD(
                dev,
                XTalkCompensationRateMegaCps,
                xtalk_per_spad_mega_cps);

            /* FixPoint1616 * FixPoint 8:8 = FixPoint0824 */
            total_xtalk_mega_cps =
                p_ranging_measurement_data->EffectiveSpadRtnCount *
                xtalk_per_spad_mega_cps;

            /* FixPoint0824 >> 8 = FixPoint1616 */
            *p_total_xtalk_rate_mcps =
                (total_xtalk_mega_cps + 0x80) >> 8;
        }
    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_total_signal_rate(VL53L0X_DEV dev,
        VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
        FixPoint1616_t *p_total_signal_rate_mcps)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    FixPoint1616_t total_xtalk_mega_cps;

    LOG_FUNCTION_START("");

    *p_total_signal_rate_mcps =
        p_ranging_measurement_data->SignalRateRtnMegaCps;

    status = VL53L0X_get_total_xtalk_rate(
                 dev, p_ranging_measurement_data, &total_xtalk_mega_cps);

    if (status == VL53L0X_ERROR_NONE) {
        *p_total_signal_rate_mcps += total_xtalk_mega_cps;
    }

    return status;
}

/* To convert ms into register value */
uint32_t VL53L0X::VL53L0X_calc_timeout_mclks(VL53L0X_DEV dev,
        uint32_t timeout_period_us,
        uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ps;
    uint32_t macro_period_ns;
    uint32_t timeout_period_mclks = 0;

    macro_period_ps = VL53L0X_calc_macro_period_ps(dev, vcsel_period_pclks);
    macro_period_ns = (macro_period_ps + 500) / 1000;

    timeout_period_mclks =
        (uint32_t)(((timeout_period_us * 1000)
                    + (macro_period_ns / 2)) / macro_period_ns);

    return timeout_period_mclks;
}

uint32_t VL53L0X::VL53L0X_isqrt(uint32_t num)
{
    /*
     * Implements an integer square root
     *
     * From: http://en.wikipedia.org/wiki/Methods_of_computing_square_roots
     */

    uint32_t  res = 0;
    uint32_t  bit = 1 << 30;
    /* The second-to-top bit is set:
     *	1 << 14 for 16-bits, 1 << 30 for 32 bits */

    /* "bit" starts at the highest power of four <= the argument. */
    while (bit > num) {
        bit >>= 2;
    }


    while (bit != 0) {
        if (num >= res + bit) {
            num -= res + bit;
            res = (res >> 1) + bit;
        } else {
            res >>= 1;
        }

        bit >>= 2;
    }

    return res;
}

VL53L0X_Error VL53L0X::VL53L0X_calc_dmax(
    VL53L0X_DEV dev,
    FixPoint1616_t total_signal_rate_mcps,
    FixPoint1616_t total_corr_signal_rate_mcps,
    FixPoint1616_t pw_mult,
    uint32_t sigma_estimate_p1,
    FixPoint1616_t sigma_estimate_p2,
    uint32_t peak_vcsel_duration_us,
    uint32_t *pd_max_mm)
{
    const uint32_t c_sigma_limit		= 18;
    const FixPoint1616_t c_signal_limit	= 0x4000; /* 0.25 */
    const FixPoint1616_t c_sigma_est_ref	= 0x00000042; /* 0.001 */
    const uint32_t c_amb_eff_width_sigma_est_ns = 6;
    const uint32_t c_amb_eff_width_d_max_ns	   = 7;
    uint32_t dmax_cal_range_mm;
    FixPoint1616_t dmax_cal_signal_rate_rtn_mcps;
    FixPoint1616_t min_signal_needed;
    FixPoint1616_t min_signal_needed_p1;
    FixPoint1616_t min_signal_needed_p2;
    FixPoint1616_t min_signal_needed_p3;
    FixPoint1616_t min_signal_needed_p4;
    FixPoint1616_t sigma_limit_tmp;
    FixPoint1616_t sigma_est_sq_tmp;
    FixPoint1616_t signal_limit_tmp;
    FixPoint1616_t signal_at0_mm;
    FixPoint1616_t dmax_dark;
    FixPoint1616_t dmax_ambient;
    FixPoint1616_t dmax_dark_tmp;
    FixPoint1616_t sigma_est_p2_tmp;
    uint32_t signal_rate_temp_mcps;

    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    dmax_cal_range_mm =
        PALDevDataGet(dev, DmaxCalRangeMilliMeter);

    dmax_cal_signal_rate_rtn_mcps =
        PALDevDataGet(dev, DmaxCalSignalRateRtnMegaCps);

    /* uint32 * FixPoint1616 = FixPoint1616 */
    signal_at0_mm = dmax_cal_range_mm * dmax_cal_signal_rate_rtn_mcps;

    /* FixPoint1616 >> 8 = FixPoint2408 */
    signal_at0_mm = (signal_at0_mm + 0x80) >> 8;
    signal_at0_mm *= dmax_cal_range_mm;

    min_signal_needed_p1 = 0;
    if (total_corr_signal_rate_mcps > 0) {

        /* Shift by 10 bits to increase resolution prior to the
         * division */
        signal_rate_temp_mcps = total_signal_rate_mcps << 10;

        /* Add rounding value prior to division */
        min_signal_needed_p1 = signal_rate_temp_mcps +
                               (total_corr_signal_rate_mcps / 2);

        /* FixPoint0626/FixPoint1616 = FixPoint2210 */
        min_signal_needed_p1 /= total_corr_signal_rate_mcps;

        /* Apply a factored version of the speed of light.
         Correction to be applied at the end */
        min_signal_needed_p1 *= 3;

        /* FixPoint2210 * FixPoint2210 = FixPoint1220 */
        min_signal_needed_p1 *= min_signal_needed_p1;

        /* FixPoint1220 >> 16 = FixPoint2804 */
        min_signal_needed_p1 = (min_signal_needed_p1 + 0x8000) >> 16;
    }

    min_signal_needed_p2 = pw_mult * sigma_estimate_p1;

    /* FixPoint1616 >> 16 =	 uint32 */
    min_signal_needed_p2 = (min_signal_needed_p2 + 0x8000) >> 16;

    /* uint32 * uint32	=  uint32 */
    min_signal_needed_p2 *= min_signal_needed_p2;

    /* Check sigmaEstimateP2
     * If this value is too high there is not enough signal rate
     * to calculate dmax value so set a suitable value to ensure
     * a very small dmax.
     */
    sigma_est_p2_tmp = (sigma_estimate_p2 + 0x8000) >> 16;
    sigma_est_p2_tmp = (sigma_est_p2_tmp + c_amb_eff_width_sigma_est_ns / 2) /
                       c_amb_eff_width_sigma_est_ns;
    sigma_est_p2_tmp *= c_amb_eff_width_d_max_ns;

    if (sigma_est_p2_tmp > 0xffff) {
        min_signal_needed_p3 = 0xfff00000;
    } else {

        /* DMAX uses a different ambient width from sigma, so apply
         * correction.
         * Perform division before multiplication to prevent overflow.
         */
        sigma_estimate_p2 = (sigma_estimate_p2 + c_amb_eff_width_sigma_est_ns / 2) /
                            c_amb_eff_width_sigma_est_ns;
        sigma_estimate_p2 *= c_amb_eff_width_d_max_ns;

        /* FixPoint1616 >> 16 = uint32 */
        min_signal_needed_p3 = (sigma_estimate_p2 + 0x8000) >> 16;

        min_signal_needed_p3 *= min_signal_needed_p3;

    }

    /* FixPoint1814 / uint32 = FixPoint1814 */
    sigma_limit_tmp = ((c_sigma_limit << 14) + 500) / 1000;

    /* FixPoint1814 * FixPoint1814 = FixPoint3628 := FixPoint0428 */
    sigma_limit_tmp *= sigma_limit_tmp;

    /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
    sigma_est_sq_tmp = c_sigma_est_ref * c_sigma_est_ref;

    /* FixPoint3232 >> 4 = FixPoint0428 */
    sigma_est_sq_tmp = (sigma_est_sq_tmp + 0x08) >> 4;

    /* FixPoint0428 - FixPoint0428	= FixPoint0428 */
    sigma_limit_tmp -=  sigma_est_sq_tmp;

    /* uint32_t * FixPoint0428 = FixPoint0428 */
    min_signal_needed_p4 = 4 * 12 * sigma_limit_tmp;

    /* FixPoint0428 >> 14 = FixPoint1814 */
    min_signal_needed_p4 = (min_signal_needed_p4 + 0x2000) >> 14;

    /* uint32 + uint32 = uint32 */
    min_signal_needed = (min_signal_needed_p2 + min_signal_needed_p3);

    /* uint32 / uint32 = uint32 */
    min_signal_needed += (peak_vcsel_duration_us / 2);
    min_signal_needed /= peak_vcsel_duration_us;

    /* uint32 << 14 = FixPoint1814 */
    min_signal_needed <<= 14;

    /* FixPoint1814 / FixPoint1814 = uint32 */
    min_signal_needed += (min_signal_needed_p4 / 2);
    min_signal_needed /= min_signal_needed_p4;

    /* FixPoint3200 * FixPoint2804 := FixPoint2804*/
    min_signal_needed *= min_signal_needed_p1;

    /* Apply correction by dividing by 1000000.
     * This assumes 10E16 on the numerator of the equation
     * and 10E-22 on the denominator.
     * We do this because 32bit fix point calculation can't
     * handle the larger and smaller elements of this equation,
     * i.e. speed of light and pulse widths.
     */
    min_signal_needed = (min_signal_needed + 500) / 1000;
    min_signal_needed <<= 4;

    min_signal_needed = (min_signal_needed + 500) / 1000;

    /* FixPoint1616 >> 8 = FixPoint2408 */
    signal_limit_tmp = (c_signal_limit + 0x80) >> 8;

    /* FixPoint2408/FixPoint2408 = uint32 */
    if (signal_limit_tmp != 0) {
        dmax_dark_tmp = (signal_at0_mm + (signal_limit_tmp / 2))
                        / signal_limit_tmp;
    } else {
        dmax_dark_tmp = 0;
    }

    dmax_dark = VL53L0X_isqrt(dmax_dark_tmp);

    /* FixPoint2408/FixPoint2408 = uint32 */
    if (min_signal_needed != 0) {
        dmax_ambient = (signal_at0_mm + min_signal_needed / 2)
                       / min_signal_needed;
    } else {
        dmax_ambient = 0;
    }

    dmax_ambient = VL53L0X_isqrt(dmax_ambient);

    *pd_max_mm = dmax_dark;
    if (dmax_dark > dmax_ambient) {
        *pd_max_mm = dmax_ambient;
    }

    LOG_FUNCTION_END(status);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_calc_sigma_estimate(VL53L0X_DEV dev,
        VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
        FixPoint1616_t *p_sigma_estimate,
        uint32_t *p_dmax_mm)
{
    /* Expressed in 100ths of a ns, i.e. centi-ns */
    const uint32_t c_pulse_effective_width_centi_ns   = 800;
    /* Expressed in 100ths of a ns, i.e. centi-ns */
    const uint32_t c_ambient_effective_width_centi_ns = 600;
    const FixPoint1616_t c_dflt_final_range_integration_time_milli_secs	= 0x00190000; /* 25ms */
    const uint32_t c_vcsel_pulse_width_ps	= 4700; /* pico secs */
    const FixPoint1616_t c_sigma_est_max	= 0x028F87AE;
    const FixPoint1616_t c_sigma_est_rtn_max	= 0xF000;
    const FixPoint1616_t c_amb_to_signal_ratio_max = 0xF0000000 /
            c_ambient_effective_width_centi_ns;
    /* Time Of Flight per mm (6.6 pico secs) */
    const FixPoint1616_t c_tof_per_mm_ps		= 0x0006999A;
    const uint32_t c_16bit_rounding_param		= 0x00008000;
    const FixPoint1616_t c_max_x_talk_kcps		= 0x00320000;
    const uint32_t c_pll_period_ps			= 1655;

    uint32_t vcsel_total_events_rtn;
    uint32_t final_range_timeout_micro_secs;
    uint32_t pre_range_timeout_micro_secs;
    uint32_t final_range_integration_time_milli_secs;
    FixPoint1616_t sigma_estimate_p1;
    FixPoint1616_t sigma_estimate_p2;
    FixPoint1616_t sigma_estimate_p3;
    FixPoint1616_t delta_t_ps;
    FixPoint1616_t pw_mult;
    FixPoint1616_t sigma_est_rtn;
    FixPoint1616_t sigma_estimate;
    FixPoint1616_t x_talk_correction;
    FixPoint1616_t ambient_rate_kcps;
    FixPoint1616_t peak_signal_rate_kcps;
    FixPoint1616_t x_talk_comp_rate_mcps;
    uint32_t x_talk_comp_rate_kcps;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    FixPoint1616_t diff1_mcps;
    FixPoint1616_t diff2_mcps;
    FixPoint1616_t sqr1;
    FixPoint1616_t sqr2;
    FixPoint1616_t sqr_sum;
    FixPoint1616_t sqrt_result_centi_ns;
    FixPoint1616_t sqrt_result;
    FixPoint1616_t total_signal_rate_mcps;
    FixPoint1616_t corrected_signal_rate_mcps;
    FixPoint1616_t sigma_est_ref;
    uint32_t vcsel_width;
    uint32_t final_range_macro_pclks;
    uint32_t pre_range_macro_pclks;
    uint32_t peak_vcsel_duration_us;
    uint8_t final_range_vcsel_pclks;
    uint8_t pre_range_vcsel_pclks;
    /*! \addtogroup calc_sigma_estimate
     * @{
     *
     * Estimates the range sigma
     */

    LOG_FUNCTION_START("");

    VL53L0X_GETPARAMETERFIELD(dev, XTalkCompensationRateMegaCps,
                              x_talk_comp_rate_mcps);

    /*
     * We work in kcps rather than mcps as this helps keep within the
     * confines of the 32 Fix1616 type.
     */

    ambient_rate_kcps =
        (p_ranging_measurement_data->AmbientRateRtnMegaCps * 1000) >> 16;

    corrected_signal_rate_mcps =
        p_ranging_measurement_data->SignalRateRtnMegaCps;


    status = VL53L0X_get_total_signal_rate(
                 dev, p_ranging_measurement_data, &total_signal_rate_mcps);
    status = VL53L0X_get_total_xtalk_rate(
                 dev, p_ranging_measurement_data, &x_talk_comp_rate_mcps);


    /* Signal rate measurement provided by device is the
     * peak signal rate, not average.
     */
    peak_signal_rate_kcps = (total_signal_rate_mcps * 1000);
    peak_signal_rate_kcps = (peak_signal_rate_kcps + 0x8000) >> 16;

    x_talk_comp_rate_kcps = x_talk_comp_rate_mcps * 1000;

    if (x_talk_comp_rate_kcps > c_max_x_talk_kcps) {
        x_talk_comp_rate_kcps = c_max_x_talk_kcps;
    }

    if (status == VL53L0X_ERROR_NONE) {

        /* Calculate final range macro periods */
        final_range_timeout_micro_secs = VL53L0X_GETDEVICESPECIFICPARAMETER(
                                             dev, FinalRangeTimeoutMicroSecs);

        final_range_vcsel_pclks = VL53L0X_GETDEVICESPECIFICPARAMETER(
                                      dev, FinalRangeVcselPulsePeriod);

        final_range_macro_pclks = VL53L0X_calc_timeout_mclks(
                                      dev, final_range_timeout_micro_secs, final_range_vcsel_pclks);

        /* Calculate pre-range macro periods */
        pre_range_timeout_micro_secs = VL53L0X_GETDEVICESPECIFICPARAMETER(
                                           dev, PreRangeTimeoutMicroSecs);

        pre_range_vcsel_pclks = VL53L0X_GETDEVICESPECIFICPARAMETER(
                                    dev, PreRangeVcselPulsePeriod);

        pre_range_macro_pclks = VL53L0X_calc_timeout_mclks(
                                    dev, pre_range_timeout_micro_secs, pre_range_vcsel_pclks);

        vcsel_width = 3;
        if (final_range_vcsel_pclks == 8) {
            vcsel_width = 2;
        }


        peak_vcsel_duration_us = vcsel_width * 2048 *
                                 (pre_range_macro_pclks + final_range_macro_pclks);
        peak_vcsel_duration_us = (peak_vcsel_duration_us + 500) / 1000;
        peak_vcsel_duration_us *= c_pll_period_ps;
        peak_vcsel_duration_us = (peak_vcsel_duration_us + 500) / 1000;

        /* Fix1616 >> 8 = Fix2408 */
        total_signal_rate_mcps = (total_signal_rate_mcps + 0x80) >> 8;

        /* Fix2408 * uint32 = Fix2408 */
        vcsel_total_events_rtn = total_signal_rate_mcps *
                                 peak_vcsel_duration_us;

        /* Fix2408 >> 8 = uint32 */
        vcsel_total_events_rtn = (vcsel_total_events_rtn + 0x80) >> 8;

        /* Fix2408 << 8 = Fix1616 = */
        total_signal_rate_mcps <<= 8;
    }

    if (status != VL53L0X_ERROR_NONE) {
        LOG_FUNCTION_END(status);
        return status;
    }

    if (peak_signal_rate_kcps == 0) {
        *p_sigma_estimate = c_sigma_est_max;
        PALDevDataSet(dev, SigmaEstimate, c_sigma_est_max);
        *p_dmax_mm = 0;
    } else {
        if (vcsel_total_events_rtn < 1) {
            vcsel_total_events_rtn = 1;
        }

        sigma_estimate_p1 = c_pulse_effective_width_centi_ns;

        /* ((FixPoint1616 << 16)* uint32)/uint32 = FixPoint1616 */
        sigma_estimate_p2 = (ambient_rate_kcps << 16) / peak_signal_rate_kcps;
        if (sigma_estimate_p2 > c_amb_to_signal_ratio_max) {
            /* Clip to prevent overflow. Will ensure safe
             * max result. */
            sigma_estimate_p2 = c_amb_to_signal_ratio_max;
        }
        sigma_estimate_p2 *= c_ambient_effective_width_centi_ns;

        sigma_estimate_p3 = 2 * VL53L0X_isqrt(vcsel_total_events_rtn * 12);

        /* uint32 * FixPoint1616 = FixPoint1616 */
        delta_t_ps = p_ranging_measurement_data->RangeMilliMeter *
                     c_tof_per_mm_ps;

        /*
         * vcselRate - xtalkCompRate
         * (uint32 << 16) - FixPoint1616 = FixPoint1616.
         * Divide result by 1000 to convert to mcps.
         * 500 is added to ensure rounding when integer division
         * truncates.
         */
        diff1_mcps = (((peak_signal_rate_kcps << 16) -
                       2 * x_talk_comp_rate_kcps) + 500) / 1000;

        /* vcselRate + xtalkCompRate */
        diff2_mcps = ((peak_signal_rate_kcps << 16) + 500) / 1000;

        /* Shift by 8 bits to increase resolution prior to the
         * division */
        diff1_mcps <<= 8;

        /* FixPoint0824/FixPoint1616 = FixPoint2408 */
//		xTalkCorrection	 = abs(diff1_mcps/diff2_mcps);
// abs is causing compiler overloading isue in C++, but unsigned types. So, redundant call anyway!
        x_talk_correction	 = diff1_mcps / diff2_mcps;

        /* FixPoint2408 << 8 = FixPoint1616 */
        x_talk_correction <<= 8;

        if (p_ranging_measurement_data->RangeStatus != 0) {
            pw_mult = 1 << 16;
        } else {
            /* FixPoint1616/uint32 = FixPoint1616 */
            pw_mult = delta_t_ps / c_vcsel_pulse_width_ps; /* smaller than 1.0f */

            /*
             * FixPoint1616 * FixPoint1616 = FixPoint3232, however both
             * values are small enough such that32 bits will not be
             * exceeded.
             */
            pw_mult *= ((1 << 16) - x_talk_correction);

            /* (FixPoint3232 >> 16) = FixPoint1616 */
            pw_mult = (pw_mult + c_16bit_rounding_param) >> 16;

            /* FixPoint1616 + FixPoint1616 = FixPoint1616 */
            pw_mult += (1 << 16);

            /*
             * At this point the value will be 1.xx, therefore if we square
             * the value this will exceed 32 bits. To address this perform
             * a single shift to the right before the multiplication.
             */
            pw_mult >>= 1;
            /* FixPoint1715 * FixPoint1715 = FixPoint3430 */
            pw_mult = pw_mult * pw_mult;

            /* (FixPoint3430 >> 14) = Fix1616 */
            pw_mult >>= 14;
        }

        /* FixPoint1616 * uint32 = FixPoint1616 */
        sqr1 = pw_mult * sigma_estimate_p1;

        /* (FixPoint1616 >> 16) = FixPoint3200 */
        sqr1 = (sqr1 + 0x8000) >> 16;

        /* FixPoint3200 * FixPoint3200 = FixPoint6400 */
        sqr1 *= sqr1;

        sqr2 = sigma_estimate_p2;

        /* (FixPoint1616 >> 16) = FixPoint3200 */
        sqr2 = (sqr2 + 0x8000) >> 16;

        /* FixPoint3200 * FixPoint3200 = FixPoint6400 */
        sqr2 *= sqr2;

        /* FixPoint64000 + FixPoint6400 = FixPoint6400 */
        sqr_sum = sqr1 + sqr2;

        /* SQRT(FixPoin6400) = FixPoint3200 */
        sqrt_result_centi_ns = VL53L0X_isqrt(sqr_sum);

        /* (FixPoint3200 << 16) = FixPoint1616 */
        sqrt_result_centi_ns <<= 16;

        /*
         * Note that the Speed Of Light is expressed in um per 1E-10
         * seconds (2997) Therefore to get mm/ns we have to divide by
         * 10000
         */
        sigma_est_rtn = (((sqrt_result_centi_ns + 50) / 100) /
                         sigma_estimate_p3);
        sigma_est_rtn		 *= VL53L0X_SPEED_OF_LIGHT_IN_AIR;

        /* Add 5000 before dividing by 10000 to ensure rounding. */
        sigma_est_rtn		 += 5000;
        sigma_est_rtn		 /= 10000;

        if (sigma_est_rtn > c_sigma_est_rtn_max) {
            /* Clip to prevent overflow. Will ensure safe
             * max result. */
            sigma_est_rtn = c_sigma_est_rtn_max;
        }
        final_range_integration_time_milli_secs =
            (final_range_timeout_micro_secs + pre_range_timeout_micro_secs + 500) / 1000;

        /* sigmaEstRef = 1mm * 25ms/final range integration time (inc pre-range)
         * sqrt(FixPoint1616/int) = FixPoint2408)
         */
        sigma_est_ref =
            VL53L0X_isqrt((c_dflt_final_range_integration_time_milli_secs +
                           final_range_integration_time_milli_secs / 2) /
                          final_range_integration_time_milli_secs);

        /* FixPoint2408 << 8 = FixPoint1616 */
        sigma_est_ref <<= 8;
        sigma_est_ref = (sigma_est_ref + 500) / 1000;

        /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
        sqr1 = sigma_est_rtn * sigma_est_rtn;
        /* FixPoint1616 * FixPoint1616 = FixPoint3232 */
        sqr2 = sigma_est_ref * sigma_est_ref;

        /* sqrt(FixPoint3232) = FixPoint1616 */
        sqrt_result = VL53L0X_isqrt((sqr1 + sqr2));
        /*
         * Note that the Shift by 4 bits increases resolution prior to
         * the sqrt, therefore the result must be shifted by 2 bits to
         * the right to revert back to the FixPoint1616 format.
         */

        sigma_estimate	 = 1000 * sqrt_result;

        if ((peak_signal_rate_kcps < 1) || (vcsel_total_events_rtn < 1) ||
                (sigma_estimate > c_sigma_est_max)) {
            sigma_estimate = c_sigma_est_max;
        }

        *p_sigma_estimate = (uint32_t)(sigma_estimate);
        PALDevDataSet(dev, SigmaEstimate, *p_sigma_estimate);
        status = VL53L0X_calc_dmax(
                     dev,
                     total_signal_rate_mcps,
                     corrected_signal_rate_mcps,
                     pw_mult,
                     sigma_estimate_p1,
                     sigma_estimate_p2,
                     peak_vcsel_duration_us,
                     p_dmax_mm);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_pal_range_status(VL53L0X_DEV dev,
        uint8_t device_range_status,
        FixPoint1616_t signal_rate,
        uint16_t effective_spad_rtn_count,
        VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data,
        uint8_t *p_pal_range_status)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t none_flag;
    uint8_t sigma_limitflag = 0;
    uint8_t signal_ref_clipflag = 0;
    uint8_t range_ignore_thresholdflag = 0;
    uint8_t sigma_limit_check_enable = 0;
    uint8_t signal_rate_final_range_limit_check_enable = 0;
    uint8_t signal_ref_clip_limit_check_enable = 0;
    uint8_t range_ignore_threshold_limit_check_enable = 0;
    FixPoint1616_t sigma_estimate;
    FixPoint1616_t sigma_limit_value;
    FixPoint1616_t signal_ref_clip_value;
    FixPoint1616_t range_ignore_threshold_value;
    FixPoint1616_t signal_rate_per_spad;
    uint8_t device_range_status_internal = 0;
    uint16_t tmp_word = 0;
    uint8_t temp8;
    uint32_t dmax_mm = 0;
    FixPoint1616_t last_signal_ref_mcps;

    LOG_FUNCTION_START("");


    /*
     * VL53L0X has a good ranging when the value of the
     * DeviceRangeStatus = 11. This function will replace the value 0 with
     * the value 11 in the DeviceRangeStatus.
     * In addition, the SigmaEstimator is not included in the VL53L0X
     * DeviceRangeStatus, this will be added in the PalRangeStatus.
     */

    device_range_status_internal = ((device_range_status & 0x78) >> 3);

    if (device_range_status_internal == 0 ||
            device_range_status_internal == 5 ||
            device_range_status_internal == 7 ||
            device_range_status_internal == 12 ||
            device_range_status_internal == 13 ||
            device_range_status_internal == 14 ||
            device_range_status_internal == 15
       ) {
        none_flag = 1;
    } else {
        none_flag = 0;
    }

    /*
     * Check if Sigma limit is enabled, if yes then do comparison with limit
     * value and put the result back into pPalRangeStatus.
     */
    if (status == VL53L0X_ERROR_NONE) {
        status =  VL53L0X_get_limit_check_enable(dev,
                  VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                  &sigma_limit_check_enable);
    }

    if ((sigma_limit_check_enable != 0) && (status == VL53L0X_ERROR_NONE)) {
        /*
        * compute the Sigma and check with limit
        */
        status = VL53L0X_calc_sigma_estimate(
                     dev,
                     p_ranging_measurement_data,
                     &sigma_estimate,
                     &dmax_mm);
        if (status == VL53L0X_ERROR_NONE) {
            p_ranging_measurement_data->RangeDMaxMilliMeter = dmax_mm;
        }

        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_get_limit_check_value(dev,
                                                   VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                                   &sigma_limit_value);

            if ((sigma_limit_value > 0) &&
                    (sigma_estimate > sigma_limit_value)) {
                /* Limit Fail */
                sigma_limitflag = 1;
            }
        }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     */
    if (status == VL53L0X_ERROR_NONE) {
        status =  VL53L0X_get_limit_check_enable(dev,
                  VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                  &signal_ref_clip_limit_check_enable);
    }

    if ((signal_ref_clip_limit_check_enable != 0) &&
            (status == VL53L0X_ERROR_NONE)) {

        status = VL53L0X_get_limit_check_value(dev,
                                               VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                                               &signal_ref_clip_value);

        /* Read LastSignalRefMcps from device */
        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_write_byte(dev, 0xFF, 0x01);
        }

        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_read_word(dev,
                                       VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF,
                                       &tmp_word);
        }

        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_write_byte(dev, 0xFF, 0x00);
        }

        last_signal_ref_mcps = VL53L0X_FIXPOINT97TOFIXPOINT1616(tmp_word);
        PALDevDataSet(dev, LastSignalRefMcps, last_signal_ref_mcps);

        if ((signal_ref_clip_value > 0) &&
                (last_signal_ref_mcps > signal_ref_clip_value)) {
            /* Limit Fail */
            signal_ref_clipflag = 1;
        }
    }

    /*
     * Check if Signal ref clip limit is enabled, if yes then do comparison
     * with limit value and put the result back into pPalRangeStatus.
     * EffectiveSpadRtnCount has a format 8.8
     * If (Return signal rate < (1.5 x Xtalk x number of Spads)) : FAIL
     */
    if (status == VL53L0X_ERROR_NONE) {
        status =  VL53L0X_get_limit_check_enable(dev,
                  VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                  &range_ignore_threshold_limit_check_enable);
    }

    if ((range_ignore_threshold_limit_check_enable != 0) &&
            (status == VL53L0X_ERROR_NONE)) {

        /* Compute the signal rate per spad */
        if (effective_spad_rtn_count == 0) {
            signal_rate_per_spad = 0;
        } else {
            signal_rate_per_spad = (FixPoint1616_t)((256 * signal_rate)
                                                    / effective_spad_rtn_count);
        }

        status = VL53L0X_get_limit_check_value(dev,
                                               VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                               &range_ignore_threshold_value);

        if ((range_ignore_threshold_value > 0) &&
                (signal_rate_per_spad < range_ignore_threshold_value)) {
            /* Limit Fail add 2^6 to range status */
            range_ignore_thresholdflag = 1;
        }
    }

    if (status == VL53L0X_ERROR_NONE) {
        if (none_flag == 1) {
            *p_pal_range_status = 255;	 /* NONE */
        } else if (device_range_status_internal == 1 ||
                   device_range_status_internal == 2 ||
                   device_range_status_internal == 3) {
            *p_pal_range_status = 5; /* HW fail */
        } else if (device_range_status_internal == 6 ||
                   device_range_status_internal == 9) {
            *p_pal_range_status = 4;  /* Phase fail */
        } else if (device_range_status_internal == 8 ||
                   device_range_status_internal == 10 ||
                   signal_ref_clipflag == 1) {
            *p_pal_range_status = 3;  /* Min range */
        } else if (device_range_status_internal == 4 ||
                   range_ignore_thresholdflag == 1) {
            *p_pal_range_status = 2;  /* Signal Fail */
        } else if (sigma_limitflag == 1) {
            *p_pal_range_status = 1;  /* Sigma	 Fail */
        } else {
            *p_pal_range_status = 0; /* Range Valid */
        }
    }

    /* DMAX only relevant during range error */
    if (*p_pal_range_status == 0) {
        p_ranging_measurement_data->RangeDMaxMilliMeter = 0;
    }

    /* fill the Limit Check Status */

    status =  VL53L0X_get_limit_check_enable(dev,
              VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
              &signal_rate_final_range_limit_check_enable);

    if (status == VL53L0X_ERROR_NONE) {
        if ((sigma_limit_check_enable == 0) || (sigma_limitflag == 1)) {
            temp8 = 1;
        } else {
            temp8 = 0;
        }
        VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksStatus,
                                       VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, temp8);

        if ((device_range_status_internal == 4) ||
                (signal_rate_final_range_limit_check_enable == 0)) {
            temp8 = 1;
        } else {
            temp8 = 0;
        }
        VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksStatus,
                                       VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,
                                       temp8);

        if ((signal_ref_clip_limit_check_enable == 0) ||
                (signal_ref_clipflag == 1)) {
            temp8 = 1;
        } else {
            temp8 = 0;
        }

        VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksStatus,
                                       VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP, temp8);

        if ((range_ignore_threshold_limit_check_enable == 0) ||
                (range_ignore_thresholdflag == 1)) {
            temp8 = 1;
        } else {
            temp8 = 0;
        }

        VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksStatus,
                                       VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                       temp8);
    }

    LOG_FUNCTION_END(status);
    return status;

}

VL53L0X_Error VL53L0X::VL53L0X_get_ranging_measurement_data(VL53L0X_DEV dev,
        VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t device_range_status;
    uint8_t range_fractional_enable;
    uint8_t pal_range_status;
    uint8_t x_talk_compensation_enable;
    uint16_t ambient_rate;
    FixPoint1616_t signal_rate;
    uint16_t x_talk_compensation_rate_mega_cps;
    uint16_t effective_spad_rtn_count;
    uint16_t tmpuint16;
    uint16_t xtalk_range_milli_meter;
    uint16_t linearity_corrective_gain;
    uint8_t localBuffer[12];
    VL53L0X_RangingMeasurementData_t last_range_data_buffer;

    LOG_FUNCTION_START("");

    /*
     * use multi read even if some registers are not useful, result will
     * be more efficient
     * start reading at 0x14 dec20
     * end reading at 0x21 dec33 total 14 bytes to read
     */
    status = VL53L0X_read_multi(dev, 0x14, localBuffer, 12);

    if (status == VL53L0X_ERROR_NONE) {

        p_ranging_measurement_data->ZoneId = 0; /* Only one zone */
        p_ranging_measurement_data->TimeStamp = 0; /* Not Implemented */

        tmpuint16 = VL53L0X_MAKEUINT16(localBuffer[11], localBuffer[10]);
        /* cut1.1 if SYSTEM__RANGE_CONFIG if 1 range is 2bits fractional
         *(format 11.2) else no fractional
         */

        p_ranging_measurement_data->MeasurementTimeUsec = 0;

        signal_rate = VL53L0X_FIXPOINT97TOFIXPOINT1616(
                          VL53L0X_MAKEUINT16(localBuffer[7], localBuffer[6]));
        /* peak_signal_count_rate_rtn_mcps */
        p_ranging_measurement_data->SignalRateRtnMegaCps = signal_rate;

        ambient_rate = VL53L0X_MAKEUINT16(localBuffer[9], localBuffer[8]);
        p_ranging_measurement_data->AmbientRateRtnMegaCps =
            VL53L0X_FIXPOINT97TOFIXPOINT1616(ambient_rate);

        effective_spad_rtn_count = VL53L0X_MAKEUINT16(localBuffer[3],
                                   localBuffer[2]);
        /* EffectiveSpadRtnCount is 8.8 format */
        p_ranging_measurement_data->EffectiveSpadRtnCount =
            effective_spad_rtn_count;

        device_range_status = localBuffer[0];

        /* Get Linearity Corrective Gain */
        linearity_corrective_gain = PALDevDataGet(dev,
                                    LinearityCorrectiveGain);

        /* Get ranging configuration */
        range_fractional_enable = PALDevDataGet(dev,
                                                RangeFractionalEnable);

        if (linearity_corrective_gain != 1000) {

            tmpuint16 = (uint16_t)((linearity_corrective_gain
                                    * tmpuint16 + 500) / 1000);

            /* Implement Xtalk */
            VL53L0X_GETPARAMETERFIELD(dev,
                                      XTalkCompensationRateMegaCps,
                                      x_talk_compensation_rate_mega_cps);
            VL53L0X_GETPARAMETERFIELD(dev, XTalkCompensationEnable,
                                      x_talk_compensation_enable);

            if (x_talk_compensation_enable) {

                if ((signal_rate
                        - ((x_talk_compensation_rate_mega_cps
                            * effective_spad_rtn_count) >> 8))
                        <= 0) {
                    if (range_fractional_enable) {
                        xtalk_range_milli_meter = 8888;
                    } else {
                        xtalk_range_milli_meter = 8888 << 2;
                    }
                } else {
                    xtalk_range_milli_meter =
                        (tmpuint16 * signal_rate)
                        / (signal_rate
                           - ((x_talk_compensation_rate_mega_cps
                               * effective_spad_rtn_count)
                              >> 8));
                }

                tmpuint16 = xtalk_range_milli_meter;
            }

        }

        if (range_fractional_enable) {
            p_ranging_measurement_data->RangeMilliMeter =
                (uint16_t)((tmpuint16) >> 2);
            p_ranging_measurement_data->RangeFractionalPart =
                (uint8_t)((tmpuint16 & 0x03) << 6);
        } else {
            p_ranging_measurement_data->RangeMilliMeter = tmpuint16;
            p_ranging_measurement_data->RangeFractionalPart = 0;
        }

        /*
         * For a standard definition of RangeStatus, this should
         * return 0 in case of good result after a ranging
         * The range status depends on the device so call a device
         * specific function to obtain the right Status.
         */
        status |= VL53L0X_get_pal_range_status(dev, device_range_status,
                                               signal_rate, effective_spad_rtn_count,
                                               p_ranging_measurement_data, &pal_range_status);

        if (status == VL53L0X_ERROR_NONE) {
            p_ranging_measurement_data->RangeStatus = pal_range_status;
        }

    }

    if (status == VL53L0X_ERROR_NONE) {
        /* Copy last read data into Dev buffer */
        last_range_data_buffer = PALDevDataGet(dev, LastRangeMeasure);

        last_range_data_buffer.RangeMilliMeter =
            p_ranging_measurement_data->RangeMilliMeter;
        last_range_data_buffer.RangeFractionalPart =
            p_ranging_measurement_data->RangeFractionalPart;
        last_range_data_buffer.RangeDMaxMilliMeter =
            p_ranging_measurement_data->RangeDMaxMilliMeter;
        last_range_data_buffer.MeasurementTimeUsec =
            p_ranging_measurement_data->MeasurementTimeUsec;
        last_range_data_buffer.SignalRateRtnMegaCps =
            p_ranging_measurement_data->SignalRateRtnMegaCps;
        last_range_data_buffer.AmbientRateRtnMegaCps =
            p_ranging_measurement_data->AmbientRateRtnMegaCps;
        last_range_data_buffer.EffectiveSpadRtnCount =
            p_ranging_measurement_data->EffectiveSpadRtnCount;
        last_range_data_buffer.RangeStatus =
            p_ranging_measurement_data->RangeStatus;

        PALDevDataSet(dev, LastRangeMeasure, last_range_data_buffer);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_single_ranging_measurement(VL53L0X_DEV dev,
        VL53L0X_RangingMeasurementData_t *p_ranging_measurement_data)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    LOG_FUNCTION_START("");

    /* This function will do a complete single ranging
     * Here we fix the mode! */
    status = VL53L0X_set_device_mode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_perform_single_measurement(dev);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_ranging_measurement_data(dev,
                 p_ranging_measurement_data);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_clear_interrupt_mask(dev, 0);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::perform_ref_signal_measurement(VL53L0X_DEV dev,
        uint16_t *p_ref_signal_rate)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t ranging_measurement_data;

    uint8_t sequence_config = 0;

    /* store the value of the sequence config,
     * this will be reset before the end of the function
     */

    sequence_config = PALDevDataGet(dev, SequenceConfig);

    /*
     * This function performs a reference signal rate measurement.
     */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xC0);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_perform_single_ranging_measurement(dev,
                 &ranging_measurement_data);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0xFF, 0x01);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_word(dev,
                                   VL53L0X_REG_RESULT_PEAK_SIGNAL_RATE_REF,
                                   p_ref_signal_rate);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0xFF, 0x00);
    }

    if (status == VL53L0X_ERROR_NONE) {
        /* restore the previous Sequence Config */
        status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                                    sequence_config);
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, SequenceConfig, sequence_config);
        }
    }

    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_perform_ref_spad_management(VL53L0X_DEV dev,
        uint32_t *ref_spad_count,
        uint8_t *is_aperture_spads)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t last_spad_array[6];
    uint8_t start_select = 0xB4;
    uint32_t minimum_spad_count = 3;
    uint32_t max_spad_count = 44;
    uint32_t current_spad_index = 0;
    uint32_t last_spad_index = 0;
    int32_t next_good_spad = 0;
    uint16_t target_ref_rate = 0x0A00; /* 20 MCPS in 9:7 format */
    uint16_t peak_signal_rate_ref;
    uint32_t need_apt_spads = 0;
    uint32_t index = 0;
    uint32_t spad_array_size = 6;
    uint32_t signal_rate_diff = 0;
    uint32_t last_signal_rate_diff = 0;
    uint8_t complete = 0;
    uint8_t vhv_settings = 0;
    uint8_t phase_cal = 0;
    uint32_t ref_spad_count_int = 0;
    uint8_t	 is_aperture_spads_int = 0;

    /*
     * The reference SPAD initialization procedure determines the minimum
     * amount of reference spads to be enables to achieve a target reference
     * signal rate and should be performed once during initialization.
     *
     * Either aperture or non-aperture spads are applied but never both.
     * Firstly non-aperture spads are set, begining with 5 spads, and
     * increased one spad at a time until the closest measurement to the
     * target rate is achieved.
     *
     * If the target rate is exceeded when 5 non-aperture spads are enabled,
     * initialization is performed instead with aperture spads.
     *
     * When setting spads, a 'Good Spad Map' is applied.
     *
     * This procedure operates within a SPAD window of interest of a maximum
     * 44 spads.
     * The start point is currently fixed to 180, which lies towards the end
     * of the non-aperture quadrant and runs in to the adjacent aperture
     * quadrant.
     */
    target_ref_rate = PALDevDataGet(dev, targetRefRate);

    /*
     * Initialize Spad arrays.
     * Currently the good spad map is initialised to 'All good'.
     * This is a short term implementation. The good spad map will be
     * provided as an input.
     * Note that there are 6 bytes. Only the first 44 bits will be used to
     * represent spads.
     */
    for (index = 0; index < spad_array_size; index++) {
        dev->Data.SpadData.RefSpadEnables[index] = 0;
    }


    status = VL53L0X_write_byte(dev, 0xFF, 0x01);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0xFF, 0x00);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT,
                                    start_select);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_POWER_MANAGEMENT_GO1_POWER_FORCE, 0);
    }

    /* Perform ref calibration */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_perform_ref_calibration(dev, &vhv_settings,
                 &phase_cal, 0);
    }

    if (status == VL53L0X_ERROR_NONE) {
        /* Enable Minimum NON-APERTURE Spads */
        current_spad_index = 0;
        last_spad_index = current_spad_index;
        need_apt_spads = 0;
        status = enable_ref_spads(dev,
                                  need_apt_spads,
                                  dev->Data.SpadData.RefGoodSpadMap,
                                  dev->Data.SpadData.RefSpadEnables,
                                  spad_array_size,
                                  start_select,
                                  current_spad_index,
                                  minimum_spad_count,
                                  &last_spad_index);
    }

    if (status == VL53L0X_ERROR_NONE) {
        current_spad_index = last_spad_index;

        status = perform_ref_signal_measurement(dev,
                                                &peak_signal_rate_ref);
        if ((status == VL53L0X_ERROR_NONE) &&
                (peak_signal_rate_ref > target_ref_rate)) {
            /* Signal rate measurement too high,
             * switch to APERTURE SPADs */

            for (index = 0; index < spad_array_size; index++) {
                dev->Data.SpadData.RefSpadEnables[index] = 0;
            }


            /* Increment to the first APERTURE spad */
            while ((is_aperture(start_select + current_spad_index)
                    == 0) && (current_spad_index < max_spad_count)) {
                current_spad_index++;
            }

            need_apt_spads = 1;

            status = enable_ref_spads(dev,
                                      need_apt_spads,
                                      dev->Data.SpadData.RefGoodSpadMap,
                                      dev->Data.SpadData.RefSpadEnables,
                                      spad_array_size,
                                      start_select,
                                      current_spad_index,
                                      minimum_spad_count,
                                      &last_spad_index);

            if (status == VL53L0X_ERROR_NONE) {
                current_spad_index = last_spad_index;
                status = perform_ref_signal_measurement(dev,
                                                        &peak_signal_rate_ref);

                if ((status == VL53L0X_ERROR_NONE) &&
                        (peak_signal_rate_ref > target_ref_rate)) {
                    /* Signal rate still too high after
                     * setting the minimum number of
                     * APERTURE spads. Can do no more
                     * therefore set the min number of
                     * aperture spads as the result.
                     */
                    is_aperture_spads_int = 1;
                    ref_spad_count_int = minimum_spad_count;
                }
            }
        } else {
            need_apt_spads = 0;
        }
    }

    if ((status == VL53L0X_ERROR_NONE) &&
            (peak_signal_rate_ref < target_ref_rate)) {
        /* At this point, the minimum number of either aperture
         * or non-aperture spads have been set. Proceed to add
         * spads and perform measurements until the target
         * reference is reached.
         */
        is_aperture_spads_int = need_apt_spads;
        ref_spad_count_int	= minimum_spad_count;

        memcpy(last_spad_array, dev->Data.SpadData.RefSpadEnables,
               spad_array_size);
        last_signal_rate_diff = abs(peak_signal_rate_ref -
                                    target_ref_rate);
        complete = 0;

        while (!complete) {
            get_next_good_spad(
                dev->Data.SpadData.RefGoodSpadMap,
                spad_array_size, current_spad_index,
                &next_good_spad);

            if (next_good_spad == -1) {
                status = VL53L0X_ERROR_REF_SPAD_INIT;
                break;
            }

            /* Cannot combine Aperture and Non-Aperture spads, so
             * ensure the current spad is of the correct type.
             */
            if (is_aperture((uint32_t)start_select + next_good_spad) !=
                    need_apt_spads) {
                /* At this point we have enabled the maximum
                 * number of Aperture spads.
                 */
                complete = 1;
                break;
            }

            (ref_spad_count_int)++;

            current_spad_index = next_good_spad;
            status = enable_spad_bit(
                         dev->Data.SpadData.RefSpadEnables,
                         spad_array_size, current_spad_index);

            if (status == VL53L0X_ERROR_NONE) {
                current_spad_index++;
                /* Proceed to apply the additional spad and
                 * perform measurement. */
                status = set_ref_spad_map(dev,
                                          dev->Data.SpadData.RefSpadEnables);
            }

            if (status != VL53L0X_ERROR_NONE) {
                break;
            }

            status = perform_ref_signal_measurement(dev,
                                                    &peak_signal_rate_ref);

            if (status != VL53L0X_ERROR_NONE) {
                break;
            }

            signal_rate_diff = abs(peak_signal_rate_ref - target_ref_rate);

            if (peak_signal_rate_ref > target_ref_rate) {
                /* Select the spad map that provides the
                 * measurement closest to the target rate,
                 * either above or below it.
                 */
                if (signal_rate_diff > last_signal_rate_diff) {
                    /* Previous spad map produced a closer
                     * measurement, so choose this. */
                    status = set_ref_spad_map(dev,
                                              last_spad_array);
                    memcpy(
                        dev->Data.SpadData.RefSpadEnables,
                        last_spad_array, spad_array_size);

                    (ref_spad_count_int)--;
                }
                complete = 1;
            } else {
                /* Continue to add spads */
                last_signal_rate_diff = signal_rate_diff;
                memcpy(last_spad_array,
                       dev->Data.SpadData.RefSpadEnables,
                       spad_array_size);
            }

        } /* while */
    }

    if (status == VL53L0X_ERROR_NONE) {
        *ref_spad_count = ref_spad_count_int;
        *is_aperture_spads = is_aperture_spads_int;

        VL53L0X_SETDEVICESPECIFICPARAMETER(dev, RefSpadsInitialised, 1);
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                           ReferenceSpadCount, (uint8_t)(*ref_spad_count));
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                           ReferenceSpadType, *is_aperture_spads);
    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_reference_spads(VL53L0X_DEV dev,
        uint32_t count, uint8_t is_aperture_spads)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t current_spad_index = 0;
    uint8_t start_select = 0xB4;
    uint32_t spad_array_size = 6;
    uint32_t max_spad_count = 44;
    uint32_t last_spad_index;
    uint32_t index;

    /*
     * This function applies a requested number of reference spads, either
     * aperture or
     * non-aperture, as requested.
     * The good spad map will be applied.
     */

    status = VL53L0X_write_byte(dev, 0xFF, 0x01);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0xFF, 0x00);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT,
                                    start_select);
    }

    for (index = 0; index < spad_array_size; index++) {
        dev->Data.SpadData.RefSpadEnables[index] = 0;
    }

    if (is_aperture_spads) {
        /* Increment to the first APERTURE spad */
        while ((is_aperture(start_select + current_spad_index) == 0) &&
                (current_spad_index < max_spad_count)) {
            current_spad_index++;
        }
    }
    status = enable_ref_spads(dev,
                              is_aperture_spads,
                              dev->Data.SpadData.RefGoodSpadMap,
                              dev->Data.SpadData.RefSpadEnables,
                              spad_array_size,
                              start_select,
                              current_spad_index,
                              count,
                              &last_spad_index);

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev, RefSpadsInitialised, 1);
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                           ReferenceSpadCount, (uint8_t)(count));
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                           ReferenceSpadType, is_aperture_spads);
    }

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_wait_device_booted(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NOT_IMPLEMENTED;
    LOG_FUNCTION_START("");

    /* not implemented on VL53L0X */

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_ref_calibration(VL53L0X_DEV dev, uint8_t *p_vhv_settings,
        uint8_t *p_phase_cal)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = VL53L0X_perform_ref_calibration(dev, p_vhv_settings,
             p_phase_cal, 1);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_perform_ref_spad_management(VL53L0X_DEV dev,
        uint32_t *ref_spad_count, uint8_t *is_aperture_spads)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_perform_ref_spad_management(dev, ref_spad_count,
             is_aperture_spads);

    LOG_FUNCTION_END(status);

    return status;
}

/* Group PAL Init Functions */
VL53L0X_Error VL53L0X::VL53L0X_set_device_address(VL53L0X_DEV dev, uint8_t device_address)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = VL53L0X_write_byte(dev, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS,
                                device_address / 2);

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_gpio_config(VL53L0X_DEV dev, uint8_t pin,
        VL53L0X_DeviceModes device_mode, VL53L0X_GpioFunctionality functionality,
        VL53L0X_InterruptPolarity polarity)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t data;

    LOG_FUNCTION_START("");

    if (pin != 0) {
        status = VL53L0X_ERROR_GPIO_NOT_EXISTING;
    } else if (device_mode == VL53L0X_DEVICEMODE_GPIO_DRIVE) {
        if (polarity == VL53L0X_INTERRUPTPOLARITY_LOW) {
            data = 0x10;
        } else {
            data = 1;
        }

        status = VL53L0X_write_byte(dev,
                                    VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, data);

    } else {
        if (device_mode == VL53L0X_DEVICEMODE_GPIO_OSC) {

            status |= VL53L0X_write_byte(dev, 0xff, 0x01);
            status |= VL53L0X_write_byte(dev, 0x00, 0x00);

            status |= VL53L0X_write_byte(dev, 0xff, 0x00);
            status |= VL53L0X_write_byte(dev, 0x80, 0x01);
            status |= VL53L0X_write_byte(dev, 0x85, 0x02);

            status |= VL53L0X_write_byte(dev, 0xff, 0x04);
            status |= VL53L0X_write_byte(dev, 0xcd, 0x00);
            status |= VL53L0X_write_byte(dev, 0xcc, 0x11);

            status |= VL53L0X_write_byte(dev, 0xff, 0x07);
            status |= VL53L0X_write_byte(dev, 0xbe, 0x00);

            status |= VL53L0X_write_byte(dev, 0xff, 0x06);
            status |= VL53L0X_write_byte(dev, 0xcc, 0x09);

            status |= VL53L0X_write_byte(dev, 0xff, 0x00);
            status |= VL53L0X_write_byte(dev, 0xff, 0x01);
            status |= VL53L0X_write_byte(dev, 0x00, 0x00);

        } else {

            if (status == VL53L0X_ERROR_NONE) {
                switch (functionality) {
                    case VL53L0X_GPIOFUNCTIONALITY_OFF:
                        data = 0x00;
                        break;
                    case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_LOW:
                        data = 0x01;
                        break;
                    case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_HIGH:
                        data = 0x02;
                        break;
                    case VL53L0X_GPIOFUNCTIONALITY_THRESHOLD_CROSSED_OUT:
                        data = 0x03;
                        break;
                    case VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY:
                        data = 0x04;
                        break;
                    default:
                        status =
                            VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED;
                }
            }

            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_write_byte(dev,
                                            VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, data);
            }

            if (status == VL53L0X_ERROR_NONE) {
                if (polarity == VL53L0X_INTERRUPTPOLARITY_LOW) {
                    data = 0;
                } else {
                    data = (uint8_t)(1 << 4);
                }
                status = VL53L0X_update_byte(dev,
                                             VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH, 0xEF, data);
            }

            if (status == VL53L0X_ERROR_NONE) {
                VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                                   Pin0GpioFunctionality, functionality);
            }

            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_clear_interrupt_mask(dev, 0);
            }
        }
    }
    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_fraction_enable(VL53L0X_DEV dev, uint8_t *p_enabled)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = VL53L0X_read_byte(dev, VL53L0X_REG_SYSTEM_RANGE_CONFIG, p_enabled);

    if (status == VL53L0X_ERROR_NONE) {
        *p_enabled = (*p_enabled & 1);
    }

    LOG_FUNCTION_END(status);
    return status;
}

uint16_t VL53L0X::VL53L0X_encode_timeout(uint32_t timeout_macro_clks)
{
    /*!
     * Encode timeout in macro periods in (LSByte * 2^MSByte) + 1 format
     */

    uint16_t encoded_timeout = 0;
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_macro_clks > 0) {
        ls_byte = timeout_macro_clks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte = ls_byte >> 1;
            ms_byte++;
        }

        encoded_timeout = (ms_byte << 8)
                          + (uint16_t)(ls_byte & 0x000000FF);
    }

    return encoded_timeout;

}

VL53L0X_Error VL53L0X::set_sequence_step_timeout(VL53L0X_DEV dev,
        VL53L0X_SequenceStepId sequence_step_id,
        uint32_t timeout_micro_secs)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t current_vcsel_pulse_period_p_clk;
    uint8_t msrc_encoded_time_out;
    uint16_t pre_range_encoded_time_out;
    uint16_t pre_range_time_out_m_clks;
    uint16_t msrc_range_time_out_m_clks;
    uint32_t final_range_time_out_m_clks;
    uint16_t final_range_encoded_time_out;
    VL53L0X_SchedulerSequenceSteps_t scheduler_sequence_steps;

    if ((sequence_step_id == VL53L0X_SEQUENCESTEP_TCC)	 ||
            (sequence_step_id == VL53L0X_SEQUENCESTEP_DSS)	 ||
            (sequence_step_id == VL53L0X_SEQUENCESTEP_MSRC)) {

        status = VL53L0X_get_vcsel_pulse_period(dev,
                                                VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                &current_vcsel_pulse_period_p_clk);

        if (status == VL53L0X_ERROR_NONE) {
            msrc_range_time_out_m_clks = VL53L0X_calc_timeout_mclks(dev,
                                         timeout_micro_secs,
                                         (uint8_t)current_vcsel_pulse_period_p_clk);

            if (msrc_range_time_out_m_clks > 256) {
                msrc_encoded_time_out = 255;
            } else {
                msrc_encoded_time_out =
                    (uint8_t)msrc_range_time_out_m_clks - 1;
            }

            VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                               LastEncodedTimeout,
                                               msrc_encoded_time_out);
        }

        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP,
                                        msrc_encoded_time_out);
        }
    } else {

        if (sequence_step_id == VL53L0X_SEQUENCESTEP_PRE_RANGE) {

            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_get_vcsel_pulse_period(dev,
                                                        VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                        &current_vcsel_pulse_period_p_clk);
                pre_range_time_out_m_clks =
                    VL53L0X_calc_timeout_mclks(dev,
                                               timeout_micro_secs,
                                               (uint8_t)current_vcsel_pulse_period_p_clk);
                pre_range_encoded_time_out = VL53L0X_encode_timeout(
                                                 pre_range_time_out_m_clks);

                VL53L0X_SETDEVICESPECIFICPARAMETER(dev,
                                                   LastEncodedTimeout,
                                                   pre_range_encoded_time_out);
            }

            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_write_word(dev,
                                            VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                                            pre_range_encoded_time_out);
            }

            if (status == VL53L0X_ERROR_NONE) {
                VL53L0X_SETDEVICESPECIFICPARAMETER(
                    dev,
                    PreRangeTimeoutMicroSecs,
                    timeout_micro_secs);
            }
        } else if (sequence_step_id == VL53L0X_SEQUENCESTEP_FINAL_RANGE) {

            /* For the final range timeout, the pre-range timeout
             * must be added. To do this both final and pre-range
             * timeouts must be expressed in macro periods MClks
             * because they have different vcsel periods.
             */

            VL53L0X_get_sequence_step_enables(dev,
                                              &scheduler_sequence_steps);
            pre_range_time_out_m_clks = 0;
            if (scheduler_sequence_steps.PreRangeOn) {

                /* Retrieve PRE-RANGE VCSEL Period */
                status = VL53L0X_get_vcsel_pulse_period(dev,
                                                        VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                                                        &current_vcsel_pulse_period_p_clk);

                /* Retrieve PRE-RANGE Timeout in Macro periods
                 * (MCLKS) */
                if (status == VL53L0X_ERROR_NONE) {
                    status = VL53L0X_read_word(dev, 0x51,
                                               &pre_range_encoded_time_out);
                    pre_range_time_out_m_clks =
                        VL53L0X_decode_timeout(
                            pre_range_encoded_time_out);
                }
            }

            /* Calculate FINAL RANGE Timeout in Macro Periods
             * (MCLKS) and add PRE-RANGE value
             */
            if (status == VL53L0X_ERROR_NONE) {
                status = VL53L0X_get_vcsel_pulse_period(dev,
                                                        VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                                                        &current_vcsel_pulse_period_p_clk);
            }
            if (status == VL53L0X_ERROR_NONE) {
                final_range_time_out_m_clks =
                    VL53L0X_calc_timeout_mclks(dev,
                                               timeout_micro_secs,
                                               (uint8_t) current_vcsel_pulse_period_p_clk);

                final_range_time_out_m_clks += pre_range_time_out_m_clks;

                final_range_encoded_time_out =
                    VL53L0X_encode_timeout(final_range_time_out_m_clks);

                if (status == VL53L0X_ERROR_NONE) {
                    status = VL53L0X_write_word(dev, 0x71,
                                                final_range_encoded_time_out);
                }

                if (status == VL53L0X_ERROR_NONE) {
                    VL53L0X_SETDEVICESPECIFICPARAMETER(
                        dev,
                        FinalRangeTimeoutMicroSecs,
                        timeout_micro_secs);
                }
            }
        } else {
            status = VL53L0X_ERROR_INVALID_PARAMS;
        }

    }
    return status;
}

VL53L0X_Error VL53L0X::wrapped_VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
        uint32_t measurement_timing_budget_micro_seconds)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t final_range_timing_budget_micro_seconds;
    VL53L0X_SchedulerSequenceSteps_t scheduler_sequence_steps;
    uint32_t msrc_dcc_tcc_timeout_micro_seconds	= 2000;
    uint32_t start_overhead_micro_seconds		= 1910;
    uint32_t end_overhead_micro_seconds		= 960;
    uint32_t msrc_overhead_micro_seconds		= 660;
    uint32_t tcc_overhead_micro_seconds		= 590;
    uint32_t dss_overhead_micro_seconds		= 690;
    uint32_t pre_range_overhead_micro_seconds	= 660;
    uint32_t final_range_overhead_micro_seconds = 550;
    uint32_t pre_range_timeout_micro_seconds	= 0;
    uint32_t c_min_timing_budget_micro_seconds	= 20000;
    uint32_t sub_timeout = 0;

    LOG_FUNCTION_START("");

    if (measurement_timing_budget_micro_seconds
            < c_min_timing_budget_micro_seconds) {
        status = VL53L0X_ERROR_INVALID_PARAMS;
        return status;
    }

    final_range_timing_budget_micro_seconds =
        measurement_timing_budget_micro_seconds -
        (start_overhead_micro_seconds + end_overhead_micro_seconds);

    status = VL53L0X_get_sequence_step_enables(dev, &scheduler_sequence_steps);

    if (status == VL53L0X_ERROR_NONE &&
            (scheduler_sequence_steps.TccOn  ||
             scheduler_sequence_steps.MsrcOn ||
             scheduler_sequence_steps.DssOn)) {

        /* TCC, MSRC and DSS all share the same timeout */
        status = get_sequence_step_timeout(dev,
                                           VL53L0X_SEQUENCESTEP_MSRC,
                                           &msrc_dcc_tcc_timeout_micro_seconds);

        /* Subtract the TCC, MSRC and DSS timeouts if they are
         * enabled. */

        if (status != VL53L0X_ERROR_NONE) {
            return status;
        }

        /* TCC */
        if (scheduler_sequence_steps.TccOn) {

            sub_timeout = msrc_dcc_tcc_timeout_micro_seconds
                          + tcc_overhead_micro_seconds;

            if (sub_timeout <
                    final_range_timing_budget_micro_seconds) {
                final_range_timing_budget_micro_seconds -=
                    sub_timeout;
            } else {
                /* Requested timeout too big. */
                status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        }

        if (status != VL53L0X_ERROR_NONE) {
            LOG_FUNCTION_END(status);
            return status;
        }

        /* DSS */
        if (scheduler_sequence_steps.DssOn) {

            sub_timeout = 2 * (msrc_dcc_tcc_timeout_micro_seconds +
                               dss_overhead_micro_seconds);

            if (sub_timeout < final_range_timing_budget_micro_seconds) {
                final_range_timing_budget_micro_seconds
                -= sub_timeout;
            } else {
                /* Requested timeout too big. */
                status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        } else if (scheduler_sequence_steps.MsrcOn) {
            /* MSRC */
            sub_timeout = msrc_dcc_tcc_timeout_micro_seconds +
                          msrc_overhead_micro_seconds;

            if (sub_timeout < final_range_timing_budget_micro_seconds) {
                final_range_timing_budget_micro_seconds
                -= sub_timeout;
            } else {
                /* Requested timeout too big. */
                status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        }

    }

    if (status != VL53L0X_ERROR_NONE) {
        LOG_FUNCTION_END(status);
        return status;
    }

    if (scheduler_sequence_steps.PreRangeOn) {

        /* Subtract the Pre-range timeout if enabled. */

        status = get_sequence_step_timeout(dev,
                                           VL53L0X_SEQUENCESTEP_PRE_RANGE,
                                           &pre_range_timeout_micro_seconds);

        sub_timeout = pre_range_timeout_micro_seconds +
                      pre_range_overhead_micro_seconds;

        if (sub_timeout < final_range_timing_budget_micro_seconds) {
            final_range_timing_budget_micro_seconds -= sub_timeout;
        } else {
            /* Requested timeout too big. */
            status = VL53L0X_ERROR_INVALID_PARAMS;
        }
    }


    if (status == VL53L0X_ERROR_NONE &&
            scheduler_sequence_steps.FinalRangeOn) {

        final_range_timing_budget_micro_seconds -=
            final_range_overhead_micro_seconds;

        /* Final Range Timeout
         * Note that the final range timeout is determined by the timing
         * budget and the sum of all other timeouts within the sequence.
         * If there is no room for the final range timeout, then an error
         * will be set. Otherwise the remaining time will be applied to
         * the final range.
         */
        status = set_sequence_step_timeout(dev,
                                           VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                                           final_range_timing_budget_micro_seconds);

        VL53L0X_SETPARAMETERFIELD(dev,
                                  MeasurementTimingBudgetMicroSeconds,
                                  measurement_timing_budget_micro_seconds);
    }

    LOG_FUNCTION_END(status);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_measurement_timing_budget_micro_seconds(VL53L0X_DEV dev,
        uint32_t measurement_timing_budget_micro_seconds)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = wrapped_VL53L0X_set_measurement_timing_budget_micro_seconds(dev,
             measurement_timing_budget_micro_seconds);

    LOG_FUNCTION_END(status);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_sequence_step_enable(VL53L0X_DEV dev,
        VL53L0X_SequenceStepId sequence_step_id, uint8_t sequence_step_enabled)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t sequence_config = 0;
    uint8_t sequence_config_new = 0;
    uint32_t measurement_timing_budget_micro_seconds;
    LOG_FUNCTION_START("");

    status = VL53L0X_read_byte(dev, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG,
                               &sequence_config);

    sequence_config_new = sequence_config;

    if (status == VL53L0X_ERROR_NONE) {
        if (sequence_step_enabled == 1) {

            /* Enable requested sequence step
             */
            switch (sequence_step_id) {
                case VL53L0X_SEQUENCESTEP_TCC:
                    sequence_config_new |= 0x10;
                    break;
                case VL53L0X_SEQUENCESTEP_DSS:
                    sequence_config_new |= 0x28;
                    break;
                case VL53L0X_SEQUENCESTEP_MSRC:
                    sequence_config_new |= 0x04;
                    break;
                case VL53L0X_SEQUENCESTEP_PRE_RANGE:
                    sequence_config_new |= 0x40;
                    break;
                case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
                    sequence_config_new |= 0x80;
                    break;
                default:
                    status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        } else {
            /* Disable requested sequence step
             */
            switch (sequence_step_id) {
                case VL53L0X_SEQUENCESTEP_TCC:
                    sequence_config_new &= 0xef;
                    break;
                case VL53L0X_SEQUENCESTEP_DSS:
                    sequence_config_new &= 0xd7;
                    break;
                case VL53L0X_SEQUENCESTEP_MSRC:
                    sequence_config_new &= 0xfb;
                    break;
                case VL53L0X_SEQUENCESTEP_PRE_RANGE:
                    sequence_config_new &= 0xbf;
                    break;
                case VL53L0X_SEQUENCESTEP_FINAL_RANGE:
                    sequence_config_new &= 0x7f;
                    break;
                default:
                    status = VL53L0X_ERROR_INVALID_PARAMS;
            }
        }
    }

    if (sequence_config_new != sequence_config) {
        /* Apply New Setting */
        if (status == VL53L0X_ERROR_NONE) {
            status = VL53L0X_write_byte(dev,
                                        VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, sequence_config_new);
        }
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, SequenceConfig, sequence_config_new);
        }


        /* Recalculate timing budget */
        if (status == VL53L0X_ERROR_NONE) {
            VL53L0X_GETPARAMETERFIELD(dev,
                                      MeasurementTimingBudgetMicroSeconds,
                                      measurement_timing_budget_micro_seconds);

            VL53L0X_set_measurement_timing_budget_micro_seconds(dev,
                    measurement_timing_budget_micro_seconds);
        }
    }

    LOG_FUNCTION_END(status);

    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_set_limit_check_enable(VL53L0X_DEV dev, uint16_t limit_check_id,
        uint8_t limit_check_enable)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    FixPoint1616_t temp_fix1616 = 0;
    uint8_t limit_check_enable_int = 0;
    uint8_t limit_check_disable = 0;
    uint8_t temp8;

    LOG_FUNCTION_START("");

    if (limit_check_id >= VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS) {
        status = VL53L0X_ERROR_INVALID_PARAMS;
    } else {
        if (limit_check_enable == 0) {
            temp_fix1616 = 0;
            limit_check_enable_int = 0;
            limit_check_disable = 1;

        } else {
            VL53L0X_GETARRAYPARAMETERFIELD(dev, LimitChecksValue,
                                           limit_check_id, temp_fix1616);
            limit_check_disable = 0;
            /* this to be sure to have either 0 or 1 */
            limit_check_enable_int = 1;
        }

        switch (limit_check_id) {

            case VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE:
                /* internal computation: */
                VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksEnable,
                                               VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,
                                               limit_check_enable_int);

                break;

            case VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE:

                status = VL53L0X_write_word(dev,
                                            VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                                            VL53L0X_FIXPOINT1616TOFIXPOINT97(temp_fix1616));

                break;

            case VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP:

                /* internal computation: */
                VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksEnable,
                                               VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP,
                                               limit_check_enable_int);

                break;

            case VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD:

                /* internal computation: */
                VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksEnable,
                                               VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                               limit_check_enable_int);

                break;

            case VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC:

                temp8 = (uint8_t)(limit_check_disable << 1);
                status = VL53L0X_update_byte(dev,
                                             VL53L0X_REG_MSRC_CONFIG_CONTROL,
                                             0xFE, temp8);

                break;

            case VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE:

                temp8 = (uint8_t)(limit_check_disable << 4);
                status = VL53L0X_update_byte(dev,
                                             VL53L0X_REG_MSRC_CONFIG_CONTROL,
                                             0xEF, temp8);

                break;


            default:
                status = VL53L0X_ERROR_INVALID_PARAMS;

        }

    }

    if (status == VL53L0X_ERROR_NONE) {
        if (limit_check_enable == 0) {
            VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksEnable,
                                           limit_check_id, 0);
        } else {
            VL53L0X_SETARRAYPARAMETERFIELD(dev, LimitChecksEnable,
                                           limit_check_id, 1);
        }
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_static_init(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    VL53L0X_DeviceParameters_t current_parameters = {0};
    uint8_t *p_tuning_setting_buffer;
    uint16_t tempword = 0;
    uint8_t tempbyte = 0;
    uint8_t use_internal_tuning_settings = 0;
    uint32_t count = 0;
    uint8_t is_aperture_spads = 0;
    uint32_t ref_spad_count = 0;
    uint8_t aperture_spads = 0;
    uint8_t vcsel_pulse_period_pclk;
    uint32_t seq_timeout_micro_secs;

    LOG_FUNCTION_START("");

    status = VL53L0X_get_info_from_device(dev, 1);

    /* set the ref spad from NVM */
    count	= (uint32_t)VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
              ReferenceSpadCount);
    aperture_spads = VL53L0X_GETDEVICESPECIFICPARAMETER(dev,
                     ReferenceSpadType);

    /* NVM value invalid */
    if ((aperture_spads > 1) ||
            ((aperture_spads == 1) && (count > 32)) ||
            ((aperture_spads == 0) && (count > 12))) {
        status = wrapped_VL53L0X_perform_ref_spad_management(dev, &ref_spad_count,
                 &is_aperture_spads);
    } else {
        status = VL53L0X_set_reference_spads(dev, count, aperture_spads);
    }


    /* Initialize tuning settings buffer to prevent compiler warning. */
    p_tuning_setting_buffer = DefaultTuningSettings;

    if (status == VL53L0X_ERROR_NONE) {
        use_internal_tuning_settings = PALDevDataGet(dev,
                                       UseInternalTuningSettings);

        if (use_internal_tuning_settings == 0) {
            p_tuning_setting_buffer = PALDevDataGet(dev,
                                                    pTuningSettingsPointer);
        } else {
            p_tuning_setting_buffer = DefaultTuningSettings;
        }

    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_load_tuning_settings(dev, p_tuning_setting_buffer);
    }


    /* Set interrupt config to new sample ready */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_gpio_config(dev, 0, 0,
                                         VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
                                         VL53L0X_INTERRUPTPOLARITY_LOW);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0xFF, 0x01);
        status |= VL53L0X_read_word(dev, 0x84, &tempword);
        status |= VL53L0X_write_byte(dev, 0xFF, 0x00);
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(dev, OscFrequencyMHz,
                                           VL53L0X_FIXPOINT412TOFIXPOINT1616(tempword));
    }

    /* After static init, some device parameters may be changed,
     * so update them */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_device_parameters(dev, &current_parameters);
    }


    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_fraction_enable(dev, &tempbyte);
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, RangeFractionalEnable, tempbyte);
        }

    }

    if (status == VL53L0X_ERROR_NONE) {
        PALDevDataSet(dev, CurrentParameters, current_parameters);
    }


    /* read the sequence config and save it */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_byte(dev,
                                   VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, &tempbyte);
        if (status == VL53L0X_ERROR_NONE) {
            PALDevDataSet(dev, SequenceConfig, tempbyte);
        }
    }

    /* Disable MSRC and TCC by default */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_sequence_step_enable(dev,
                 VL53L0X_SEQUENCESTEP_TCC, 0);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_set_sequence_step_enable(dev,
                 VL53L0X_SEQUENCESTEP_MSRC, 0);
    }

    /* Set PAL State to standby */
    if (status == VL53L0X_ERROR_NONE) {
        PALDevDataSet(dev, PalState, VL53L0X_STATE_IDLE);
    }

    /* Store pre-range vcsel period */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_vcsel_pulse_period(
                     dev,
                     VL53L0X_VCSEL_PERIOD_PRE_RANGE,
                     &vcsel_pulse_period_pclk);
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            dev,
            PreRangeVcselPulsePeriod,
            vcsel_pulse_period_pclk);
    }

    /* Store final-range vcsel period */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_get_vcsel_pulse_period(
                     dev,
                     VL53L0X_VCSEL_PERIOD_FINAL_RANGE,
                     &vcsel_pulse_period_pclk);
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            dev,
            FinalRangeVcselPulsePeriod,
            vcsel_pulse_period_pclk);
    }

    /* Store pre-range timeout */
    if (status == VL53L0X_ERROR_NONE) {
        status = get_sequence_step_timeout(
                     dev,
                     VL53L0X_SEQUENCESTEP_PRE_RANGE,
                     &seq_timeout_micro_secs);
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            dev,
            PreRangeTimeoutMicroSecs,
            seq_timeout_micro_secs);
    }

    /* Store final-range timeout */
    if (status == VL53L0X_ERROR_NONE) {
        status = get_sequence_step_timeout(
                     dev,
                     VL53L0X_SEQUENCESTEP_FINAL_RANGE,
                     &seq_timeout_micro_secs);
    }

    if (status == VL53L0X_ERROR_NONE) {
        VL53L0X_SETDEVICESPECIFICPARAMETER(
            dev,
            FinalRangeTimeoutMicroSecs,
            seq_timeout_micro_secs);
    }

    LOG_FUNCTION_END(status);
    return status;
}


VL53L0X_Error VL53L0X::VL53L0X_stop_measurement(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    LOG_FUNCTION_START("");

    status = VL53L0X_write_byte(dev, VL53L0X_REG_SYSRANGE_START,
                                VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT);

    status = VL53L0X_write_byte(dev, 0xFF, 0x01);
    status = VL53L0X_write_byte(dev, 0x00, 0x00);
    status = VL53L0X_write_byte(dev, 0x91, 0x00);
    status = VL53L0X_write_byte(dev, 0x00, 0x01);
    status = VL53L0X_write_byte(dev, 0xFF, 0x00);

    if (status == VL53L0X_ERROR_NONE) {
        /* Set PAL State to Idle */
        PALDevDataSet(dev, PalState, VL53L0X_STATE_IDLE);
    }

    /* Check if need to apply interrupt settings */
    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_check_and_load_interrupt_settings(dev, 0);
    }

    LOG_FUNCTION_END(status);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_get_stop_completed_status(VL53L0X_DEV dev,
        uint32_t *p_stop_status)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t byte = 0;
    LOG_FUNCTION_START("");

    status = VL53L0X_write_byte(dev, 0xFF, 0x01);

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_read_byte(dev, 0x04, &byte);
    }

    if (status == VL53L0X_ERROR_NONE) {
        status = VL53L0X_write_byte(dev, 0xFF, 0x0);
    }

    *p_stop_status = byte;

    if (byte == 0) {
        status = VL53L0X_write_byte(dev, 0x80, 0x01);
        status = VL53L0X_write_byte(dev, 0xFF, 0x01);
        status = VL53L0X_write_byte(dev, 0x00, 0x00);
        status = VL53L0X_write_byte(dev, 0x91,
                                    PALDevDataGet(dev, StopVariable));
        status = VL53L0X_write_byte(dev, 0x00, 0x01);
        status = VL53L0X_write_byte(dev, 0xFF, 0x00);
        status = VL53L0X_write_byte(dev, 0x80, 0x00);
    }

    LOG_FUNCTION_END(status);
    return status;
}

/****************** Write and read functions from I2C *************************/

VL53L0X_Error VL53L0X::VL53L0X_write_multi(VL53L0X_DEV dev, uint8_t index, uint8_t *p_data, uint32_t count)
{
    int  status;

    status = VL53L0X_i2c_write(dev->I2cDevAddr, index, p_data, (uint16_t)count);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_read_multi(VL53L0X_DEV dev, uint8_t index, uint8_t *p_data, uint32_t count)
{
    int status;

    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
        status = VL53L0X_ERROR_INVALID_PARAMS;
    }

    status = VL53L0X_i2c_read(dev->I2cDevAddr, index, p_data, (uint16_t)count);

    return status;
}


VL53L0X_Error VL53L0X::VL53L0X_write_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    int  status;

    status = VL53L0X_i2c_write(Dev->I2cDevAddr, index, &data, 1);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_write_word(VL53L0X_DEV dev, uint8_t index, uint16_t data)
{
    int  status;
    uint8_t buffer[2];

    buffer[0] = data >> 8;
    buffer[1] = data & 0x00FF;
    status = VL53L0X_i2c_write(dev->I2cDevAddr, index, (uint8_t *)buffer, 2);
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_write_dword(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    int  status;
    uint8_t buffer[4];

    buffer[0] = (data >> 24) & 0xFF;
    buffer[1] = (data >> 16) & 0xFF;
    buffer[2] = (data >>  8) & 0xFF;
    buffer[3] = (data >>  0) & 0xFF;
    status = VL53L0X_i2c_write(Dev->I2cDevAddr, index, (uint8_t *)buffer, 4);
    return status;
}


VL53L0X_Error VL53L0X::VL53L0X_read_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t *p_data)
{
    int  status;

    status = VL53L0X_i2c_read(Dev->I2cDevAddr, index, p_data, 1);

    if (status) {
        return -1;
    }

    return 0;
}

VL53L0X_Error VL53L0X::VL53L0X_read_word(VL53L0X_DEV Dev, uint8_t index, uint16_t *p_data)
{
    int  status;
    uint8_t buffer[2] = {0, 0};

    status = VL53L0X_i2c_read(Dev->I2cDevAddr, index, buffer, 2);
    if (!status) {
        *p_data = (buffer[0] << 8) + buffer[1];
    }
    return status;

}

VL53L0X_Error VL53L0X::VL53L0X_read_dword(VL53L0X_DEV Dev, uint8_t index, uint32_t *p_data)
{
    int status;
    uint8_t buffer[4] = {0, 0, 0, 0};

    status = VL53L0X_i2c_read(Dev->I2cDevAddr, index, buffer, 4);
    if (!status) {
        *p_data = (buffer[0] << 24) + (buffer[1] << 16) + (buffer[2] << 8) + buffer[3];
    }
    return status;

}

VL53L0X_Error VL53L0X::VL53L0X_update_byte(VL53L0X_DEV Dev, uint8_t index, uint8_t and_data, uint8_t or_data)
{
    int  status;
    uint8_t buffer = 0;

    /* read data direct onto buffer */
    status = VL53L0X_i2c_read(Dev->I2cDevAddr, index, &buffer, 1);
    if (!status) {
        buffer = (buffer & and_data) | or_data;
        status = VL53L0X_i2c_write(Dev->I2cDevAddr, index, &buffer, (uint8_t)1);
    }
    return status;
}

VL53L0X_Error VL53L0X::VL53L0X_i2c_write(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t *p_data,
        uint16_t NumByteToWrite)
{
    int ret;

    ret = i2c_write(p_data, DeviceAddr, RegisterAddr, NumByteToWrite);

    if (ret) {
        return -1;
    }
    return 0;
}

VL53L0X_Error VL53L0X::VL53L0X_i2c_read(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t *p_data,
                                        uint16_t NumByteToRead)
{
    int ret;

    ret = i2c_read(p_data, DeviceAddr, RegisterAddr, NumByteToRead);

    if (ret) {
        return -1;
    }
    return 0;
}

int VL53L0X::read_id(uint8_t *id)
{
    int status = 0;
    uint16_t rl_id = 0;

    status = VL53L0X_read_word(_device, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &rl_id);
    if (rl_id == 0xEEAA) {
        return status;
    }

    return -1;
}


VL53L0X_Error VL53L0X::wait_measurement_data_ready(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t new_dat_ready = 0;
    uint32_t loop_nb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (status == VL53L0X_ERROR_NONE) {
        loop_nb = 0;
        do {
            status = VL53L0X_get_measurement_data_ready(dev, &new_dat_ready);
            if ((new_dat_ready == 0x01) || status != VL53L0X_ERROR_NONE) {
                break;
            }
            loop_nb = loop_nb + 1;
            VL53L0X_polling_delay(dev);
        } while (loop_nb < VL53L0X_DEFAULT_MAX_LOOP);

        if (loop_nb >= VL53L0X_DEFAULT_MAX_LOOP) {
            status = VL53L0X_ERROR_TIME_OUT;
        }
    }

    return status;
}

VL53L0X_Error VL53L0X::wait_stop_completed(VL53L0X_DEV dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint32_t stop_completed = 0;
    uint32_t loop_nb;

    // Wait until it finished
    // use timeout to avoid deadlock
    if (status == VL53L0X_ERROR_NONE) {
        loop_nb = 0;
        do {
            status = VL53L0X_get_stop_completed_status(dev, &stop_completed);
            if ((stop_completed == 0x00) || status != VL53L0X_ERROR_NONE) {
                break;
            }
            loop_nb = loop_nb + 1;
            VL53L0X_polling_delay(dev);
        } while (loop_nb < VL53L0X_DEFAULT_MAX_LOOP);

        if (loop_nb >= VL53L0X_DEFAULT_MAX_LOOP) {
            status = VL53L0X_ERROR_TIME_OUT;
        }

    }

    return status;
}


int VL53L0X::init_sensor(uint8_t new_addr)
{
    int status;

    VL53L0X_off();
    VL53L0X_on();

//   status=VL53L0X_WaitDeviceBooted(Device);
//   if(status)
//      printf("WaitDeviceBooted fail\n\r");
    status = is_present();
    if (!status) {
        status = init(&_my_device);
        if (status != VL53L0X_ERROR_NONE) {
            printf("Failed to init VL53L0X sensor!\n\r");
            return status;
        }

        // deduce silicon version
        status = VL53L0X_get_device_info(&_my_device, &_device_info);

        status = prepare();
        if (status != VL53L0X_ERROR_NONE) {
            printf("Failed to prepare VL53L0X!\n\r");
            return status;
        }

        if (new_addr != VL53L0X_DEFAULT_ADDRESS) {
            status = set_device_address(new_addr);
            if (status) {
                printf("Failed to change I2C address!\n\r");
                return status;
            }
        } 
//				else {
//            printf("Invalid new address!\n\r");
//            return VL53L0X_ERROR_INVALID_PARAMS;
//        }
    }
    return status;
}

int VL53L0X::range_meas_int_continuous_mode(void (*fptr)(void))
{
    int status, clr_status;

    status = VL53L0X_stop_measurement(_device); // it is safer to do this while sensor is stopped

//   status = VL53L0X_SetInterruptThresholds(Device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 0, 300);

    status = VL53L0X_set_gpio_config(_device, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                                     VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
                                     VL53L0X_INTERRUPTPOLARITY_HIGH);

    if (!status) {
        attach_interrupt_measure_detection_irq(fptr);
        enable_interrupt_measure_detection_irq();
    }

    clr_status = clear_interrupt(VL53L0X_REG_RESULT_INTERRUPT_STATUS | VL53L0X_REG_RESULT_RANGE_STATUS);
    if (clr_status) {
        VL53L0X_ErrLog("VL53L0X_ClearErrorInterrupt fail\r\n");
    }

    if (!status) {
        status = range_start_continuous_mode();
    }
    return status;
}


int VL53L0X::start_measurement(OperatingMode operating_mode, void (*fptr)(void))
{
    int Status = VL53L0X_ERROR_NONE;
    int ClrStatus;

    uint8_t VhvSettings;
    uint8_t PhaseCal;
    // *** from mass market cube expansion v1.1, ranging with satellites.
    // default settings, for normal range.
    FixPoint1616_t signalLimit = (FixPoint1616_t)(0.25 * 65536);
    FixPoint1616_t sigmaLimit = (FixPoint1616_t)(18 * 65536);
    uint32_t timingBudget = 33000;
    uint8_t preRangeVcselPeriod = 14;
    uint8_t finalRangeVcselPeriod = 10;

    if (operating_mode == range_continuous_interrupt) {
        if (_gpio1Int == NULL) {
            printf("GPIO1 Error\r\n");
            return 1;
        }

        Status = VL53L0X_stop_measurement(_device); // it is safer to do this while sensor is stopped

//        Status = VL53L0X_SetInterruptThresholds(Device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING, 0, 300);

        Status = VL53L0X_set_gpio_config(_device, 0, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING,
                                         VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY,
                                         VL53L0X_INTERRUPTPOLARITY_HIGH);

        if (Status == VL53L0X_ERROR_NONE) {
            attach_interrupt_measure_detection_irq(fptr);
            enable_interrupt_measure_detection_irq();
        }

        ClrStatus = clear_interrupt(VL53L0X_REG_RESULT_INTERRUPT_STATUS | VL53L0X_REG_RESULT_RANGE_STATUS);
        if (ClrStatus) {
            VL53L0X_ErrLog("VL53L0X_ClearErrorInterrupt fail\r\n");
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_device_mode(_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in continuous ranging mode
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_start_measurement(_device);
        }
    }

    if (operating_mode == range_single_shot_polling) {
        // singelshot, polled ranging
        if (Status == VL53L0X_ERROR_NONE) {
            // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
            Status = VL53L0X_set_device_mode(_device, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        }

        // Enable/Disable Sigma and Signal check
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_limit_check_enable(_device,
                                                    VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
        }
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_limit_check_enable(_device,
                                                    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
        }

// *** from mass market cube expansion v1.1, ranging with satellites.
        /* Ranging configuration */
//*
//        switch(rangingConfig) {
//        case LONG_RANGE:
        signalLimit = (FixPoint1616_t)(0.1 * 65536);
        sigmaLimit = (FixPoint1616_t)(60 * 65536);
        timingBudget = 33000;
        preRangeVcselPeriod = 18;
        finalRangeVcselPeriod = 14;
        /*        	break;
                case HIGH_ACCURACY:
        			signalLimit = (FixPoint1616_t)(0.25*65536);
        			sigmaLimit = (FixPoint1616_t)(18*65536);
        			timingBudget = 200000;
        			preRangeVcselPeriod = 14;
        			finalRangeVcselPeriod = 10;
        			break;
                case HIGH_SPEED:
        			signalLimit = (FixPoint1616_t)(0.25*65536);
        			sigmaLimit = (FixPoint1616_t)(32*65536);
        			timingBudget = 20000;
        			preRangeVcselPeriod = 14;
        			finalRangeVcselPeriod = 10;
         			break;
                default:
                	debug_printf("Not Supported");
                }
        */

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_limit_check_value(_device,
                                                   VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, signalLimit);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_limit_check_value(_device,
                                                   VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, sigmaLimit);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_measurement_timing_budget_micro_seconds(_device, timingBudget);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_vcsel_pulse_period(_device,
                                                    VL53L0X_VCSEL_PERIOD_PRE_RANGE, preRangeVcselPeriod);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_set_vcsel_pulse_period(_device,
                                                    VL53L0X_VCSEL_PERIOD_FINAL_RANGE, finalRangeVcselPeriod);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_perform_ref_calibration(_device, &VhvSettings, &PhaseCal);
        }

    }

    if (operating_mode == range_continuous_polling) {
        if (Status == VL53L0X_ERROR_NONE) {
            //printf("Call of VL53L0X_SetDeviceMode\n");
            Status = VL53L0X_set_device_mode(_device, VL53L0X_DEVICEMODE_CONTINUOUS_RANGING); // Setup in continuous ranging mode
        }

        if (Status == VL53L0X_ERROR_NONE) {
            //printf("Call of VL53L0X_StartMeasurement\n");
            Status = VL53L0X_start_measurement(_device);
        }
    }

    return Status;
}


int VL53L0X::get_measurement(OperatingMode operating_mode, VL53L0X_RangingMeasurementData_t *p_data)
{
    int Status = VL53L0X_ERROR_NONE;

    if (operating_mode == range_single_shot_polling) {
        Status = VL53L0X_perform_single_ranging_measurement(_device, p_data);
    }

    if (operating_mode == range_continuous_polling) {
        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_measurement_poll_for_completion(_device);
        }

        if (Status == VL53L0X_ERROR_NONE) {
            Status = VL53L0X_get_ranging_measurement_data(_device, p_data);

            // Clear the interrupt
            VL53L0X_clear_interrupt_mask(_device, VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
            VL53L0X_polling_delay(_device);
        }
    }

    if (operating_mode == range_continuous_interrupt) {
        Status = VL53L0X_get_ranging_measurement_data(_device, p_data);
        VL53L0X_clear_interrupt_mask(_device, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR | VL53L0X_REG_RESULT_INTERRUPT_STATUS);
    }

    return Status;
}


int VL53L0X::stop_measurement(OperatingMode operating_mode)
{
    int status = VL53L0X_ERROR_NONE;


    // don't need to stop for a singleshot range!
    if (operating_mode == range_single_shot_polling) {
    }

    if (operating_mode == range_continuous_interrupt || operating_mode == range_continuous_polling) {
        // continuous mode
        if (status == VL53L0X_ERROR_NONE) {
            //printf("Call of VL53L0X_StopMeasurement\n");
            status = VL53L0X_stop_measurement(_device);
        }

        if (status == VL53L0X_ERROR_NONE) {
            //printf("Wait Stop to be competed\n");
            status = wait_stop_completed(_device);
        }

        if (status == VL53L0X_ERROR_NONE)
            status = VL53L0X_clear_interrupt_mask(_device,
                                                  VL53L0X_REG_SYSTEM_INTERRUPT_GPIO_NEW_SAMPLE_READY);
    }

    return status;
}


int VL53L0X::handle_irq(OperatingMode operating_mode, VL53L0X_RangingMeasurementData_t *data)
{
    int status;
    status = get_measurement(operating_mode, data);
    enable_interrupt_measure_detection_irq();
    return status;
}

    /**
     * @brief  Writes a buffer towards the I2C peripheral device.
     * @param  pBuffer pointer to the byte-array data to send
     * @param  DeviceAddr specifies the peripheral device slave address.
     * @param  RegisterAddr specifies the internal address register
     *         where to start writing to (must be correctly masked).
     * @param  NumByteToWrite number of bytes to be written.
     * @retval 0 if ok,
     * @retval -1 if an I2C error has occured, or
     * @retval -2 on temporary buffer overflow (i.e. NumByteToWrite was too high)
     * @note   On some devices if NumByteToWrite is greater
     *         than one, the RegisterAddr must be masked correctly!
     */
    int VL53L0X::i2c_write(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
                  uint16_t NumByteToWrite) {
        int ret;
        uint8_t tmp[TEMP_BUF_SIZE];

        if(NumByteToWrite >= TEMP_BUF_SIZE) return -2;

        /* First, send device address. Then, send data and STOP condition */
        tmp[0] = RegisterAddr;
        memcpy(tmp+1, pBuffer, NumByteToWrite);

        ret = _i2c->write(DeviceAddr, (const char*)tmp, NumByteToWrite+1, false);

        if(ret) return -1;
        return 0;
    }

    /**
     * @brief  Reads a buffer from the I2C peripheral device.
     * @param  pBuffer pointer to the byte-array to read data in to
     * @param  DeviceAddr specifies the peripheral device slave address.
     * @param  RegisterAddr specifies the internal address register
     *         where to start reading from (must be correctly masked).
     * @param  NumByteToRead number of bytes to be read.
     * @retval 0 if ok,
     * @retval -1 if an I2C error has occured
     * @note   On some devices if NumByteToWrite is greater
     *         than one, the RegisterAddr must be masked correctly!
     */
    int VL53L0X::i2c_read(uint8_t* pBuffer, uint8_t DeviceAddr, uint8_t RegisterAddr,
                 uint16_t NumByteToRead) {
        int ret;

        /* Send device address, with no STOP condition */
        ret = _i2c->write(DeviceAddr, (const char*)&RegisterAddr, 1, true);
        if(!ret) {
            /* Read data, with STOP condition  */
            ret = _i2c->read(DeviceAddr, (char*)pBuffer, NumByteToRead, false);
        }

        if(ret) return -1;
        return 0;
    }

/******************************************************************************/
