/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef LIS302DL_H
#define LIS302DL_H

#include "mbed.h"

#define LIS302DL_BASE_ADDR          0x38

#define LIS302DL_WHO_AM_I_ADDR      0x0F
#define LIS302DL_CTRL_REG1_ADDR     0x20
#define LIS302DL_CTRL_REG2_ADDR     0x21
#define LIS302DL_CTRL_REG3_ADDR     0x22
#define LIS302DL_HP_FILTER_RESET_REG_ADDR     0x23
#define LIS302DL_STATUS_REG_ADDR    0x27
#define LIS302DL_OUT_X_ADDR         0x29
#define LIS302DL_OUT_Y_ADDR         0x2B
#define LIS302DL_OUT_Z_ADDR         0x2D
#define LIS302DL_FF_WU_CFG1_REG_ADDR          0x30
#define LIS302DL_FF_WU_SRC1_REG_ADDR          0x31
#define LIS302DL_FF_WU_THS1_REG_ADDR          0x32
#define LIS302DL_FF_WU_DURATION1_REG_ADDR     0x33
#define LIS302DL_FF_WU_CFG2_REG_ADDR          0x34
#define LIS302DL_FF_WU_SRC2_REG_ADDR          0x35
#define LIS302DL_FF_WU_THS2_REG_ADDR          0x36
#define LIS302DL_FF_WU_DURATION2_REG_ADDR     0x37
#define LIS302DL_CLICK_CFG_REG_ADDR           0x38
#define LIS302DL_CLICK_SRC_REG_ADDR           0x39
#define LIS302DL_CLICK_THSY_X_REG_ADDR        0x3B
#define LIS302DL_CLICK_THSZ_REG_ADDR          0x3C
#define LIS302DL_CLICK_TIMELIMIT_REG_ADDR     0x3D
#define LIS302DL_CLICK_LATENCY_REG_ADDR       0x3E
#define LIS302DL_CLICK_WINDOW_REG_ADDR        0x3F

#define I_AM_LIS302DL               0x3B

#define LIS302DL_LOWPOWERMODE_POWERDOWN       ((uint8_t)0x00)
#define LIS302DL_LOWPOWERMODE_ACTIVE          ((uint8_t)0x40)

#define LIS302DL_X_ENABLE           ((uint8_t)0x01)
#define LIS302DL_Y_ENABLE           ((uint8_t)0x02)
#define LIS302DL_Z_ENABLE           ((uint8_t)0x04)
#define LIS302DL_XYZ_ENABLE         ((uint8_t)0x07)

#define LIS302DL_SENSITIVITY_2_3G   18  /* 18 mg/digit*/
#define LIS302DL_SENSITIVITY_9_2G   72  /* 72 mg/digit*/

#define LIS302DL_DATARATE_100       ((uint8_t)0x00)
#define LIS302DL_DATARATE_400       ((uint8_t)0x80)

#define LIS302DL_FULLSCALE_2_3      ((uint8_t)0x00)
#define LIS302DL_FULLSCALE_9_2      ((uint8_t)0x20)

#define LIS302DL_SELFTEST_NORMAL    ((uint8_t)0x00)
#define LIS302DL_SELFTEST_P         ((uint8_t)0x10)
#define LIS302DL_SELFTEST_M         ((uint8_t)0x08)

#define LIS302DL_HIGHPASSFILTER_BYPASS       ((uint8_t)0x00)
#define LIS302DL_HIGHPASSFILTER_LEVEL_0      ((uint8_t)0x10)
#define LIS302DL_HIGHPASSFILTER_LEVEL_1      ((uint8_t)0x11)
#define LIS302DL_HIGHPASSFILTER_LEVEL_2      ((uint8_t)0x12)
#define LIS302DL_HIGHPASSFILTER_LEVEL_3      ((uint8_t)0x13)

#define LIS302DL_INTERRUPT_DISABLE           ((uint8_t)0x00)
#define LIS302DL_INTERRUPT_OPENDRAIN         ((uint8_t)0xC0)
#define LIS302DL_INTERRUPT_PUSHPULL          ((uint8_t)0x80)

#define LIS302DL_INTERRUPT_DRDY1             ((uint8_t)0x04)
#define LIS302DL_INTERRUPT_DRDY2             ((uint8_t)0x20)

/****************************************
 * LIS302DL Class
 ***************************************/

class LIS302DL
{
private:
	SPI *_spi;
	I2C *_i2c;
	DigitalOut *cs;
	uint8_t interface;
	uint8_t dev_addr;
	uint8_t factor;

    void write_i2c(uint8_t reg, uint8_t *data, uint8_t len);
    void write_spi(uint8_t reg, uint8_t *data, uint8_t len);

	void read_i2c(uint8_t reg, uint8_t *data, uint8_t len);
	void read_spi(uint8_t reg, uint8_t *data, uint8_t len);

	void get_xyz_raw(int8_t *x, int8_t *y, int8_t *z);

public:
	// constructor
	LIS302DL(SPI *spi, PinName pin);

	LIS302DL(I2C *i2c, uint8_t addr);

    bool init(uint8_t rate, uint8_t scale, uint8_t hp_filter, uint8_t interrupt_reg);

	void get_xyz(int16_t *x, int16_t *y, int16_t *z);

	void get_xyz(float *x, float *y, float *z);
};

#endif /* LIS302DL_H */
