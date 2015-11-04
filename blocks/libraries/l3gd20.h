/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef L3GD20_H
#define L3GD20_H

#include "mbed.h"

#define L3GD20_BASE_ADDR 0xD4

/******************************************************************************/
/*************************** START REGISTER MAPPING  **************************/
/******************************************************************************/
#define L3GD20_WHO_AM_I_ADDR          0x0F  /* device identification register */
#define L3GD20_CTRL_REG1_ADDR         0x20  /* Control register 1 */
#define L3GD20_CTRL_REG2_ADDR         0x21  /* Control register 2 */
#define L3GD20_CTRL_REG3_ADDR         0x22  /* Control register 3 */
#define L3GD20_CTRL_REG4_ADDR         0x23  /* Control register 4 */
#define L3GD20_CTRL_REG5_ADDR         0x24  /* Control register 5 */
#define L3GD20_REFERENCE_REG_ADDR     0x25  /* Reference register */
#define L3GD20_OUT_TEMP_ADDR          0x26  /* Out temp register */
#define L3GD20_STATUS_REG_ADDR        0x27  /* Status register */
#define L3GD20_OUT_X_L_ADDR           0x28  /* Output Register X */
#define L3GD20_OUT_X_H_ADDR           0x29  /* Output Register X */
#define L3GD20_OUT_Y_L_ADDR           0x2A  /* Output Register Y */
#define L3GD20_OUT_Y_H_ADDR           0x2B  /* Output Register Y */
#define L3GD20_OUT_Z_L_ADDR           0x2C  /* Output Register Z */
#define L3GD20_OUT_Z_H_ADDR           0x2D  /* Output Register Z */
#define L3GD20_FIFO_CTRL_REG_ADDR     0x2E  /* Fifo control Register */
#define L3GD20_FIFO_SRC_REG_ADDR      0x2F  /* Fifo src Register */

#define L3GD20_INT1_CFG_ADDR          0x30  /* Interrupt 1 configuration Register */
#define L3GD20_INT1_SRC_ADDR          0x31  /* Interrupt 1 source Register */
#define L3GD20_INT1_TSH_XH_ADDR       0x32  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_XL_ADDR       0x33  /* Interrupt 1 Threshold X register */
#define L3GD20_INT1_TSH_YH_ADDR       0x34  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_YL_ADDR       0x35  /* Interrupt 1 Threshold Y register */
#define L3GD20_INT1_TSH_ZH_ADDR       0x36  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_TSH_ZL_ADDR       0x37  /* Interrupt 1 Threshold Z register */
#define L3GD20_INT1_DURATION_ADDR     0x38  /* Interrupt 1 DURATION register */

/******************************************************************************/
/**************************** END REGISTER MAPPING  ***************************/
/******************************************************************************/


#define I_AM_L3GD20                 ((uint8_t)0xD4)

#define L3GD20_MODE_POWERDOWN       ((uint8_t)0x00)
#define L3GD20_MODE_ACTIVE          ((uint8_t)0x08)

#define L3GD20_OUTPUT_DATARATE_95   ((uint8_t)0x00)
#define L3GD20_OUTPUT_DATARATE_190  ((uint8_t)0x40)
#define L3GD20_OUTPUT_DATARATE_380  ((uint8_t)0x80)
#define L3GD20_OUTPUT_DATARATE_760  ((uint8_t)0xC0)

#define L3GD20_X_ENABLE            ((uint8_t)0x02)
#define L3GD20_Y_ENABLE            ((uint8_t)0x01)
#define L3GD20_Z_ENABLE            ((uint8_t)0x04)
#define L3GD20_AXES_ENABLE         ((uint8_t)0x07)
#define L3GD20_AXES_DISABLE        ((uint8_t)0x00)

#define L3GD20_BANDWIDTH_1         ((uint8_t)0x00)
#define L3GD20_BANDWIDTH_2         ((uint8_t)0x10)
#define L3GD20_BANDWIDTH_3         ((uint8_t)0x20)
#define L3GD20_BANDWIDTH_4         ((uint8_t)0x30)

#define L3GD20_FULLSCALE_250       ((uint8_t)0x00)
#define L3GD20_FULLSCALE_500       ((uint8_t)0x10)
#define L3GD20_FULLSCALE_2000      ((uint8_t)0x20)
#define L3GD20_FULLSCALE_SELECTION ((uint8_t)0x30)

#define L3GD20_SENSITIVITY_250DPS  ((float)8.75f)         /*!< gyroscope sensitivity with 250 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_500DPS  ((float)17.50f)        /*!< gyroscope sensitivity with 500 dps full scale [DPS/LSB]  */
#define L3GD20_SENSITIVITY_2000DPS ((float)70.00f)        /*!< gyroscope sensitivity with 2000 dps full scale [DPS/LSB] */

#define L3GD20_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define L3GD20_HIGHPASSFILTER_ENABLE	   ((uint8_t)0x10)

#define L3GD20_HPFCF_0              0x10
#define L3GD20_HPFCF_1              0x11
#define L3GD20_HPFCF_2              0x12
#define L3GD20_HPFCF_3              0x13
#define L3GD20_HPFCF_4              0x14
#define L3GD20_HPFCF_5              0x15
#define L3GD20_HPFCF_6              0x16
#define L3GD20_HPFCF_7              0x17
#define L3GD20_HPFCF_8              0x18
#define L3GD20_HPFCF_9              0x19



/****************************************
 * L3GD20 Class
 ***************************************/

class L3GD20
{
private:
	SPI *_spi;
	I2C *_i2c;
	DigitalOut *cs;
	uint8_t interface;
	uint8_t dev_addr;
	float factor;

    void write_i2c(uint8_t reg, uint8_t *data, uint8_t len);
    void write_spi(uint8_t reg, uint8_t *data, uint8_t len);

	void read_i2c(uint8_t reg, uint8_t *data, uint8_t len);
	void read_spi(uint8_t reg, uint8_t *data, uint8_t len);

	void get_xyz_raw(int16_t *x, int16_t *y, int16_t *z);

public:
	// constructor
	L3GD20(SPI *spi, PinName pin);

	L3GD20(I2C *i2c, uint8_t addr);

    bool init(uint8_t rate, uint8_t bandwidth, uint8_t scale, uint8_t hp_filter, bool drdy_int);

	void get_xyz(float *x, float *y, float *z);
};

#endif /* LIS302DL_H */
