/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef TMP102_H
#define TMP102_H

#include "mbed.h"

#define TMP102_BASE_ADDR 0x90

// tmp102 register addresses
#define TMP102_TEMP_REG     (0)
#define TMP102_CONFIG_REG   (1)
#define TMP102_TLOW_REG     (2)
#define TMP102_THIGH_REG    (3)

// tmp102 conversion rates
#define TMP102_RATE_025HZ   (0)
#define TMP102_RATE_1HZ     (1)
#define TMP102_RATE_4HZ     (2)
#define TMP102_RATE_8HZ     (3)

// Shutdown Bit
#define TMP102_CONTINUOUS   (0)
#define TMP102_SHUTDOWN     (1)

// thermostat (TM) Bit
#define TMP102_COMPARATOR   (0)
#define TMP102_INTERRUPT    (1)
// extended mode Bit
#define TMP102_12BIT        (0)
#define TMP102_13BIT        (1)


// change by user
#define TMP102_FAULTQUEUE   (0) // Number of consecutive fault conditions valid 0 to 3 for 1, 2, 4, 6 consecutive fault events

/****************************************
 * TMP102 Class
 ***************************************/

class TMP102
{
private:
	uint8_t dev_addr;
	uint8_t oneshot;
	uint8_t config[3];
	I2C *_i2c;

	// write data
	bool write(uint8_t *data);

	bool read(uint8_t reg, uint16_t *data);

public:
	// constructor
	TMP102(I2C *i2c, uint8_t addr);

	bool get_temp(float *temp);

	bool configure(uint8_t continuous, uint8_t thermostat, uint8_t extended, uint8_t rate);

	bool set_limits(uint16_t high, uint16_t low);

};

#endif /* TMP102_H */
