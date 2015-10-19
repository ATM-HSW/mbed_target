/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef MCP4728_H
#define MCP4728_H

#include "mbed.h"

#define MCP4728_BASE_ADDR 0xC0

#define MCP4728_GAIN_1X (0)
#define MCP4728_GAIN_2X (1)

#define MCP4728_Vref_External (0)
#define MCP4728_Vref_Internal (1)

#define MCP4728_Channel_A (0)
#define MCP4728_Channel_B (1)
#define MCP4728_Channel_C (2)
#define MCP4728_Channel_D (3)

#define MCP4728_PDM_Normal (0)
#define MCP4728_PDM_1K	   (1)
#define MCP4728_PDM_100K   (2)
#define MCP4728_PDM_500K   (3)


/****************************************
 * MCP4728 Class
 ***************************************/

class MCP4728
{
private:
	uint8_t dev_addr;
	I2C *_i2c;

	// write data
	bool write(uint8_t *data, uint8_t len);

public:
	// constructor
	MCP4728(I2C *i2c, uint8_t addr);

	// takes 12 bit raw input and converts to analog voltage
	bool write_raw(uint8_t channel, uint8_t vref, uint8_t pdm, uint8_t gain, uint16_t data);

	// outputs given voltage value
	bool write_voltage(uint8_t channel, uint8_t vref, uint8_t pdm, uint8_t gain, float data);

	// takes value in range 0..1.0 and converts to analog voltage
	bool write_relative(uint8_t channel, uint8_t vref, uint8_t pdm, uint8_t gain, float data);

	// set power down modes of each channel
	bool set_powerDownMode(uint8_t chA_pdm, uint8_t chB_pdm, uint8_t chC_pdm, uint8_t chD_pdm);
};

#endif /* MCP4728_H */
