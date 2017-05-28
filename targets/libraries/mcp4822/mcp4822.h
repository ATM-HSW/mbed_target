/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef MCP4822_H
#define MCP4822_H

#include "mbed.h"

#define MCP4822_CHA (0)
#define MCP4822_CHB (1)

#define MCP4822_GAIN_1X (1)
#define MCP4822_GAIN_2X (0)

#define MCP4822_POWER_ON (1)
#define MCP4822_POWER_OFF (0)

/****************************************
 * MCP4822 Class
 ***************************************/

class MCP4822
{
private:
	SPI *_spi;
	DigitalOut *cs;

public:
	// constructor
	MCP4822(SPI *spi, PinName pin);

	void write_raw(uint8_t channel, uint8_t gain, uint8_t shutdown, uint16_t value);

    void write_voltage(uint8_t channel, uint8_t gain, uint8_t shutdown, float value);

	void write_relative(uint8_t channel, uint8_t gain, uint8_t shutdown, float value);
};

#endif /* MCP4822_H */
