/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef MCP3204_H
#define MCP3204_H

#include "mbed.h"


#define MCP3204_CH0 (0)
#define MCP3204_CH1 (1)
#define MCP3204_CH2 (2)
#define MCP3204_CH3 (3)

#define MCP3204_DIFFERENTIAL (0)
#define MCP3204_SINGLE_ENDED (1)

/****************************************
 * MCP3204 Class
 ***************************************/

class MCP3204
{
private:
	SPI *_spi;
	DigitalOut *cs;

	uint16_t read(uint8_t *config);

public:
	// constructor
	MCP3204(SPI *spi, PinName pin);

	uint16_t read_raw(uint8_t channel, uint8_t single_ended);

	float read_relative(uint8_t channel, uint8_t single_ended);
};

#endif /* MCP3204_H */
