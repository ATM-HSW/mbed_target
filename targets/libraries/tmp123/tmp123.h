/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef TMP123_H
#define TMP123_H

#include "mbed.h"

/****************************************
 * TMP123 Class
 ***************************************/

class TMP123
{
private:
	SPI *_spi;
	DigitalOut *cs;

	int16_t read(void);

public:
	// constructor
	TMP123(SPI *spi, PinName pin);

	float get_temp(void);
};

#endif /* TMP123_H */
