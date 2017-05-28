/* Copyright 2016 M. Marquardt, HS Wismar */

#ifndef MAX6675_H
#define MAX6675_H

#include "mbed.h"

/****************************************
 * MAX6675 Class
 ***************************************/

class MAX6675
{
private:
	SPI *_spi;
	DigitalOut *cs;

	int16_t read(void);

public:
	// constructor
	MAX6675(SPI *spi, PinName _cs);

	float get_temp(void);
};

#endif /* MAX6675_H */
