/* Copyright 2016 M. Marquardt, HS Wismar */

#include "max6675.h"


// constructor
MAX6675::MAX6675(SPI *spi, PinName _cs)
{
    _spi = spi;
    cs = new DigitalOut(_cs,1);
}

int16_t MAX6675::read(void)
{
    uint16_t data = 0;

    cs->write(0);
    data = _spi->write(0) << 8;
    data |= _spi->write(0);
    cs->write(1);

    return data;
}

float MAX6675::get_temp(void)
{
	uint16_t data=0;
	data = read();
	if(data & 0x4)
		return -1.0f;
	else
	{
		data = (data & 0x7FF8) >> 3;
		return ((float)data * 0.25f);
	}
}
