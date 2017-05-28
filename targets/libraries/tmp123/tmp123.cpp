/* Copyright 2015 M. Marquardt, HS Wismar */

#include "tmp123.h"


// constructor
TMP123::TMP123(SPI *spi, PinName pin)
{
    _spi = spi;
    cs = new DigitalOut(pin,1);
}

int16_t TMP123::read(void)
{
    uint16_t data = 0;

    cs->write(0);
    data = _spi->write(0) << 8;
    data |= _spi->write(0);
    cs->write(1);

    data &= 0xFFF8;
    return (int16_t)data;
}

float TMP123::get_temp(void)
{
    return ((float)read() / 8) * 0.0625f;
}
