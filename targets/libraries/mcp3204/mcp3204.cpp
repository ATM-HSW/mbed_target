/* Copyright 2015 M. Marquardt, HS Wismar */

#include "mcp3204.h"


// constructor
MCP3204::MCP3204(SPI *spi, PinName pin)
{
    _spi = spi;
    cs = new DigitalOut(pin,1);
}

uint16_t MCP3204::read(uint8_t *config)
{
    uint16_t data = 0;

    cs->write(0);
    _spi->write(config[0]);
    data = _spi->write(config[1]) << 8;
    data |= _spi->write(0);
    cs->write(1);

    return data;
}

uint16_t MCP3204::read_raw(uint8_t channel, uint8_t single_ended)
{
    uint8_t config[2] = {4,0};

    config[0] |= (single_ended & 0x1) << 1;
    config[1] = (channel & 0x3) << 6;

    return read(config);
}

float MCP3204::read_relative(uint8_t channel, uint8_t single_ended)
{
    uint8_t config[2] = {4,0};

    config[0] |= (single_ended & 0x1) << 1;
    config[1] = (channel & 0x3) << 6;

    return ((float)read(config) / 4095.0f);
}
