/* Copyright 2015 M. Marquardt, HS Wismar */

#include "fm25l16b.h"


// constructor
FM25L16B::FM25L16B(SPI *spi, PinName pin)
{
    _spi = spi;
    cs = new DigitalOut(pin,1);
}


void FM25L16B::write_enable()
{
    cs->write(0);
    _spi->write(FM25L16B_OPCODE_WREN);
    cs->write(1);
}

uint8_t FM25L16B::read_status()
{
    uint8_t data = 0;

    cs->write(0);
    _spi->write(FM25L16B_OPCODE_RDSR);
    data = _spi->write(0);
    cs->write(1);

    return data;
}

void FM25L16B::write_block(uint16_t addr, uint8_t *buff, uint16_t len)
{
    write_enable();

    cs->write(0);
    _spi->write(FM25L16B_OPCODE_WRITE);
    _spi->write((uint8_t)(addr>>8));
    _spi->write((uint8_t)(addr & 0xFF));

    for(int i = 0; i < len; i++)
    {
        _spi->write(buff[i]);
    }

    cs->write(1);
}

void FM25L16B::read_block(uint16_t addr, uint8_t *buff, uint16_t len)
{
    cs->write(0);
    _spi->write(FM25L16B_OPCODE_READ);
    _spi->write((uint8_t)(addr>>8));
    _spi->write((uint8_t)(addr & 0xFF));

    for(int i = 0; i < len; i++)
    {
       buff[i] = _spi->write(0);
    }

    cs->write(1);
}

void FM25L16B::write_byte(uint16_t addr, uint8_t data)
{
    write_enable();

    cs->write(0);
    _spi->write(FM25L16B_OPCODE_WRITE);
    _spi->write((uint8_t)(addr>>8));
    _spi->write((uint8_t)(addr & 0xFF));

    _spi->write(data);

    cs->write(1);
}

uint8_t FM25L16B::read_byte(uint16_t addr)
{
    uint8_t data;

    cs->write(0);
    _spi->write(FM25L16B_OPCODE_READ);
    _spi->write((uint8_t)(addr>>8));
    _spi->write((uint8_t)(addr & 0xFF));

    data = _spi->write(0);

    cs->write(1);

    return data;
}
