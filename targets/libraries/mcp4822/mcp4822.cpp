/* 
 * Copyright 2015 M. Marquardt, HS Wismar 
 * Copyright 2020 Dr.O.Hagendorf, HS Wismar 
 */
 
#include "mcp4822.h"

// constructor
MCP4822::MCP4822(SPI *spi, PinName pin) {
  _spi = spi;
  cs = new DigitalOut(pin,1);
}

void MCP4822::write_raw(uint8_t channel, uint8_t gain, uint8_t shutdown, uint16_t value) {
  if(value > 4096) value = 4096;

  uint8_t out1 = ((channel & 0x1) << 7) | ((gain & 0x1) << 5) | ((shutdown & 0x1) << 4) | ((value >> 8) & 0xF);
  uint8_t out2 = value & 0xFF;

  cs->write(0);
  _spi->write(out1);
  _spi->write(out2);
  cs->write(1);
}

void MCP4822::write_voltage(uint8_t channel, uint8_t gain, uint8_t shutdown, float value) {
  uint16_t raw = 0;
  if((gain & 0x1) == 1)     {
    raw = (uint16_t)((value/2.048f)*4096);
  } else {
    raw = (uint16_t)((value/4.096f)*4096);
  }
  write_raw(channel, gain, shutdown, raw);
}

void MCP4822::write_relative(uint8_t channel, uint8_t gain, uint8_t shutdown, float value) {
  if(value > 1.0f) value = 1.0f;

  uint16_t raw = (uint16_t)((value * 4096) + 0.5f);
  write_raw(channel, gain, shutdown, raw);
}

