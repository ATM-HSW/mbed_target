/* 
 * Copyright 2015 M. Marquardt, HS Wismar 
 * Copyright 2020 Dr.O.Hagendorf, HS Wismar 
 */

#include "mcp4728.h"


// constructor
// valid addr values 0 to 7
MCP4728::MCP4728(I2C *i2c, uint8_t addr) {
  _i2c = i2c;
  dev_addr = (MCP4728_BASE_ADDR | ((addr & 0x7) << 1));
}

// write data
// ret: zero if OK
//      none zero if Fail
bool MCP4728::write(uint8_t *data, uint8_t len) {
  int ret = _i2c->write(dev_addr, (char*) data, len, 0);

  if(ret == 0)
    return false;
  else
    return true;
}

// takes 12 bit raw input and converts to analog voltage
// ret: zero if OK
//      none zero if Fail
bool MCP4728::write_raw(uint8_t channel, uint8_t vref, uint8_t pdm, uint8_t gain, uint16_t data) {
  uint8_t buf[3];

  buf[0] = 0x40 | ((channel & 0x3) << 1);

  buf[1] = (vref << 7) | ((pdm & 0x3) << 5) | ((gain & 0x1) << 4);
  buf[1] |= (uint8_t)((data & 0xf00) >> 8);

  buf[2] = (uint8_t)(data & 0xff);

  return write(buf, 3);
}

// outputs given voltage value
// ret: zero if OK
//      none zero if Fail
bool MCP4728::write_voltage(uint8_t channel, uint8_t vref, uint8_t pdm, uint8_t gain, float data) {
  float lsb = 0;
  uint16_t raw = 0;

  if(vref & 0x1)   {
    if(gain & 0x1)     {
      lsb = 4.096f / 4096;
    } else {
      lsb = 2.048f / 4096;
    }
  }
  else {
    lsb = 3.3f / 4096;
  }

  raw = (uint16_t)((data / lsb) + 0.5f);

  return write_raw(channel, vref, pdm, gain, raw);
}

// takes data in range 0..1.0 and converts to analog voltage
// ret: zero if OK
//      none zero if Fail
bool MCP4728::write_relative(uint8_t channel, uint8_t vref, uint8_t pdm, uint8_t gain, float data) {
  if(data > 1.0f)  data = 1.0f;

  uint16_t raw = (uint16_t)((data * 4096) + 0.5f);

  return write_raw(channel, vref, pdm, gain, raw);
}

// set power down modes of each channel
// ret: zero if OK
//      none zero if Fail
bool MCP4728::set_powerDownMode(uint8_t chA_pdm, uint8_t chB_pdm, uint8_t chC_pdm, uint8_t chD_pdm) {
  uint8_t buf[2];

  buf[0] = 0xA0 | ((chA_pdm & 0x3) << 2) | (chB_pdm & 0x3);
  buf[1] = ((chC_pdm & 0x3) << 6) | ((chD_pdm & 0x3) << 4);

  return write(buf, 2);
}
