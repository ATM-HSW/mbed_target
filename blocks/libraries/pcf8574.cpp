/* Copyright 2015 M. Marquardt, HS Wismar */

#include "pcf8574.h"

// constructor
// valid addr values 0..7
PCF8574::PCF8574(I2C *i2c, uint8_t addr)
{
    _i2c = i2c;
    state = 0xFF;

	dev_addr = base_addr | (addr << 1);
}

// set the whole pcf8574 port at once
// ret:  zero if OK, none zero if Fail
bool PCF8574::set_port(uint8_t data)
{
    return _i2c->write(dev_addr, (char*) &data, 1, 0);
}

// read the whole pcf8574 port at once
// ret:  zero if OK, none zero if Fail
bool PCF8574::get_port(uint8_t *data)
{
    return _i2c->read(dev_addr, (char*) data, 1, 0);
}

// set a single pin of the pcf5874
// ret:  zero if OK, none zero if Fail
bool PCF8574::set_pin(uint8_t pin, uint8_t value)
{
    if(value)
        state |= (1<<pin);
    else
        state &= ~(1<<pin);

    return set_port(state);
}

// read a single pin of the pcf 8574
// ret:  zero if OK, none zero if Fail
bool PCF8574::get_pin(uint8_t pin, uint8_t *value)
{
    uint8_t data;
    if(get_port(&data))
        return true;
    else
    {
        *value = (data >> pin) & 0x1;
    }
    return false;
}
