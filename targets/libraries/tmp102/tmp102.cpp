/* Copyright 2015 M. Marquardt, HS Wismar */

#include "tmp102.h"


// constructor
// valid addr values 0 to 3
TMP102::TMP102(I2C *i2c, uint8_t addr)
{
    _i2c = i2c;
    dev_addr = TMP102_BASE_ADDR | ((addr & 0x3) << 1);
    oneshot = 0;
}

// write data
bool TMP102::write(uint8_t *data)
{
    return _i2c->write(dev_addr, (char*)data, 3, 0);
}

// read data
bool TMP102::read(uint8_t reg, uint16_t *data)
{
    int ret = _i2c->write(dev_addr, (char*)&reg, 1, 0);
    if(ret)
        return true;

    return _i2c->read(dev_addr, (char*)data, 2, 0);
}

bool TMP102::get_temp(float *temp)
{
    uint16_t data = 0;

    if(oneshot)
    {
        if(write(config))
            return true;

        while((data & 0x80) != 0x80)
        {
            if(read(TMP102_CONFIG_REG, &data))
                return true;
        }
    }

    if(read(TMP102_TEMP_REG, &data))
        return true;

    if(data & 0x80)
    {
        if(config[2] & 0x10)
            data = ((data >> 11) | (data << 5)) & 0x1fff;
        else
            data = (data >> 12) | (data << 4);

        data |= 0xf000;
        data = (~data) + 1;
        *temp = (float)data * -0.0625f;
    }
    else
    {
        if(config[2] & 0x10)
            data = ((data >> 11) | (data << 5)) & 0x1fff;
        else
            data = (data >> 12) | (data << 4);

        *temp = (float)data * 0.0625f;
    }

    return false;
}

// configure TMP102 device
// param:   continuous -> 0 = continuous conversion mode, 1 = shutdown + oneshot mode
//          thermostat -> 0 = comparator mode, 1 = interrupt mode
//          extended   -> 0 = 12 bit resolution, 1 = 13 bit resolution
//          rate       -> valid values 0 to 3 for .25Hz, 1Hz, 4Hz and 8Hz continuous conversion rate
// ret:     0 if OK, 1 if Fail
bool TMP102::configure(uint8_t continuous, uint8_t thermostat, uint8_t extended, uint8_t rate)
{
    config[0] = TMP102_CONFIG_REG;

    config[1] = ((continuous & 0x1) << 7) | (TMP102_FAULTQUEUE << 3) | ((thermostat & 0x1) << 1) | (continuous & 0x1);

    config[2] = ((rate & 0x3) << 6) | ((extended & 0x1) << 4);

    if(continuous)
        oneshot = 1;
    else
        oneshot = 0;

    return write(config);
}

// set temperature limit registers
bool set_limits(uint16_t high, uint16_t low)
{
    return false;
}




