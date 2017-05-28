/* Copyright 2015 M. Marquardt, HS Wismar */

#include "l3gd20.h"


// constructor for SPI use
L3GD20::L3GD20(SPI *spi, PinName pin)
{
    _spi = spi;
    cs = new DigitalOut(pin,1);
    interface = 0;
}

// constructor for I2C use
// possible addr values 0...1
L3GD20::L3GD20(I2C *i2c, uint8_t addr)
{
    _i2c = i2c;
    dev_addr = (L3GD20_BASE_ADDR | ((addr & 0x1) << 1));
    interface = 1;
}

void L3GD20::write_i2c(uint8_t reg, uint8_t *data, uint8_t len)
{
    if(len > 1)
        reg |= 0x80;

    _i2c->write(dev_addr, (char*)&reg, 1, 1);
    _i2c->write(dev_addr, (char*)data, len, 0);
}
void L3GD20::write_spi(uint8_t reg, uint8_t *data, uint8_t len)
{
    if(len > 1)
        reg |= 0x40;
    cs->write(0);

    _spi->write(reg);
    for(uint8_t i=0; i<len; i++)
    {
        _spi->write(data[i]);
    }

    cs->write(1);
}

void L3GD20::read_i2c(uint8_t reg, uint8_t *data, uint8_t len)
{
    if(len > 1)
        reg |= 0x80;

    _i2c->write(dev_addr, (char*)&reg, 1, 1);
    _i2c->read(dev_addr, (char*)data, len, 0);
}
void L3GD20::read_spi(uint8_t reg, uint8_t *data, uint8_t len)
{
    if(len > 1)
        reg |= 0x40;

    reg |= 0x80;

    cs->write(0);

    data[0] = _spi->write(reg);
    for(uint8_t i=0; i<len; i++)
    {
        data[i] = _spi->write(0);
    }
    cs->write(1);
}


bool L3GD20::init(uint8_t rate, uint8_t bandwidth, uint8_t scale, uint8_t hp_filter, bool drdy_int)
{
    uint8_t buf[5], whoami=0;

    buf[0] = (rate & 0xC0) | (bandwidth & 0x30) | L3GD20_MODE_ACTIVE | L3GD20_AXES_ENABLE;
    buf[1] = hp_filter & 0x0F;
    buf[2] = (drdy_int << 3);
    buf[3] = scale & 0x30;
    buf[4] = (hp_filter & 0x10);

    if(scale == L3GD20_FULLSCALE_250)
        factor = L3GD20_SENSITIVITY_250DPS;
    else if(scale == L3GD20_FULLSCALE_500)
        factor = L3GD20_SENSITIVITY_500DPS;
    else
        factor = L3GD20_SENSITIVITY_2000DPS;

    if(interface == 0) // spi used
    {
        read_spi(L3GD20_WHO_AM_I_ADDR, &whoami, 1);
        if( !( (whoami == 0xD4) || (whoami == 0xD5) ) )
        {
            return true;
        }

        write_spi(L3GD20_CTRL_REG1_ADDR, buf, 5);
    }
    else               // i2c used
    {
        read_i2c(L3GD20_WHO_AM_I_ADDR, &whoami, 1);
        if( !( (whoami == 0xD4) || (whoami == 0xD5) ) )
        {
            return true;
        }

        write_i2c(L3GD20_CTRL_REG1_ADDR, buf, 5);
    }
    return false;
}

void L3GD20::get_xyz_raw(int16_t *x, int16_t *y, int16_t *z)
{
    int8_t buf[6];
    if(interface == 0)  // spi used
    {
        read_spi(L3GD20_OUT_X_L_ADDR, (uint8_t*)buf, 6);
    }
    else                // i2c used
    {
        read_i2c(L3GD20_OUT_X_L_ADDR, (uint8_t*)buf, 6);
    }

    *x = (buf[1] << 8) | buf[0];
    *y = (buf[3] << 8) | buf[2];
    *z = (buf[5] << 8) | buf[4];
}

void L3GD20::get_xyz(float *x, float *y, float *z)
{
    int16_t x_, y_, z_;

    get_xyz_raw(&x_, &y_, &z_);

    *x = (float)x_ / factor;
    *y = (float)y_ / factor;
    *z = (float)z_ / factor;
}
