/* Copyright 2015 M. Marquardt, HS Wismar */

#include "lis302dl.h"


// constructor for SPI use
LIS302DL::LIS302DL(SPI *spi, PinName pin)
{
    _spi = spi;
    cs = new DigitalOut(pin,1);
    interface = 0;
}

// constructor for I2C use
// possible addr values 0...1
LIS302DL::LIS302DL(I2C *i2c, uint8_t addr)
{
    _i2c = i2c;
    dev_addr = (LIS302DL_BASE_ADDR | ((addr & 0x1) << 1));
    interface = 1;
}

void LIS302DL::write_i2c(uint8_t reg, uint8_t *data, uint8_t len)
{
    if(len > 1)
        reg |= 0x80;

    _i2c->write(dev_addr, (char*)&reg, 1, 1);
    _i2c->write(dev_addr, (char*)data, len, 0);
}
void LIS302DL::write_spi(uint8_t reg, uint8_t *data, uint8_t len)
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

void LIS302DL::read_i2c(uint8_t reg, uint8_t *data, uint8_t len)
{
    if(len > 1)
        reg |= 0x80;

    _i2c->write(dev_addr, (char*)&reg, 1, 1);
    _i2c->read(dev_addr, (char*)data, len, 0);
}
void LIS302DL::read_spi(uint8_t reg, uint8_t *data, uint8_t len)
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


bool LIS302DL::init(uint8_t rate, uint8_t scale, uint8_t hp_filter, uint8_t interrupt_reg)
{
    uint8_t buf[3], whoami=0;

    buf[0] = (rate & 0x80) | (scale & 0x20) | LIS302DL_SELFTEST_NORMAL | LIS302DL_LOWPOWERMODE_ACTIVE | LIS302DL_XYZ_ENABLE;
    buf[1] = hp_filter;
    buf[2] = interrupt_reg;

    if(scale & 0x20)
        factor = 72;
    else
        factor = 18;

    if(interface == 0) // spi used
    {
        read_spi(LIS302DL_WHO_AM_I_ADDR, &whoami, 1);
        if(whoami != 0x3B)
        {
            return true;
        }

        write_spi(LIS302DL_CTRL_REG1_ADDR, buf, 3);
    }
    else               // i2c used
    {
        read_i2c(LIS302DL_WHO_AM_I_ADDR, &whoami, 1);
        if(whoami != 0x3B)
        {
            return true;
        }

        write_i2c(LIS302DL_CTRL_REG1_ADDR, buf, 3);
    }
	return false;
}

void LIS302DL::get_xyz_raw(int8_t *x, int8_t *y, int8_t *z)
{
    int8_t buf[5];
    if(interface == 0)  // spi used
    {
        read_spi(LIS302DL_OUT_X_ADDR, (uint8_t*)buf, 5);
    }
    else                // i2c used
    {
        read_i2c(LIS302DL_OUT_X_ADDR, (uint8_t*)buf, 5);
    }

    *x = buf[0];
    *y = buf[2];
    *z = buf[4];
}

void LIS302DL::get_xyz(int16_t *x, int16_t *y, int16_t *z)
{
    int8_t x_, y_, z_;
    get_xyz_raw(&x_, &y_, &z_);

    *x = (int16_t)x_ * factor;
    *y = (int16_t)y_ * factor;
    *z = (int16_t)z_ * factor;
}

void LIS302DL::get_xyz(float *x, float *y, float *z)
{
    int8_t x_, y_, z_;

    get_xyz_raw(&x_, &y_, &z_);

    *x = (float)x_ * factor;
    *y = (float)y_ * factor;
    *z = (float)z_ * factor;
}
