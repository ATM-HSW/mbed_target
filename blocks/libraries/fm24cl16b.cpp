/* Copyright 2015 M. Marquardt, HS Wismar */

#include "fm24cl16b.h"


// constructor
// valid addr values 0 to 7
FM24CL16B::FM24CL16B(I2C *i2c)
{
	_i2c = i2c;

	base_addr = FM24CL16B_BASE_ADDR;
}

bool FM24CL16B::send_addr(uint16_t addr)
{
    uint8_t dev_addr, word_addr;

    dev_addr = base_addr | ((addr>>7) & 0xE);
    word_addr = addr & 0xFF;

    return _i2c->write(dev_addr, (char*)&word_addr, 1, 1);
}


bool FM24CL16B::write_block(uint16_t addr, uint8_t *buff, uint16_t len)
{
    int ret = send_addr(addr);

    if(ret)
    {
        _i2c->stop();
        return true;
    }

	for (int i = 0; i < len; i++)
    {
		ret = _i2c->write(buff[i]);
		if (ret != 1)
		{
			_i2c->stop();
			return true;
		}
	}

	_i2c->stop();
    return false;
}

bool FM24CL16B::read_block(uint16_t addr, uint8_t *buff, uint16_t len)
{
    int ret = send_addr(addr);

    if(ret)
    {
        _i2c->stop();
        return true;
    }

    return _i2c->read((base_addr | ((addr>>7) & 0xE)), (char*)buff, len, 0);
}

bool FM24CL16B::write_byte(uint16_t addr, uint8_t data)
{
    uint8_t dev_addr = base_addr | ((addr>>7) & 0xE);
    uint8_t buff[2];

    buff[0] = addr & 0xFF;
    buff[1] = data;

    return _i2c->write(dev_addr, (char*)buff, 2, 0);
}

bool FM24CL16B::read_byte(uint16_t addr, uint8_t *data)
{
    int ret = send_addr(addr);

    if(ret)
    {
        _i2c->stop();
        return true;
    }

    return _i2c->read((base_addr | ((addr>>7) & 0xE)), (char*)data, 1, 0);
}
