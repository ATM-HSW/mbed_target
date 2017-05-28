/* Copyright 2015 M. Marquardt, HS Wismar */

#include "mcp3428.h"


// constructor
// valid addr values 0 to 7
MCP3428::MCP3428(I2C *i2c, uint8_t addr)
{
	_i2c = i2c;

	dev_addr = (MCP3428_BASE_ADDR | ((addr & 0x7) << 1));
}



// perform atomic read
// ret: zero if OK
//      none zero Fail
bool MCP3428::read(int16_t *data, uint8_t *conf)
{
	char buf[3];
	int ret = _i2c->read(dev_addr, buf, 3, 0);

	if(ret != 0)
		return true;

	*data = buf[0] << 8 | buf[1];
	*conf = buf[2];

	return false;
}
// write configuration
// ret: zero if OK
//      none zero Fail
bool MCP3428::write(uint8_t conf)
{
	int ret = _i2c->write(dev_addr, (char*) &conf, 1 ,0);

	if(ret != 0)
		return true;
    else
        return false;
}


// perform conversion and return raw result
// ret: zero if OK
//      none zero Fail
bool MCP3428::read_raw(uint8_t channel, uint8_t samplerate, uint8_t pga, int16_t *data, bool continuous)
{
	uint8_t conf = 0;
	conf |= ((channel & 3) << 5) | (continuous << 4) | MCP3428_nRDY_BIT;
	conf |= ((samplerate & 3) << 2) | ((pga & 3) << 0);

    bool status = write(conf);
	if(status)
		return true;

	conf = 0xff;

	while(conf & MCP3428_nRDY_BIT)
	{
		if(read(data, &conf))
			return true;
	}

	return false;
}

// perform conversion and return voltage
// ret: zero if OK
//      none zero Fail
bool MCP3428::read_voltage(uint8_t channel, uint8_t samplerate, uint8_t pga, float *data, bool continuous)
{
	uint8_t conf = 0;
	int16_t data_int = 0;
	conf |= ((channel & 3) << 5) | (continuous << 4) | MCP3428_nRDY_BIT;
	conf |= ((samplerate & 3) << 2) | ((pga & 3) << 0);

    bool status = write(conf);
	if(status)
		return true;

	conf = 0xff;

	while(conf & MCP3428_nRDY_BIT)
	{
		if(read(&data_int, &conf))
			return true;
	}

	float lsb_V = (2.048f / (1<<(11+samplerate*2))) / (1<<pga);

	*data = data_int * lsb_V;

	return false;
}
// perform conversion and return value in range 0..1
// ret: zero if OK
//      none zero Fail
bool MCP3428::read_relative(uint8_t channel, uint8_t samplerate, uint8_t pga, float *data, bool continuous)
{
	uint8_t conf = 0;
	int16_t data_int = 0;
	conf |= ((channel & 3) << 5) | (continuous << 4) | MCP3428_nRDY_BIT;
	conf |= ((samplerate & 3) << 2) | ((pga & 3) << 0);

    bool status = write(conf);
	if(status)
		return true;

	conf = 0xff;

	while(conf & MCP3428_nRDY_BIT)
	{
		if(read(&data_int, &conf))
			return true;
	}

	*data = (float)data_int / ((1<<(11+samplerate*2))-1);

	return false;
}
