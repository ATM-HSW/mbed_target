#include "mcp3428.h"


// constructor
MCP3428::MCP3428(I2C *i2c, uint8_t addr)
{
	_i2c = i2c;
	dev_addr = (MCP3428_BASE_ADDR | addr);
}



// perform atomic read
bool MCP3428::read(int16_t *data, uint8_t *conf)
{
	char buf[3];
	int ret = _i2c->read(dev_addr, buf, 3, 0);

	if(ret != 0)
		return false;

	*data = buf[0] << 8 | buf[1];
	*conf = buf[2];

	return true;
}
// write configuration
bool MCP3428::write(uint8_t conf)
{
	int ret = _i2c->write(dev_addr, (char*) &conf, 1 ,0);

	if(ret != 0)
		return false;
    else
        return true;
}


// perform conversion and return raw result
bool MCP3428::read_raw(uint8_t channel, uint8_t samplerate, uint8_t pga, int16_t *data, bool continuous)
{
	uint8_t conf = 0;
	conf |= ((channel & 3) << 5) | (continuous << 4) | MCP3428_nRDY_BIT;
	conf |= ((samplerate & 3) << 2) | ((pga & 3) << 0);

    bool status = write(conf);
	if(!status)
		return false;

	conf = 0xff;

	while(conf & MCP3428_nRDY_BIT)
	{
		if(!read(data, &conf))
			return false;
	}

	return true;
}

// perform conversion and return voltage
bool MCP3428::read_voltage(uint8_t channel, uint8_t samplerate, uint8_t pga, float *data, bool continuous)
{
	uint8_t conf = 0;
	int16_t data_int = 0;
	conf |= ((channel & 3) << 5) | (continuous << 4) | MCP3428_nRDY_BIT;
	conf |= ((samplerate & 3) << 2) | ((pga & 3) << 0);

    bool status = write(conf);
	if(!status)
		return false;

	conf = 0xff;

	while(conf & MCP3428_nRDY_BIT)
	{
		if(!read(&data_int, &conf))
			return false;
	}

	float lsb_V = (2.048f / (1<<(11+samplerate*2))) / (1<<pga);

	*data = data_int * lsb_V;

	return true;
}
// perform conversion and return value in range 0..1
bool MCP3428::read_relative(uint8_t channel, uint8_t samplerate, uint8_t pga, float *data, bool continuous)
{
	uint8_t conf = 0;
	int16_t data_int = 0;
	conf |= ((channel & 3) << 5) | (continuous << 4) | MCP3428_nRDY_BIT;
	conf |= ((samplerate & 3) << 2) | ((pga & 3) << 0);

    bool status = write(conf);
	if(!status)
		return false;

	conf = 0xff;

	while(conf & MCP3428_nRDY_BIT)
	{
		if(!read(&data_int, &conf))
			return false;
	}

	*data = (float)data_int / ((1<<(11+samplerate*2))-1);

	return true;
}
