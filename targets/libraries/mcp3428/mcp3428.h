#ifndef MCP3428_H
#define MCP3428_H

#include "mbed.h"

#define MCP3428_BASE_ADDR 0xD0

#define MCP3428_SAMPLERATE_240SPS (0) // 12 bit resolution
#define MCP3428_SAMPLERATE_60SPS (1)  // 14 bit resolution
#define MCP3428_SAMPLERATE_15SPS (2)  // 16 bit resolution

#define MCP3428_PGA_GAIN_1X (0)
#define MCP3428_PGA_GAIN_2X (1)
#define MCP3428_PGA_GAIN_4X (2)
#define MCP3428_PGA_GAIN_8X (3)

#define MCP3428_nRDY_BIT (1<<7)

/****************************************
 * MCP3428 Class
 ***************************************/

class MCP3428
{
private:
	uint8_t dev_addr;
	I2C *_i2c;

	// perform atomic read
	bool read(int16_t *data, uint8_t *conf);
	// write configuration
	bool write(uint8_t conf);

public:
	// constructor
	MCP3428(I2C *i2c, uint8_t addr);

	// perform conversion and return raw result
	bool read_raw(uint8_t channel, uint8_t samplerate, uint8_t pga, int16_t *data, bool continuous);
	// perform conversion and return voltage
	bool read_voltage(uint8_t channel, uint8_t samplerate, uint8_t pga, float *data, bool continuous);
	// perform conversion and return value in range 0..1
	bool read_relative(uint8_t channel, uint8_t samplerate, uint8_t pga, float *data, bool continuous);


};

#endif /* MCP3428_H */
