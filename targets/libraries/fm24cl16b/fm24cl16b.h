#ifndef FM24CL16B_H
#define FM24CL16B_H

#include "mbed.h"

#define FM24CL16B_BASE_ADDR 0xA0

/****************************************
 * FM24CL16B Class
 ***************************************/

class FM24CL16B
{
private:
	uint8_t base_addr;
	I2C *_i2c;

    bool send_addr(uint16_t addr);
public:
	// constructor
	FM24CL16B(I2C *i2c);

    bool write_block(uint16_t addr, uint8_t *buff, uint16_t len);

    bool read_block(uint16_t addr, uint8_t *buff, uint16_t len);

    bool write_byte(uint16_t addr, uint8_t data);

    bool read_byte(uint16_t addr, uint8_t *data);


};

#endif /* FM24CL16B_H */
