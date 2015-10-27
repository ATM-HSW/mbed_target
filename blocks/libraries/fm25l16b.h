/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef FM25L16B_H
#define FM25L16B_H

#include "mbed.h"

/****************************************
 * FM25L16B Class
 ***************************************/

#define FM25L16B_OPCODE_WREN (0b0110)
#define FM25L16B_OPCODE_WRDI (0b0100)
#define FM25L16B_OPCODE_RDSR (0b0101)
#define FM25L16B_OPCODE_WRSR (0b0001)
#define FM25L16B_OPCODE_READ (0b0011)
#define FM25L16B_OPCODE_WRITE (0b0010)

class FM25L16B
{
private:
	SPI *_spi;
	DigitalOut *cs;

    void write_enable();

public:
	// constructor
	FM25L16B(SPI *spi, PinName pin);

    uint8_t read_status();

    void write_block(uint16_t addr, uint8_t *buff, uint16_t len);

    void read_block(uint16_t addr, uint8_t *buff, uint16_t len);

    void write_byte(uint16_t addr, uint8_t data);

    uint8_t read_byte(uint16_t addr);
};

#endif /* FM25L16B_H */
