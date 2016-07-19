/*Copyright 2016 M. Marquardt, HS Wismar */

#ifndef ONEWIRE_H
#define ONEWIRE_H

#include "mbed.h"


#define OW_READ_ROM     0x33
#define OW_MATCH_ROM    0x55
#define OW_SKIP_ROM     0xCC
#define OW_SEARCH_ROM   0xF0

#define OW_LAST_DEVICE  0x00


class OneWire
{
	private:	
		DigitalInOut _data;
		//DigitalOut _power;
	  uint8_t diff;	// last decrepancy in search_rom routine
		bool search_done;
	  uint8_t CRC_byte(uint8_t _CRC, uint8_t byte);
	
	public:	
		OneWire(PinName data);
	
		bool reset();
	  void bit_out(bool bit);
	  void byte_out(uint8_t byte);
	  bool bit_in();
	  uint8_t byte_in();
	  
	  bool read_rom(uint8_t* rom);
	  bool search_rom(uint8_t* rom);
	  void search_rom_init();
	  bool match_rom(uint8_t* rom);
	  bool skip_rom();
	  bool checksum(uint8_t* data, uint8_t count);
	
};


#endif // ONEWIRE_H
