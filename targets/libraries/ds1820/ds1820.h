/*Copyright 2016 M. Marquardt, HS Wismar */

#ifndef DS1820_H
#define DS1820_H

#include "onewire.h"


#define FAMILY_CODE_DS1820 0x10
#define FAMILY_CODE_DS18S20 0x10
#define FAMILY_CODE_DS18B20 0x28


class DS1820
{

	private:
	  uint8_t RAM[9];
	  OneWire* bus;
		
	public:
	
enum devices{
        this_device,     // command applies to only this device
        all_devices };   // command applies to all devices

		uint8_t ROM[8];
	
		DS1820(OneWire* _bus);
		bool read_RAM();
	  bool search_device();
		bool search_device(uint8_t* id);
		int convert_temperature(bool wait, devices device=this_device);
	  float temperature(char scale='c');	  
};

#endif // DS1820_H
