/*Copyright 2016 M. Marquardt, HS Wismar */

#include "ds1820.h"


DS1820::DS1820(OneWire* _bus)
{
	bus = _bus;
}

bool DS1820::search_device()
{
	while(1)
	{
		if(!bus->search_rom(ROM))
			return false;
		else if((ROM[0] == FAMILY_CODE_DS18B20) || (ROM[0] == FAMILY_CODE_DS1820))
			return true;
	}
}

bool DS1820::search_device(uint8_t* id)
{
	int i;
	bool match;
	bus->search_rom_init();
	
	while(1)
	{
		match = true;
		if(!bus->search_rom(ROM))
			return false;
		if((ROM[0] != FAMILY_CODE_DS18B20) && (ROM[0] != FAMILY_CODE_DS1820))
			continue;
		
		for(i=0; i<6; i++)
		{
			if(ROM[i+1] != id[i])
			{
				match = false;
				break;
			}
		}
		if(match)
			return true;
	}
}

bool DS1820::read_RAM()
{
	// This will copy the DS1820's 9 bytes of RAM data
	// into the objects RAM array. Functions that use
	// RAM values will automaticly call this procedure.
	int i;
	
	bus->match_rom(ROM);    // Select this device
	bus->byte_out(0xBE);   //Read Scratchpad command
	
	for(i=0;i<9;i++) 
	{
		RAM[i] = bus->byte_in();
	}
  if(!bus->checksum(RAM, 9))
		return false;
	else
		return true;
}

int DS1820::convert_temperature(bool wait, devices device)
{
	// Convert temperature into scratchpad RAM for all devices at once
	int delay_time = 750; // Default delay time
	char resolution;
	if (device==all_devices)
		bus->skip_rom();          // Skip ROM command, will convert for ALL devices
	else 
	{
		bus->match_rom(ROM);
		if (ROM[0] == FAMILY_CODE_DS18B20) 
		{
			resolution = RAM[4] & 0x60;
			if (resolution == 0x00) // 9 bits
				delay_time = 94;
			if (resolution == 0x20) // 10 bits
				delay_time = 188;
			if (resolution == 0x40) // 11 bits. Note 12bits uses the 750ms default
				delay_time = 375;
		}
	}
	bus->byte_out(0x44);  // perform temperature conversion

	if(wait) 
	{
		thread_sleep_for(delay_time);
		delay_time = 0;
	}

	return delay_time;
}

float DS1820::temperature(char scale)
{
// The data specs state that count_per_degree should be 0x10 (16), I found my devices
// to have a count_per_degree of 0x4B (75). With the standard resolution of 1/2 deg C
// this allowed an expanded resolution of 1/150th of a deg C. I wouldn't rely on this
// being super acurate, but it does allow for a smooth display in the 1/10ths of a
// deg C or F scales.
	float answer, remaining_count, count_per_degree;
	int reading;
	
	if(!read_RAM())
		answer = -1000.0f;			// Indicate we got a CRC error
	else 
	{
		reading = (RAM[1] << 8) + RAM[0];
		if(reading & 0x8000)  // negative degrees C
		{
			reading = 0-((reading ^ 0xffff) + 1); // 2's comp then convert to signed int
		}
		answer = reading + 0.0f; // convert to floating point
		if(ROM[0] == FAMILY_CODE_DS18B20) 
		{
			answer = answer / 8.0f;
		}
		else 
		{
			remaining_count = RAM[6];
			count_per_degree = RAM[7];
			answer = answer - 0.25f + (count_per_degree - remaining_count) / count_per_degree;
		}
		if (scale=='C' or scale=='c')
			answer = answer / 2.0f;
		else
			answer = answer * 9.0f / 10.0f + 32.0f;				// Convert to deg F
	}
	return answer;
}