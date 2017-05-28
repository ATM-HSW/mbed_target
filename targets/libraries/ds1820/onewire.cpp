/*Copyright 2016 M. Marquardt, HS Wismar */

#include "onewire.h"

OneWire::OneWire(PinName data) : _data(data)
{
	diff = OW_LAST_DEVICE;
	search_done = false;
}
	
bool OneWire::reset()
{
	bool presence=false;
	_data.output();
	_data = 0;           // bring low for 500 us
	wait_us(500);
	_data.input();       // let the data line float high
	wait_us(90);            // wait 90us
	if (_data.read()==0) // see if any devices are pulling the data line low
		presence=true;
	wait_us(410);
	return presence;
}

void OneWire::bit_out(bool bit)
{
	_data.output();
	_data = 0;
	wait_us(2);
	if (bit) 
	{
		_data.input(); // bring data line high
		wait_us(80);
	} 
	else 
	{
		wait_us(80);            // keep data line low
		_data.input();
		wait_us(10);
	}
}

void OneWire::byte_out(uint8_t byte)
{
	int i;
	for (i=0; i<8; i++) 
	{
		bit_out(byte & 0x01);
		byte = byte >> 1; // now the next bit is in the least sig bit position.
  }
}

bool OneWire::bit_in()
{
	bool answer;
	_data.output();
	_data = 0;
	wait_us(2);              
	_data.input();
	wait_us(2);               
	answer = _data.read();
	wait_us(45);                
	return answer;
}

uint8_t OneWire::byte_in()
{
	uint8_t answer = 0x00;
	int i;
	for (i=0; i<8; i++) 
	{
		answer = answer >> 1; // shift over to make room for the next bit
		if (bit_in())
			answer = answer | 0x80; // if the data port is high, make this bit a 1
	}
	return answer;
}
	  
bool OneWire::read_rom(uint8_t* rom)
{
	int i;
	bool error=false;
	error = reset();
	byte_out(OW_READ_ROM); // read ROM command
	for (i=0; i<8; i++)
		rom[i]=byte_in();
	
	return error;
}

bool OneWire::search_rom(uint8_t* rom)
{
	uint8_t i, j, next_diff, bitmask;
  bool a,b;
	
	if(!reset())
		return false;    // error, no device found

	byte_out(OW_SEARCH_ROM);            // ROM search command
	next_diff = OW_LAST_DEVICE;            // unchanged on last device

	i = 1;
	j = 0;
	bitmask = 0x01;
	
	if(!search_done)
	{
		while(i<=64)
		{
			a = bit_in();
			b = bit_in();
			
			if(a&b)							// data error 
				return false;
			
			if(a|b)			// bits are different
			{
				if(a)			// set ROM bit to a
					rom[j] = rom[j] | bitmask;
				else				
					rom[j] = rom[j] &~bitmask; // set ROM bit to zero
			}
			else				// two or more devices present
			{
				if(i == diff)
				{
					rom[j] = rom[j] | bitmask;	// set ROM bit to one
				}
				else
				{
					if(i>diff)
					{
						rom[j] = rom[j] &~bitmask; // set ROM bit to zero
						next_diff = i;
					}
					else
					{
						if((rom[j] & bitmask) == 0x00)
							next_diff = i;
					}					
				}
			}
			bit_out(rom[j] & bitmask);
			i++;
			if(bitmask & 0x80)
			{
				j++;
				bitmask = 0x01;
			}
			else
				bitmask = bitmask << 1;
		}
		diff = next_diff;                // to continue search
		
		if(diff == 0x00)
			search_done = true;
		
		if(!checksum(rom,8))
		{
			search_done = true;
			return false;
		}
		else
			return true;	
	}
	else
	{
		rom[0] = 0xFF;
		return false;
	}
}

void OneWire::search_rom_init()
{
	diff = OW_LAST_DEVICE;
	search_done = false;
}

bool OneWire::match_rom(uint8_t* rom)
{
	int i;
	bool error=false;
	error = reset();
	byte_out(OW_MATCH_ROM);  //Match ROM command
	for (i=0;i<8;i++)
		byte_out(rom[i]);
	
	return error;		// false when an error occured
}

bool OneWire::skip_rom()
{
	bool error=false;
	error = reset();
  byte_out(OW_SKIP_ROM);   // Skip ROM command
	
	return error;
}

uint8_t OneWire::CRC_byte(uint8_t _CRC, uint8_t byte)
{
	int j;
    for(j=0;j<8;j++) 
	  {
			if ((byte & 0x01 ) ^ (_CRC & 0x01)) 
			{
				// DATA ^ LSB _CRC = 1
				_CRC = _CRC>>1;
				// Set the MSB to 1
				_CRC = _CRC | 0x80;
				// Check bit 3
				if (_CRC & 0x04) 
				{
						_CRC = _CRC & 0xFB; // Bit 3 is set, so clear it
				} 
				else 
				{
						_CRC = _CRC | 0x04; // Bit 3 is clear, so set it
				}
				// Check bit 4
				if (_CRC & 0x08) 
				{
						_CRC = _CRC & 0xF7; // Bit 4 is set, so clear it
				} 
				else 
				{
						_CRC = _CRC | 0x08; // Bit 4 is clear, so set it
				}
			} 
			else 
			{
				// DATA ^ LSB _CRC = 0
				_CRC = _CRC>>1;
				// clear MSB
				_CRC = _CRC & 0x7F;
				// No need to check bits, with DATA ^ LSB _CRC = 0, they will remain unchanged
			}
			byte = byte>>1;
    }
	return _CRC;
}

bool OneWire::checksum(uint8_t* data, uint8_t count)
{	
    uint8_t _CRC=0x00;
    int i;
    for(i=0;i<count-1;i++)
        _CRC = CRC_byte(_CRC, data[i]);
    // After 7 bytes _CRC should equal the 8th byte (ROM _CRC)
    return (_CRC == data[i]); // will return true if there is a _CRC checksum mis-match
}
