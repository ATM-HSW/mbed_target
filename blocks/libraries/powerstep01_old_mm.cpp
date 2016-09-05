/* Copyright 2016 M. Marquardt, HS Wismar */

#include "powerstep01.h"


// constructor
POWERSTEP01::POWERSTEP01(SPI *spi, PinName _cs, PinName _reset, uint8_t _numMotors)
{
	_spi = spi;
	cs = new DigitalOut(_cs,1);
	reset = new DigitalOut(_reset,0);
	
	reset->write(1);
	numMotors = _numMotors;
}

void POWERSTEP01::setParam(uint8_t command, uint32_t *value, uint8_t numBytes)
{
	int8_t i,j,shift;
	
	cs->write(0);	
	for(i=numMotors-1; i>=0; i--)
	{
		_spi->write(command);
	}
	cs->write(1);
	
	wait_us(1);
	
	for(j=numBytes; j>0; j--)
	{
		shift = 8*(j-1);
		cs->write(0);	
		for(i=numMotors-1; i>=0; i--)
		{
			_spi->write((uint8_t)(value[i] >> shift));
		}
		cs->write(1);
		
		wait_us(1);
	}
}

void POWERSTEP01::getParam(uint8_t command, uint32_t *value, uint8_t numBytes)
{	
	int8_t i,j,shift;
	
	for(i=0; i<numMotors; i++)
	{
		value[i] = 0;
	}	
	
	cs->write(0);	
	for(i=0; i<numMotors; i++)
	{
		_spi->write(command);
	}
	cs->write(1);
	
	wait_us(1);
	
	for(j=numBytes; j>0; j--)
	{
		shift = 8*(j-1);
		cs->write(0);	
		for(i=numMotors-1; i>=0; i--)
		{
			value[i] |= _spi->write(0) << shift;
		}
		cs->write(1);
		
		wait_us(1);
	}
}

void POWERSTEP01::setup(const float* acc, const float* dec, const float* maxSpeed, const float* minSpeed, uint8_t* mode, uint8_t* voltage)
{
	uint32_t dummy[numMotors];
	int8_t i;

	setParam(0xC0, dummy, 0); // Reset Device
	
	for(i=numMotors-1; i>=0; i--)
	{
		dummy[i] = (uint32_t)(acc[i]/14.55f + 0.5);
		if(dummy[i] >= 0xfff)						// value 0xFFF is reserved (section 9.1.5 in datasheet)
			dummy[i] = 0xffe;
	}

	setParam(0x05, dummy, 2);	// setParam to ACC register address 0x05
	
	for(i=numMotors-1; i>=0; i--)
	{
		dummy[i] = (uint32_t)(dec[i]/14.55f + 0.5);	
		if(dummy[i] >= 0xfff)						// value 0xFFF is reserved (section 9.1.5 in datasheet)
			dummy[i] = 0xffe;
	}

	setParam(0x06, dummy, 2); // setParam to DEC register address 0x06

	for(i=numMotors-1; i>=0; i--)
	{
		dummy[i] = ((uint32_t)(maxSpeed[i]/15.25f + 0.5)) & 0x3FF;	// 10 bit maximum speed register value
	}

	setParam(0x07, dummy, 2); // setParam to MAX_SPEED register address 0x07
	
	for(i=numMotors-1; i>=0; i--)
	{
		dummy[i] = ((uint32_t)(minSpeed[i]/0.238f + 0.5)) & 0xFFF;	// 12 bit minimum speed register value
	}

	setParam(0x08, dummy, 2); // setParam to MIN_SPEED register address 0x08
	
	for(i=0; i<numMotors; i++)
	{
		dummy[i] = (uint32_t)mode[i];
	}
	setParam(0x16, dummy, 1);  // step mode register
	
	for(i=0; i<numMotors; i++)
	{
		dummy[i] = 0x1F;
	}
	setParam(0x13, dummy, 1);	// over current threshold
	setParam(0x14, dummy, 1);  // stall threshold
	
	for(i=0; i<numMotors; i++)
	{
		dummy[i] = 0x2C08;
	}
	setParam(0x1A, dummy, 2);  // config
	
	for(i=0; i<numMotors; i++)
	{
		dummy[i] = (uint32_t)voltage[i];
	}
	setParam(0x09, dummy, 1);
	setParam(0x0A, dummy, 1);
	setParam(0x0B, dummy, 1);
	setParam(0x0C, dummy, 1);
	
}

void POWERSTEP01::run(float* speed, uint8_t* dir)
{
	int8_t i,j,shift;

	uint32_t _speed[numMotors];
	
	for(i=numMotors-1; i>=0; i--)
	{
		_speed[i] = (uint32_t)(speed[i]/0.015f + 0.5);
	}

	cs->write(0);	
	for(i=numMotors-1; i>=0; i--)
	{
		_spi->write(0x50 | (dir[i] & 0x01));
	}
	cs->write(1);
	
	wait_us(1);
	
	for(j=3; j>0; j--)
	{
		shift = 8*(j-1);
		cs->write(0);	
		for(i=numMotors-1; i>=0; i--)
		{
			_spi->write((uint8_t)(_speed[i] >> shift));
		}
		cs->write(1);
		
		wait_us(1);
	}
}

void POWERSTEP01::move(uint32_t* steps, uint8_t* dir)
{	
	int8_t i,j,shift;
	
	cs->write(0);	
	for(i=numMotors-1; i>=0; i--)
	{
		_spi->write(0x40 | (dir[i] & 0x01));
	}
	cs->write(1);
	
	wait_us(1);
	
	for(j=3; j>0; j--)
	{
		shift = 8*(j-1);
		cs->write(0);	
		for(i=numMotors-1; i>=0; i--)
		{
			_spi->write((uint8_t)(steps[i] >> shift));
		}
		cs->write(1);
		
		wait_us(1);
	}
}

void POWERSTEP01::resetPosition()
{	
	uint32_t dummy=0;
	setParam(0xD8, &dummy, 0);
}

void POWERSTEP01::softStop()
{	
	uint32_t dummy=0;
	setParam(0xB0, &dummy, 0);
}

void POWERSTEP01::hardStop()
{
	uint32_t dummy=0;
	setParam(0xB8, &dummy, 0);	
}

void POWERSTEP01::softHiZ()
{
	uint32_t dummy=0;
	setParam(0xA0, &dummy, 0);	
}

void POWERSTEP01::hardHiZ()
{
	uint32_t dummy=0;
	setParam(0xA8, &dummy, 0);	
}

void POWERSTEP01::getStatus(uint32_t* status)
{
	getParam(0xD0, status, 2);
}

void POWERSTEP01::getPosition(uint32_t* position)
{	
	getParam(0x21, position, 3);
}
