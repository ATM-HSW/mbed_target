/* Copyright 2016 M. Marquardt, HS Wismar */

#ifndef __POWERSTEP01_H
#define __POWERSTEP01_H

#include "mbed.h"



// Step mode defines

#define PowerSTEP01_MODE_FULLSTEP			0
#define PowerSTEP01_MODE_HALFSTEP			1
#define PowerSTEP01_MODE_1_4_STEP			2
#define PowerSTEP01_MODE_1_8_STEP			3
#define PowerSTEP01_MODE_1_16_STEP		4
#define PowerSTEP01_MODE_1_32_STEP		5
#define PowerSTEP01_MODE_1_64_STEP		6
#define PowerSTEP01_MODE_1_128_STEP		7

/****************************************
 * powerSTEP01 Class
 ***************************************/

class POWERSTEP01
{
private:
	SPI *_spi;
	DigitalOut *cs, *reset;
	uint8_t numMotors;

public:
	void setParam(uint8_t reg, uint32_t* value, uint8_t numBytes);
  void getParam(uint8_t reg, uint32_t* value, uint8_t numBytes);

	// constructor
	POWERSTEP01(SPI *spi, PinName _cs, PinName _reset, uint8_t _numMotors=1);

	void setup(const float* acc, const float* dec, const float* maxSpeed, const float* minSpeed, uint8_t* mode, uint8_t* voltage);
	void run(float* speed, uint8_t* dir);
	void move(uint32_t* steps, uint8_t* dir);

	void resetPosition();
	void softStop();
	void hardStop();
	void softHiZ();
	void hardHiZ();

	void getStatus(uint32_t* status);
  void getPosition(uint32_t* position);
};

#endif // __POWERSTEP01_H
