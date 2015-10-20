#ifndef BMP180_H
#define BMP180_H
 
#include "mbed.h"

#define BMP180_ADDRESS          0x77<<1 // I2C address of BMP180, eight bit address on mbed
#define BMP180_WHO_AM_I         0xD0    // WHO_AM_I id of BMP180, should return 0x55
#define BMP180_RESET            0xE0
#define BMP180_CONTROL          0xF4
#define BMP180_OUT_MSB          0xF6
#define BMP180_OUT_LSB          0xF7
#define BMP180_OUT_XLSB         0xF8
#define OSS 3 	// maximum pressure resolution

class BMP180 {
 
    protected:
        I2C *_i2c;
		// Set initial input parameters     
			
		// These are constants used to calculate the temperature and pressure from the BMP-180 sensor
		int16_t ac1, ac2, ac3, b1, b2, mb, mc, md;  
		uint16_t ac4, ac5, ac6;
		long b5;
		 
    public:
// Constructor for BMP180 Class	
	BMP180(I2C *i2c);
//===================================================================================================================
//====== Set of useful function to access pressure and temperature data
//===================================================================================================================

    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);

    char readByte(uint8_t address, uint8_t subAddress);

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
 

// Stores all of the BMP180's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
// These BMP-180 functions were adapted from Jim Lindblom of SparkFun Electronics
	void BMP180Calibration();

// Temperature returned will be in units of 0.1 deg C
	int16_t BMP180GetTemperature();

// Calculate pressure read calibration values  
// b5 is also required so BMP180GetTemperature() must be called first.
// Value returned will be pressure in units of Pa.
	long BMP180GetPressure();    

  };
#endif