#include "BMP180.h"

// Constructor for BMP180 Class
	BMP180::BMP180(I2C *i2c) 
{
	_i2c=i2c;
}
	
//===================================================================================================================
//====== Set of useful function to access pressure and temperature data
//===================================================================================================================

    void BMP180::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
   char data_write[2];
   data_write[0] = subAddress;
   data_write[1] = data;
   _i2c->write(address, data_write, 2, 0);
}

    char BMP180::readByte(uint8_t address, uint8_t subAddress)
{
    char data[1]; // `data` will store the register data     
    char data_write[1];
    data_write[0] = subAddress;
    _i2c->write(address, data_write, 1, 1); // no stop
    _i2c->read(address, data, 1, 0); 
    return data[0]; 
}

    void BMP180::readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{     
    char data[14];
    char data_write[1];
    data_write[0] = subAddress;
    _i2c->write(address, data_write, 1, 1); // no stop
    _i2c->read(address, data, count, 0); 
    for(int ii = 0; ii < count; ii++) {
     dest[ii] = data[ii];
    }
} 
 

// Stores all of the BMP180's calibration values into global variables
// Calibration values are required to calculate temp and pressure
// This function should be called at the beginning of the program
// These BMP-180 functions were adapted from Jim Lindblom of SparkFun Electronics
void BMP180::BMP180Calibration()
{
  ac1 = readByte(BMP180_ADDRESS, 0xAA) << 8 | readByte(BMP180_ADDRESS, 0xAB);
  ac2 = readByte(BMP180_ADDRESS, 0xAC) << 8 | readByte(BMP180_ADDRESS, 0xAD);
  ac3 = readByte(BMP180_ADDRESS, 0xAE) << 8 | readByte(BMP180_ADDRESS, 0xAF);
  ac4 = readByte(BMP180_ADDRESS, 0xB0) << 8 | readByte(BMP180_ADDRESS, 0xB1);
  ac5 = readByte(BMP180_ADDRESS, 0xB2) << 8 | readByte(BMP180_ADDRESS, 0xB3);
  ac6 = readByte(BMP180_ADDRESS, 0xB4) << 8 | readByte(BMP180_ADDRESS, 0xB5);
  b1  = readByte(BMP180_ADDRESS, 0xB6) << 8 | readByte(BMP180_ADDRESS, 0xB7);
  b2  = readByte(BMP180_ADDRESS, 0xB8) << 8 | readByte(BMP180_ADDRESS, 0xB9);
  mb  = readByte(BMP180_ADDRESS, 0xBA) << 8 | readByte(BMP180_ADDRESS, 0xBB);
  mc  = readByte(BMP180_ADDRESS, 0xBC) << 8 | readByte(BMP180_ADDRESS, 0xBD);
  md  = readByte(BMP180_ADDRESS, 0xBE) << 8 | readByte(BMP180_ADDRESS, 0xBF);
}

  // Temperature returned will be in units of 0.1 deg C
  int16_t BMP180::BMP180GetTemperature()
{
  long ut = 0;
  writeByte(BMP180_ADDRESS, 0xF4, 0x2E); // start temperature measurement
  thread_sleep_for(5);	// Max. conversion time = 4.5 ms
  uint8_t rawData[2] = {0, 0};
  readBytes(BMP180_ADDRESS, 0xF6, 2, &rawData[0]); // read raw temperature measurement
  ut = (((long) rawData[0] << 8) | rawData[1]);
 
 long x1, x2;
  
  x1 = (((long)ut - (uint16_t)ac6)*(uint16_t)ac5) >> 15;
  x2 = ((int16_t)mc << 11)/(x1 + (int16_t)md);
  b5 = x1 + x2;

  return  ((b5 + 8)>>4);  
}

// Calculate pressure read calibration values  
// b5 is also required so BMP180GetTemperature() must be called first.
// Value returned will be pressure in units of Pa.
long BMP180::BMP180GetPressure()
{
  long up = 0;
  writeByte(BMP180_ADDRESS, 0xF4, 0x34 | OSS << 6); // Configure pressure measurement for highest resolution
  thread_sleep_for(26); // delay 5 ms at lowest resolution, 26 ms at highest
  uint8_t rawData[3] = {0, 0, 0};
  readBytes(BMP180_ADDRESS, 0xF6, 3, &rawData[0]); // read raw pressure measurement of 19 bits
  up = (((long) rawData[0] << 16) | ((long)rawData[1] << 8) | rawData[2]) >> (8 - OSS);

  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;
  
  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;
  
  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;
  
  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;
    
  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;
  
  return p;
}      

