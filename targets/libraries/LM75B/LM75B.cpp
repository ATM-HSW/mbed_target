#include "LM75B.h"

LM75B::LM75B(I2C *i2c) {
   char cmd[2];
  _i2c = i2c;
   cmd[0]    = LM75B_Conf;
   cmd[1]    = 0x0;   
   _i2c->write( LM75B_ADDR, cmd, 2);
}

LM75B::~LM75B()
{

}

float LM75B::read()
{
    char cmd[2];
    cmd[0] = LM75B_Temp;

    _i2c->write( LM75B_ADDR, cmd, 1); // Send command string
    _i2c->read( LM75B_ADDR, cmd, 2); // Reads 2 bytes
    return (   float((cmd[0]<<8)|cmd[1]) / 256.0   );
}
