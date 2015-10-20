/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef PCF8574_H
#define PCF8574_H

#include "mbed.h"

#define PCF8574_BASE_ADDR 0x40
#define PCF8574A_BASE_ADDR 0x70


/***************************************
*  PCF8574 Class
***************************************/

class PCF8574
{
private:
    I2C *_i2c;
    uint8_t state;
    uint8_t dev_addr;

protected:
    const uint8_t base_addr = PCF8574_BASE_ADDR;

public:

    PCF8574(I2C *i2c, uint8_t addr);
    // set the whole pcf8574 port at once
    // ret:  zero if OK, none zero if Fail
    bool set_port(uint8_t data);

    // read the whole pcf8574 port at once
    // ret:  zero if OK, none zero if Fail
    bool get_port(uint8_t *data);

    // set a single pin of the pcf5874
    // ret:  zero if OK, none zero if Fail
    bool set_pin(uint8_t pin, uint8_t value);

    // read a single pin of the pcf 8574
    // ret:  zero if OK, none zero if Fail
    bool get_pin(uint8_t pin, uint8_t *value);
};

class PCF8574A : public PCF8574
{
protected:
    const uint8_t base_addr = PCF8574A_BASE_ADDR;

public:

    PCF8574A(I2C *i2c, uint8_t addr)
     : PCF8574(i2c, addr)
    {
    }
};

#endif /* PCF8574_H */
