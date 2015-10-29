/* Copyright 2015 M. Marquardt, HS Wismar */

#include "max11300.h"


// constructor
MAX11300::MAX11300(SPI *spi, PinName pin)
{
    _spi = spi;
    cs = new DigitalOut(pin,1);
    gpio_state = 0;
}

// reads the content of a MAX11300 register
// param:   reg - register address
// retval:  16Bit register content
uint16_t MAX11300::read(uint8_t reg)
{
    uint16_t data = 0;

    cs->write(0);

    _spi->write((reg << 1) | 1); // write register address and R/W bit set to 1

    data = (uint16_t) _spi->write(0) << 8;
    data |= _spi->write(0);

    cs->write(1);
    return data;
}

// writes data to a MAX11300 register
// param:  reg - register address
//         data - new register content that is being written
void MAX11300::write(uint8_t reg, uint16_t data)
{
    cs->write(0);

    _spi->write(reg << 1); // write register address

    _spi->write(data >> 8);
    _spi->write(data & 0xFF);

    cs->write(1);
}

// configures the MAX11300 device's control register
// param: adcRate - sets the conversion rate of the ADC
//        dacRef  - selects the reference voltage of the DAC to either external (=0) or internal (=1)
//        temp_channel - enables temperature measurement through the given channel (internal(1), ch1(2) and/or ch2(4))
void MAX11300::config_device(uint8_t adcRate, uint8_t dacRef, uint8_t temp_channel)
{
    uint16_t data = 0;
	data = MAX11300_ADCCTL_CONTINOUS | MAX11300_DACCTL_SEQUENTIUAL;
    data |= temp_channel << 8;
    data |= adcRate << 4;
	data |= dacRef << 6;
    write(MAX11300_CTL_REG, data);
}

// configures a pin of the MAX11300 to perform a defined function
// param:  pin - the pin which should be configured 0 to 19
//         mode - function of the pin (ADC, DAC, GPIO, etc)
//         associatedPort - if mode selects a function which needs two pins (e.g. differential ADC) this parameter is the pin number of the other port
//         powerOfSamples - defines the number of samples to be averaged before loading the ADC result in die data register (possible 2^(0...7))
//         range - determines the input or output voltage range (depending on the mode)
//         adcRefExternal - selects the reference voltage for an ADC configured pin to either external (1) or internal (0)
//         invertInput - in GPI controlled mode the received data can be inverted (1) or not inverted (0)
void MAX11300::config_pin(uint8_t pin, uint8_t mode, uint8_t associatedPort, uint8_t powerOfSamples, uint8_t range, bool adcRefExternal, bool invertInput)
{
	uint16_t config = 0;
	config |= (associatedPort & 0x1F);
	config |= (powerOfSamples & 0x7) << 5;
	config |= (range & 0x7) << 8;

	if(mode == 4 || mode == 11)
		config |= invertInput << 11;
	else
		config |= adcRefExternal << 11;

	config |= (mode & 0xF) << 12;

	write(0x20 + pin, config);
}

// reads the ADC conversion result of an anaolg input pin
// param: pin - number of the pin to be read (0..19)
//        pinMode - ADC mode of the pin
// retval: 16Bit conversion result
uint16_t MAX11300::read_adcRaw(uint8_t pin, uint8_t pinMode)
{
	uint16_t data = read(0x40 + pin);
	if(pinMode == 8 && data & 1<<11)
		data |= 0xF000; // set MSBs
	return data;
}

// reads the ADC conversion result and converts it to relative range output between 0 and 1
// param: pin - number of the pin to be read (0..19)
//        pinMode - ADC mode of the pin
// retval: the relative conversion result
float MAX11300::read_adcRelative(uint8_t pin, uint8_t pinMode)
{
    return (float)read_adcRaw(pin, pinMode) / 4095.0f;
}

// writes an anaolg output value to an DAC configured pin
// param: pin - number of the pin to be read (0..19)
//        value - 16Bit value to be written
void MAX11300::write_dacRaw(uint8_t pin, uint16_t value)
{
    write(0x60 + pin, value);
}

// writes an anaolg output value to an DAC configured pin
// param: pin - number of the pin to be read (0..19)
//        value - relative value between 0 and 1
void MAX11300::write_dacRelative(uint8_t pin, float value)
{
    if(value > 1.0f)
        value = 1.0f;

    uint16_t raw = (uint16_t)((value * 4096) + 0.5f);

	write_dacRaw(pin,raw);
}

// sets or clears the given digital output pin
// param: pin - number of the pin to be read (0..19)
//        value - is either 0 to clear the pin to low level or non zero to set the pin to high level
void MAX11300::setPin(uint8_t pin, uint8_t value)
{
	if(value)
		gpio_state |= 1 << pin;
	else
		gpio_state &= ~(1 << pin);

	if(pin <= 15)
    {
        write(0x0D, (uint16_t)(gpio_state & 0xFF));
    }
	else
    {
        write(0x0E, (uint16_t)((gpio_state >> 16) & 0x0F));
    }
}

// reads a digital input pin and return the state
// param: pin - number of the pin to be read (0..19)
// retval: either 1 if pin is at high level or 0 if pin is at low level or vice versa if pin config was set to invert the input
uint8_t MAX11300::getPin(uint8_t pin)
{
	if(pin <= 15)
    {
		return (read(0x0B) & (1<<pin)) >> pin;
	}
	else
	{
		return (read(0x0C) & (1<<(pin-16))) >> (pin-16);
	}
}

// reads the temperature value of the given channel
// param: channel - channel to be read (internal(0), ch1(1) or ch2(2))
// retval: the measured temperature in °C
float MAX11300::read_temperature(uint8_t channel)
{
    return  ((float)read(0x08 + channel)) * 0.125f;
}
