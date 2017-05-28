/* Copyright 2015 M. Marquardt, HS Wismar */

#ifndef MAX11300_H
#define MAX11300_H

#include "mbed.h"

#define MAX11300_CTL_REG 0x10

#define MAX11300_ADCCTL_CONTINOUS (3)
#define MAX11300_DACCTL_SEQUENTIUAL (0<<2)
#define MAX11300_TEMP_INTERNAL (1)
#define MAX11300_TEMP_CH1      (2)
#define MAX11300_TEMP_CH2      (4)

#define MAX11300_ADCRATE_200KSPS (0)
#define MAX11300_ADCRATE_250KSPS (1)
#define MAX11300_ADCRATE_333KSPS (2)
#define MAX11300_ADCRATE_400KSPS (3)

#define MAX11300_RANGE_0to10V    (1)
#define MAX11300_RANGE_5to5V     (2)
#define MAX11300_RANGE_10to0V    (3)

#define MAX11300_DACREF_EXTERNAL (0)
#define MAX11300_DACREF_INTERNAL (1)

#define MAX11300_PORT_MODE_HI_Z  (0)    // High Impedance
#define MAX11300_PORT_MODE_GPI   (1)    // Digital Input
#define MAX11300_PORT_MODE_GPO   (3)    // Digital Output
#define MAX11300_PORT_MODE_DAC   (5)    // Analog Output
#define MAX11300_PORT_MODE_ADC_SINLGE (7) // Single ended analog input
#define MAX11300_PORT_MODE_ADC_POS (8)  // positive port of differential analog input
#define MAX11300_PORT_MODE_ADC_NEG (9)  // negative port of differential analog input

#define MAX11300_ADCREF_EXTERNAL (1)
#define MAX11300_ADCREF_INTERNAL (0)

#define MAX11300_GPI_INVERT      (1)
#define MAX11300_GPI_NONEINVERT  (0)

/****************************************
 * MAX11300 Class
 ***************************************/

class MAX11300
{
private:
	SPI *_spi;
	DigitalOut *cs;
	uint32_t gpio_state;

	uint16_t read(uint8_t reg);

	void write(uint8_t reg, uint16_t data);

public:
	// constructor
	MAX11300(SPI *spi, PinName pin);

	void config_device(uint8_t adcRate, uint8_t dacRef, uint8_t temp_channel);

	void config_pin(uint8_t pin, uint8_t mode, uint8_t associatedPort, uint8_t powerOfSamples, uint8_t range, bool adcRefExternal, bool invertInput);

    uint16_t read_adcRaw(uint8_t pin, uint8_t pinMode);

	float read_adcRelative(uint8_t pin, uint8_t pinMode);

	void write_dacRaw(uint8_t pin, uint16_t value);

	void write_dacRelative(uint8_t pin, float value);

	void setPin(uint8_t pin, uint8_t value);

	uint8_t getPin(uint8_t pin);

	float read_temperature(uint8_t channel);
};

#endif /* MAX11300_H */
