#ifndef _LCD_SCOPE_H_
#define _LCD_SCOPE_H_

// Header
#include "mbed.h"
#include "mi0283qt.h"

// Defines
#define CALC_EXP                10
#define FLOAT_DECIMAL_PLACES	2
#define TEXT_HIGH               10
#define TEXT_WIDTH              9
#define X_BORDER                (2*TEXT_HIGH)
#define Y_BORDER                ((3+FLOAT_DECIMAL_PLACES)*TEXT_WIDTH)
#define MAX_SCOPE_VALUE_NUMBER  5

// Prototypen
int lcd_drawScope(SPI&, DigitalOut&, uint16_t, int16_t, int16_t);
int lcd_drawScopeTemplate(SPI&, DigitalOut&, int16_t, int16_t);
int lcd_drawScopeText(SPI&, DigitalOut&, uint16_t, uint16_t, int16_t, int16_t);
int lcd_drawScopeValue(SPI&, DigitalOut&, uint16_t, int16_t*, uint16_t, int16_t, int16_t, uint16_t);
int lcd_drawScopeLegend(SPI&, DigitalOut&, uint16_t);

uint16_t calc_timeToPoint(uint16_t, uint16_t, uint16_t);
uint16_t calc_valueToPoint(int16_t, int16_t, int16_t);
int calc_copyArray(uint16_t*, uint16_t*, uint16_t);
uint16_t calc_Time(uint16_t);
int ftoa(float, char*);

#endif //_LCD_SCOPE_H_
