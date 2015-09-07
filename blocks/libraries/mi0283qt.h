#ifndef _MI0283QT_H_
#define _MI0283QT_H_

//Header
#include "mbed.h"
#include "cmd.h"

//Defines
#define SET_HIGH			1
#define SET_LOW				0
#define LCD_WIDTH			(320)
#define LCD_HEIGHT			(240)
#define MI0283QT_FREQ		(8*1000*1000)
#define MI0283QT_FRAME_SIZE	8

#define RGB(r,g,b)			(((r&0xF8)<<8)|((g&0xFC)<<3)|((b&0xF8)>>3))
#define COLOR_WHITE			RGB(255,255,255)
#define COLOR_RED			RGB(255,0,0)
#define COLOR_GREEN			RGB(0,255,0)
#define COLOR_BLUE			RGB(0,0,255)
#define COLOR_YELLOW		RGB(255,255,0)
#define COLOR_MAGENTA		RGB(255,0,255)
#define COLOR_CYAN			RGB(0,255,255)
#define COLOR_BLACK			RGB(0,0,0)

#define LCD_LEVEL_FG		(true)
#define LCD_LEVEL_BG		(false)
#define LCD_BRIGHT			(75)

//Prototypen
int spi_test(SPI&, DigitalOut&, DigitalOut&);
int lcd_init(SPI&, DigitalOut&, DigitalOut&);
int lcd_testScreen(SPI&, DigitalOut&);
int lcd_getWidthHight(SPI&, DigitalOut&, uint16_t*, uint16_t*);
int lcd_backlight(SPI&, DigitalOut&, uint16_t);
int lcd_clearScreen(SPI&, DigitalOut&);
int lcd_drawRect(SPI&, DigitalOut&, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, bool);
int lcd_drawFillRect(SPI&, DigitalOut&, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, bool);
int lcd_writePosition(SPI&, uint16_t);
int lcd_drawText(SPI&, DigitalOut&, uint16_t, uint16_t, char*, uint16_t, bool);
int lcd_setBackground(SPI&, DigitalOut&, uint16_t);
int lcd_setColor(SPI&, DigitalOut&, uint16_t, bool);
int lcd_drawLine(SPI&, DigitalOut&, uint16_t, uint16_t, uint16_t, uint16_t, uint16_t, bool);

#endif //_MI0283QT_H_
