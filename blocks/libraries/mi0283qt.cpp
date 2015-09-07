#include "mi0283qt.h"

int spi_test(SPI &spi, DigitalOut &cs, DigitalOut &rst)
{
	uint8_t i = 0;
	cs = 0;
	while (1)
	{
		cs = 1;
		spi.write(i++);
		wait_us(100);
		cs = 0;
	}
	return 0;
}

int lcd_init(SPI &spi, DigitalOut &cs, DigitalOut &rst)
{
	// Reset
	rst = SET_LOW;
	wait_ms(10);
	rst = SET_HIGH;

	// Activate
	cs = SET_LOW;
	spi.write(0);
	wait_ms(800);
	cs = SET_HIGH;
	wait_ms(10);

	return 0;
}

int lcd_testScreen(SPI &spi, DigitalOut &cs)
{
	uint16_t x0 = 0;
	uint16_t y0 = 0;
	uint16_t xmax;
	uint16_t ymax;
	char string[] = "Testscreen";
	uint16_t strLen = strlen(string);

	lcd_getWidthHight(spi, cs, &xmax, &ymax);
	lcd_clearScreen(spi, cs);

	lcd_setBackground(spi, cs, COLOR_WHITE);
	lcd_drawText(spi, cs, (xmax/2)-(3*strLen), ymax/2, string, COLOR_BLACK, LCD_LEVEL_FG);
	lcd_drawRect(spi, cs, x0+30, y0+30, xmax-30, ymax-30, COLOR_BLACK, LCD_LEVEL_FG);
	wait(1);

	lcd_clearScreen(spi, cs);
	lcd_setBackground(spi, cs, COLOR_RED);
	lcd_drawText(spi, cs, (xmax/2)-(3*3), ymax/2, "Rot", COLOR_WHITE, LCD_LEVEL_FG);
	wait(1);

	lcd_clearScreen(spi, cs);
	lcd_setBackground(spi, cs, COLOR_GREEN);
	lcd_drawText(spi, cs, (xmax/2)-(3*5), ymax/2, "Gruen", COLOR_WHITE, LCD_LEVEL_FG);
	wait(1);

	lcd_clearScreen(spi, cs);
	lcd_setBackground(spi, cs, COLOR_BLUE);
	lcd_drawText(spi, cs, (xmax/2)-(3*4), ymax/2, "Blau", COLOR_WHITE, LCD_LEVEL_FG);
	wait(1);

	return 0;
}

int lcd_getWidthHight(SPI &spi, DigitalOut &cs, uint16_t* x, uint16_t* y)
{
	cs = SET_LOW;

	//Width
	spi.write(CMD_LCD_WIDTH);
	wait_ms(10);
	*x = spi.write(0xFF)<<8;
	*x |= spi.write(0xFF);

	//Hight
	spi.write(CMD_LCD_HEIGHT);
	wait_ms(10);
	*y = spi.write(0xFF)<<8;
	*y |= spi.write(0xFF);

	cs = SET_HIGH;
	return 0;
}
int lcd_backlight(SPI &spi, DigitalOut &cs, uint16_t bright)
{
	cs = SET_LOW;

	spi.write(CMD_LCD_LED);
	spi.write(bright);

	cs = SET_HIGH;
	return 0;
}

int lcd_clearScreen(SPI &spi, DigitalOut &cs)
{
	lcd_backlight(spi, cs, LCD_BRIGHT);

	cs = SET_LOW;
	spi.write(CMD_LCD_CLEARBG);
	cs = SET_HIGH;
	return 0;
}

int lcd_drawRect(SPI &spi, DigitalOut &cs, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, bool level)
{
	lcd_setColor(spi, cs, color, level);

	cs = SET_LOW;

	if(level == LCD_LEVEL_FG)
	{
		spi.write(CMD_LCD_DRAWRECTFG);
	}
	else
	{
		spi.write(CMD_LCD_DRAWRECTBG);
	}
	lcd_writePosition(spi, x0);
	lcd_writePosition(spi, y0);
	lcd_writePosition(spi, x1-x0);
	lcd_writePosition(spi, y1-y0);

	cs = SET_HIGH;
	return 0;
}

int lcd_writePosition(SPI &spi, uint16_t pos)
{
	spi.write(pos>>8);
	spi.write(pos);
	return 0;
}

int lcd_drawText(SPI &spi, DigitalOut &cs, uint16_t x, uint16_t y, char* string, uint16_t color, bool level)
{
	int i;
	uint8_t strLen = strlen(string);

	lcd_setColor(spi, cs, color, level);

	cs = SET_LOW;

	if(level == LCD_LEVEL_FG)
	{
		spi.write(CMD_LCD_DRAWTEXTFG);
	}
	else
	{
		spi.write(CMD_LCD_DRAWTEXTBG);
	}
	lcd_writePosition(spi, x);
	lcd_writePosition(spi, y);
	spi.write(1);
	spi.write(strLen);

	for(i=0; i<strLen; i++)
	{
		spi.write(string[i]);
	}

	cs = SET_HIGH;
	return 0;
}

int lcd_setBackground(SPI &spi, DigitalOut &cs, uint16_t color)
{
	lcd_setColor(spi, cs, color, LCD_LEVEL_BG);

	cs = SET_LOW;

	spi.write(CMD_LCD_FILLRECTBG);
	lcd_writePosition(spi, 0);
	lcd_writePosition(spi, 0);
	lcd_writePosition(spi, LCD_WIDTH);
	lcd_writePosition(spi, LCD_HEIGHT);

	cs = SET_HIGH;
	return 0;
}

int lcd_setColor(SPI &spi, DigitalOut &cs, uint16_t color, bool level)
{
	cs = SET_LOW;

	if(level == LCD_LEVEL_FG)
	{
		spi.write(CMD_LCD_FGCOLOR);
	}
	else
	{
		spi.write(CMD_LCD_BGCOLOR);
	}

	spi.write(color>>8);
	spi.write(color);

	cs = SET_HIGH;
	return 0;
}

int lcd_drawLine(SPI &spi, DigitalOut &cs, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, bool level)
{
	lcd_setColor(spi, cs, color, level);

	cs = SET_LOW;

	if(level == LCD_LEVEL_FG)
	{
		spi.write(CMD_LCD_DRAWLINEFG);
	}
	else
	{
		spi.write(CMD_LCD_DRAWLINEBG);
	}
	lcd_writePosition(spi, x0);
	lcd_writePosition(spi, y0);
	lcd_writePosition(spi, x1);
	lcd_writePosition(spi, y1);

	cs = SET_HIGH;
  return 0;
}

int lcd_drawFillRect(SPI &spi, DigitalOut &cs, uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color, bool level)
{
	lcd_setColor(spi, cs, color, level);

	cs = SET_LOW;

	if(level == LCD_LEVEL_FG)
	{
		spi.write(CMD_LCD_FILLRECTFG);
	}
	else
	{
		spi.write(CMD_LCD_FILLRECTBG);
	}
	lcd_writePosition(spi, x0);
	lcd_writePosition(spi, y0);
	lcd_writePosition(spi, x1-x0);
	lcd_writePosition(spi, y1-y0);

	cs = SET_HIGH;
	return 0;
}
/*
int lcd_drawPoint(SPI &spi, DigitalOut &cs, uint16_t color, bool level)
{
}
*/
