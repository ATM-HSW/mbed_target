#include "lcd_scope.h"

uint16_t signalColors[] = {COLOR_YELLOW, COLOR_MAGENTA, COLOR_CYAN, COLOR_RED, COLOR_GREEN};

int lcd_drawScope(SPI &spi, DigitalOut &cs, uint16_t timeRange, int16_t yMin, int16_t yMax)
{
	lcd_drawScopeTemplate(spi, cs, yMin, yMax);
	lcd_drawScopeText(spi, cs, 0, timeRange, yMin, yMax);
	return 0;
}

int lcd_drawScopeTemplate(SPI &spi, DigitalOut &cs, int16_t yMin, int16_t yMax)
{
	lcd_clearScreen(spi, cs);
	lcd_setBackground(spi, cs, COLOR_WHITE);
	lcd_drawFillRect(spi, cs, 0, LCD_HEIGHT-X_BORDER, LCD_WIDTH, LCD_HEIGHT-X_BORDER, COLOR_BLACK, LCD_LEVEL_FG);
	lcd_drawFillRect(spi, cs, Y_BORDER, 0, Y_BORDER, LCD_HEIGHT, COLOR_BLACK, LCD_LEVEL_FG);

	return 0;
}

int lcd_drawScopeText(SPI &spi, DigitalOut &cs, uint16_t minTime, uint16_t maxTime, int16_t yMin, int16_t yMax)
{
	char sMinTime[15], sMaxTime[15], sMinValue[15], sMaxValue[15];
	char sFacMinTime[15], sFacMaxTime[15], sFacMinValue[15], sFacMaxValue[15];
	float shortMinTime, shortMaxTime, shortMinValue, shortMaxValue;
	uint16_t facMinTime, facMaxTime, facMinValue, facMaxValue;

	facMinTime = log10(abs(minTime));
	facMaxTime = log10(abs(maxTime));
	facMinValue = log10(abs(yMin));
	facMaxValue = log10(abs(yMax));

	shortMinTime = minTime/pow(CALC_EXP,facMinTime);
	shortMaxTime = maxTime/pow(CALC_EXP,facMaxTime);
	shortMinValue = yMin/pow(CALC_EXP,facMinValue);
	shortMaxValue = yMax/pow(CALC_EXP,facMaxValue);

	ftoa(shortMinTime, sMinTime);
	ftoa(shortMaxTime, sMaxTime);
	ftoa(shortMinValue, sMinValue);
	ftoa(shortMaxValue, sMaxValue);
  
	sprintf(sFacMinTime, "1e%d", facMinTime);
	sprintf(sFacMaxTime, "1e%d", facMaxTime);
	sprintf(sFacMinValue, "1e%d", facMinValue);
	sprintf(sFacMaxValue, "1e%d", facMaxValue);

	// Draw X
	lcd_drawText(spi, cs, Y_BORDER+5, LCD_HEIGHT-X_BORDER+1, sMinTime, COLOR_BLACK, LCD_LEVEL_FG);
	lcd_drawText(spi, cs, LCD_WIDTH-(strlen(sMaxTime)*TEXT_WIDTH), LCD_HEIGHT-X_BORDER+1, sMaxTime, COLOR_BLACK, LCD_LEVEL_FG);

		// Exponent
		if(facMinTime > 0)lcd_drawText(spi, cs, Y_BORDER+5, LCD_HEIGHT-X_BORDER+TEXT_HIGH, sFacMinTime, COLOR_BLACK, LCD_LEVEL_FG);
		if(facMaxTime > 0)lcd_drawText(spi, cs, LCD_WIDTH-(strlen(sFacMaxTime)*TEXT_WIDTH), LCD_HEIGHT-X_BORDER+TEXT_HIGH, sFacMaxTime, COLOR_BLACK, LCD_LEVEL_FG);

	// Draw Y
	lcd_drawText(spi, cs, Y_BORDER-(strlen(sMinValue)*TEXT_WIDTH), LCD_HEIGHT-X_BORDER+1, sMinValue, COLOR_BLACK, LCD_LEVEL_FG);
	lcd_drawText(spi, cs, Y_BORDER-(strlen(sMaxValue)*TEXT_WIDTH), 1, sMaxValue, COLOR_BLACK, LCD_LEVEL_FG);

		// Exponent
		if(facMinValue > 0)lcd_drawText(spi, cs, Y_BORDER-(strlen(sFacMinValue)*TEXT_WIDTH), LCD_HEIGHT-X_BORDER+TEXT_HIGH, sFacMinValue, COLOR_BLACK, LCD_LEVEL_FG);
		if(facMaxValue > 0)lcd_drawText(spi, cs, Y_BORDER-(strlen(sFacMaxValue)*TEXT_WIDTH), 1+TEXT_HIGH, sFacMaxValue, COLOR_BLACK, LCD_LEVEL_FG);

	return 0;
}

int lcd_drawScopeValue(SPI &spi, DigitalOut &cs, uint16_t time, int16_t* value, uint16_t timeRange, int16_t yMin, int16_t yMax, uint16_t scopeNumber)
{
	int valueIndex;

	uint16_t x_act;
	static uint16_t x_last = Y_BORDER;

	uint16_t y_act[MAX_SCOPE_VALUE_NUMBER];
	static uint16_t y_last[MAX_SCOPE_VALUE_NUMBER];// = calc_valueToPoint(value, yMin, yMax);

	static uint16_t time_overrun = 0;

	// Calc Points
	x_act = calc_timeToPoint(time, timeRange, time_overrun);

	// Draw
	for(valueIndex=0;valueIndex<scopeNumber;valueIndex++)
	{
		y_act[valueIndex] = calc_valueToPoint(value[valueIndex], yMin, yMax);
		lcd_drawLine(spi, cs, x_last, y_last[valueIndex], x_act, y_act[valueIndex], signalColors[valueIndex], LCD_LEVEL_FG);
	}

	// Check Overrun
	if((x_act) > LCD_WIDTH)
	{
		time_overrun++;

		lcd_drawScope(spi, cs, timeRange, yMin, yMax);
		lcd_drawScopeLegend(spi, cs, scopeNumber);
		x_last = Y_BORDER;
	}
	else
	{
		x_last = x_act;
	}
	calc_copyArray(y_act, y_last, scopeNumber);
	return 0;
}

uint16_t calc_timeToPoint(uint16_t time, uint16_t timeRange, uint16_t time_overrun)
{
	uint16_t x_point;

	// Check Time
	time = time - time_overrun*timeRange;

	// Calc
	x_point = ((time*(LCD_WIDTH-Y_BORDER))/(timeRange))+Y_BORDER;

	return x_point;
}

uint16_t calc_valueToPoint(int16_t value, int16_t yMin, int16_t yMax)
{
	int16_t disp_offset;
	uint16_t y_point;

	// Check Value
	if (value > yMax)
		{value = yMax;}
	else if(value < yMin)
		{value = yMin;}

	// Calc
	disp_offset = (yMin*(LCD_HEIGHT-X_BORDER))/(yMax-yMin);
	y_point = LCD_HEIGHT-X_BORDER-((value*(LCD_HEIGHT-X_BORDER))/(yMax-yMin))+disp_offset;

	return y_point;
}

int calc_copyArray(uint16_t* a, uint16_t* b, uint16_t maxRun)
{
	int run;
	for (run = 0; run < maxRun; run++)
	{
		b[run] = a[run];
	}
	return 0;
}

uint16_t calc_Time(uint16_t deltaTime)
{
	static uint16_t time = 0;
	return time = time + deltaTime;
}

int ftoa(float num, char* string)
{
	int d1 = num;
	float f2 = num - d1;
	int d2 = trunc(f2 * pow(10,FLOAT_DECIMAL_PLACES));

	sprintf(string, "%d.%01d", d1, abs(d2));
	return 0;
}

int lcd_drawScopeLegend(SPI &spi, DigitalOut &cs, uint16_t scopeNumber)
{
	int valueIndex;
	char tempSignalName[5];

	for(valueIndex=0;valueIndex<scopeNumber;valueIndex++)
	{
		sprintf(tempSignalName, "%d", valueIndex+1);
		lcd_drawText(spi, cs, LCD_WIDTH-(strlen(tempSignalName)*TEXT_WIDTH), 1+(valueIndex*TEXT_WIDTH), tempSignalName, signalColors[valueIndex], LCD_LEVEL_FG);
	}
	return 0;
}
