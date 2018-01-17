/*
		Library: 			Liquid Crystal Display (LCD) 16X2
		Written by:  	Mohamed Yaqoob
		Date:				 	06/04/2016
		Modified:				18/11/2016
		Description:	This is a library for the standard 16X2 LCD display, for the STM32F4xx series.
									It perfroms the basic Text printing to your 16X2 LCD and is capable of operating
									in 8 bits and 4 bits modes of operation.
		References**:
									This was written based on the open source Arduino LiquidCrystal library
									and by referring to the DATASHEET of the LCD16X2, also with the help of
									the following YouTube tutorials on LCD 16X2:
									(1): 'RC Tractor Guy' YouTube tutorial on the following link:
									     https://www.youtube.com/watch?v=efi2nlsvbCI
									(2): 'Explore Embedded' YouTube tutorial on the following link:
											 https://www.youtube.com/watch?v=YDJISiPUdA8
*/

//(1): Header files includes
#include "stm32f1xx_hal.h"
#include "STM_MY_LCD16X2.h"
#include  <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
//(2): Variables definition
//---- Private Variables ----//
static bool Mode_8BIT;														// 1: 8 bits, 0: 4bits
static GPIO_TypeDef* PORT_RS_and_E;							  // RS and E PORT
static uint16_t PIN_RS, PIN_E;									  // RS and E pins
static GPIO_TypeDef* PORT_LSB;										// LSBs D0, D1, D2 and D3 PORT
static uint16_t D0_PIN, D1_PIN, D2_PIN, D3_PIN;	// LSBs D0, D1, D2 and D3 pins
static GPIO_TypeDef* PORT_MSB;										// MSBs D5, D6, D7 and D8 PORT
static uint16_t D4_PIN, D5_PIN, D6_PIN, D7_PIN;	// MSBs D5, D6, D7 and D8 pins
static uint8_t DisplayControl = 0x0F;
static uint8_t FunctionSet = 0x38;
static uint32_t FreqClockSys=100;
//---- Public Variables ----//

//(3): Functions definitions
//---- Private Functions ----//
// Funcion(2): RS and E pins
static void RS_pin(bool state)
{
	if(state)
	{
		HAL_GPIO_WritePin(PORT_RS_and_E, PIN_RS, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(PORT_RS_and_E, PIN_RS, GPIO_PIN_RESET);
	}
}
static void E_pin(bool state)
{
	if(state)
	{
		HAL_GPIO_WritePin(PORT_RS_and_E, PIN_E, GPIO_PIN_SET);
	}
	else
	{
		HAL_GPIO_WritePin(PORT_RS_and_E, PIN_E, GPIO_PIN_RESET);
	}
}
// Function(1): Write command
static void writeCommand(uint8_t command)
{
	uint8_t LSB_data, MSB_data;
	LSB_data = 0x0F&command;
	MSB_data = 0x0F&(command>>4);
	if(Mode_8BIT)
	{
		// Reset pins from D0 to D3
		PORT_LSB->ODR &= ~((1UL<<D0_PIN)|(1UL<<D1_PIN)|(1UL<<D2_PIN)|(1UL<<D3_PIN));
		// Write new command to D0 to D3
		PORT_LSB->ODR |= (((LSB_data>>0)&1UL)<<D0_PIN);
		PORT_LSB->ODR |= (((LSB_data>>1)&1UL)<<D1_PIN);
		PORT_LSB->ODR |= (((LSB_data>>2)&1UL)<<D2_PIN);
		PORT_LSB->ODR |= (((LSB_data>>3)&1UL)<<D3_PIN);
		// Reset pins from D4 to D7
		PORT_MSB->ODR &= ~((1UL<<D4_PIN)|(1UL<<D5_PIN)|(1UL<<D6_PIN)|(1UL<<D7_PIN));
		// Write new command to D4 to D7
		PORT_MSB->ODR |= (((MSB_data>>0)&1UL)<<D4_PIN);
		PORT_MSB->ODR |= (((MSB_data>>1)&1UL)<<D5_PIN);
		PORT_MSB->ODR |= (((MSB_data>>2)&1UL)<<D6_PIN);
		PORT_MSB->ODR |= (((MSB_data>>3)&1UL)<<D7_PIN);
		for(uint32_t i=0; i<30; i++);
		E_pin(true);
		for(uint32_t i=0; i<FreqClockSys; i++);
		E_pin(false);
	}
	else
	{
		// Reset pins from D4 to D7
		PORT_MSB->ODR &= ~((1UL<<D4_PIN)|(1UL<<D5_PIN)|(1UL<<D6_PIN)|(1UL<<D7_PIN));
		//PORT_LSB->ODR &= ~((1UL<<D0_PIN)|(1UL<<D1_PIN)|(1UL<<D2_PIN)|(1UL<<D3_PIN));
		// Write new command to D4 to D7
		PORT_MSB->ODR |= (((MSB_data>>0)&1UL)<<D4_PIN);
		PORT_MSB->ODR |= (((MSB_data>>1)&1UL)<<D5_PIN);
		PORT_MSB->ODR |= (((MSB_data>>2)&1UL)<<D6_PIN);
		PORT_MSB->ODR |= (((MSB_data>>3)&1UL)<<D7_PIN);
		for(uint32_t i=0; i<10; i++);
		E_pin(true);
		for(uint32_t i=0; i<FreqClockSys; i++);
		E_pin(false);
		//for(uint32_t i=0; i<70; i++);
		// Reset pins from D4 to D7
		PORT_MSB->ODR &= ~((1UL<<D4_PIN)|(1UL<<D5_PIN)|(1UL<<D6_PIN)|(1UL<<D7_PIN));
		// Write new command to D4 to D7
		PORT_MSB->ODR |= (((LSB_data>>0)&1UL)<<D4_PIN);
		PORT_MSB->ODR |= (((LSB_data>>1)&1UL)<<D5_PIN);
		PORT_MSB->ODR |= (((LSB_data>>2)&1UL)<<D6_PIN);
		PORT_MSB->ODR |= (((LSB_data>>3)&1UL)<<D7_PIN);
		for(uint32_t i=0; i<10; i++);
		E_pin(true);
		for(uint32_t i=0; i<FreqClockSys; i++);
		E_pin(false);
	}
}

// Funciton(3): Change mode
static void switchMode(bool Mode)
{
	RS_pin(false);
	HAL_Delay(20);
	writeCommand(0x30);
	HAL_Delay(5);
	writeCommand(0x30);
	HAL_Delay(5);
	writeCommand(0x30);
	HAL_Delay(1);
	if(Mode)
	{
		writeCommand(0x30);
	}
	else
	{
		writeCommand(0x20);
	}
	HAL_Delay(1);
}
// Function(4): Write a single Character to the display
static void LCD_writeChar(char text)
{
	RS_pin(true);
	writeCommand(text);
}
static void setDataLength(bool length)
{
	RS_pin(false);
	if(length)
	{
		FunctionSet |= (0x10);
		writeCommand(FunctionSet);
	}
	else
	{
		FunctionSet &= ~(0x10);
		writeCommand(FunctionSet);
	}
}
// Function(5): itoa and ftoa assist functions
static void reverse(char s[])
{
     int i, j;
     char c;

     for (i = 0, j = strlen(s)-1; i<j; i++, j--) {
         c = s[i];
         s[i] = s[j];
         s[j] = c;
     }
}
static int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverseF(str, i);
    str[i] = '\0';
    return i;
}

static void reverseF(char *str, int len)
{
	  int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
//---- Public Functions ----//
// Funciton(1): LCD_begin8BIT
void LCD_begin8BIT(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_LSBs0to4, uint16_t D0, uint16_t D1, uint16_t D2, uint16_t D3, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7)
{
	FreqClockSys = HAL_RCC_GetSysClockFreq()/160000;
	Mode_8BIT = true; 					// 8 bits mode is enabled
	PORT_RS_and_E = PORT_RS_E;
	PIN_RS = (1UL<<RS);
	PIN_E = (1UL<<E);
	PORT_LSB = PORT_LSBs0to4;
	D0_PIN = D0;
	D1_PIN = D1;
	D2_PIN = D2;
	D3_PIN = D3;
	PORT_MSB = PORT_MSBs4to7;
	D4_PIN = D4;
	D5_PIN = D5;
	D6_PIN = D6;
	D7_PIN = D7;
	switchMode(true);
	setDataLength(true);
	writeCommand(0x0F);
	HAL_Delay(2);
	LCD_clear();
}
void LCD_begin4BIT(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7)
{
	FreqClockSys = HAL_RCC_GetSysClockFreq()/160000;
	Mode_8BIT = false; 					// 8 bits mode is disabled
	PORT_RS_and_E = PORT_RS_E;
	PIN_RS = (1UL<<RS);
	PIN_E = (1UL<<E);
	PORT_MSB = PORT_MSBs4to7;
	D4_PIN = D4;
	D5_PIN = D5;
	D6_PIN = D6;
	D7_PIN = D7;
	// Switch to 4bits mode
	HAL_Delay(20);
	switchMode(false);
	setDataLength(false);
	writeCommand(0x0F);
	HAL_Delay(2);
	writeCommand(0x0F);
	HAL_Delay(2);
	LCD_clear();
	LCD_TwoLines();
}
// Function(3): Print to Display
void LCD_print(char string[])
{
	/*
	for(uint8_t i=0;  i< 16 && string[i]!=NULL; i++)
	{
		LCD_writeChar(string[i]);
	}
	*/
	uint8_t i=0;
	while(i< 16 && string[i]!=NULL)
	{
		LCD_writeChar(string[i]);
		++i;
	}
}
// Function(4): Clear display
void LCD_clear(void)
{
	RS_pin(false);
	writeCommand(0x01);
	HAL_Delay(2);
}
// Function (5): Set Cursor position
void LCD_setCursor(uint8_t row, uint8_t col)
{
	uint8_t maskData;
	RS_pin(false);
	maskData = (col-1)&0x0F;
	if(row==1)
	{
		maskData |= (0x80);
		writeCommand(maskData);
	}
	else
	{
		maskData |= (0xc0);
		writeCommand(maskData);
	}
}
// Function(6): Enable two lines
void LCD_TwoLines(void)
{
	RS_pin(false);
	FunctionSet |= (0x08);
	writeCommand(FunctionSet);
}
void LCD_OneLine(void)
{
	RS_pin(false);
	FunctionSet &= ~(0x08);
	writeCommand(FunctionSet);
}
// Function(7): Blinking cursor
void LCD_noBlink(void)
{
	RS_pin(false);
	DisplayControl &= ~(0x01);
	writeCommand(DisplayControl);
}
void LCD_blink(void)
{
	RS_pin(false);
	DisplayControl |= 0x01;
	writeCommand(DisplayControl);
}
// Function(8): Display ON/OFF
void LCD_noDisplay(void)
{
	RS_pin(false);
	DisplayControl &= ~(0x04);
	writeCommand(DisplayControl);
}
void LCD_display(void)
{
	RS_pin(false);
	DisplayControl |= (0x04);
	writeCommand(DisplayControl);
}
// Function(9): Cursor ON/OFF
void LCD_noCursor(void)
{
	RS_pin(false);
	DisplayControl &= ~(0x03);
	writeCommand(DisplayControl);
}
void LCD_cursor(void)
{
	RS_pin(false);
	DisplayControl |= (0x03);
	writeCommand(DisplayControl);
}
// Function(10): Shift Display or Cursor, right or left
void LCD_shiftToRight(uint8_t num)
{
	RS_pin(false);
	for(uint8_t i=0; i<num;i++)
	{
		writeCommand(0x1c);
	}
}
void LCD_shiftToLeft(uint8_t num)
{
	RS_pin(false);
	for(uint8_t i=0; i<num;i++)
	{
		writeCommand(0x18);
	}
}
// Function(11): itoa and ftoa to convert from int and float to string
void LCD_itoa(int n, char s[])
{
     int i, sign;

     if ((sign = n) < 0)  /* record sign */
         n = -n;          /* make n positive */
     i = 0;
     do {       /* generate digits in reverse order */
         s[i++] = n % 10 + '0';   /* get next digit */
     } while ((n /= 10) > 0);     /* delete it */
     if (sign < 0)
         s[i++] = '-';
     s[i] = '\0';
     reverse(s);
}
void LCD_ftoa(float n, char *res, int afterpoint)// n = float_number, res[] = array store float string, afterpoint  number of decimal
{
		if(n<0)
		{
		  n = -n;
		  LCD_print("-");
	  }
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 1);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}
// Function(12): Write single ASCII character to LCD
void LCD_printChar(char Char)
{
	LCD_writeChar(Char);
}
// Function(13): Integer to string
void LCD_int2str(int value)
{
	 unsigned char sign=0,digit[5],i;
	
	if(value<0)
	{
		value = -value;
		sign = 1;
	}
	 for(i=0; value!=0; ++i)
	 {
		 digit[i] = value%10 + 0x30;
		 value /= 10;
	 }
	 if(sign == 1)
	 {
		 LCD_print("-");
	 }
	 do{
		 LCD_writeChar(digit[i-1]);
		 i -= 1;
	 }while(i>0);	 
}
