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
#include  <stdbool.h>
//(2): Constants definition
#define ENTRY_MODE							0x04
#define DISPLAY_CONTROL					0x08
#define DISPLAYandCURSOR_SHIFT	0x10
//(3): Functions prototypes
//---- Private Functions ----//
// Funciton(1): Write command word to LCD
static void writeCommand(uint8_t command);
// Function(2): RS ans E pins
static void RS_pin(bool state);
static void E_pin(bool state);
// Function(3): Switch mode of operation
static void switchMode(bool Mode);
// Function(4): Write a single Character to the display
static void LCD_writeChar(char text);
static void setDataLength(bool length);
// Functions(5): convert to string assist functions (used by itoa and ftoa)
static void reverse(char s[]);
static int intToStr(int x, char str[], int d);
static void reverseF(char *str, int len);

//---- Public Functions ----//
// Function(1): Begin 8-bits mode of operation
void LCD_begin8BIT(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_LSBs0to4, uint16_t D0, uint16_t D1, uint16_t D2, uint16_t D3, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
// Function(2): Begin 4-bits mode of operation
void LCD_begin4BIT(GPIO_TypeDef* PORT_RS_E, uint16_t RS, uint16_t E, GPIO_TypeDef* PORT_MSBs4to7, uint16_t D4, uint16_t D5, uint16_t D6, uint16_t D7);
// Function(3): Print to Display
void LCD_print(char string[]);
// Function(4): Clear display
void LCD_clear(void);
// Function(5): Set Cursor position
void LCD_setCursor(uint8_t row, uint8_t col);
// Function(6): Enable two lines
void LCD_TwoLines(void);
void LCD_OneLine(void);
// Function(7): Cursor ON/OFF
void LCD_noCursor(void);
void LCD_cursor(void);
// Function(8): Blinking cursor
void LCD_noBlink(void);
void LCD_blink(void);
// Function(9): Display ON/OFF
void LCD_noDisplay(void);
void LCD_display(void);
// Function(10): Shift Display, right or left
void LCD_shiftToRight(uint8_t num);
void LCD_shiftToLeft(uint8_t num);
// Function(11): itoa and ftoa to convert from int and float to string
void LCD_itoa(int n, char s[]);
void LCD_ftoa(float n, char *res, int afterpoint);
// Function(12): Write single ASCII character to LCD
void LCD_printChar(char Char);
// Function(13): Integer to string
void LCD_int2str(int value); // int = 4 byte
// Function(14): Float to string
void LCD_float2str(float value,unsigned char num_dec);
