/*
 * LCD.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */


#include "lcd.h"
#include <string.h>


static void write_4_DR_bits(uint8_t value);
static void lcd_enable(void);

/*
 * delay in milli seconds as per param
 */
void msdelay(uint32_t count)
{
	// Assuming each iteration takes about 1 microsecond
	for (uint32_t i = 0; i < count*1000; i++);
}

/*
 * delay in micro seconds as per param
 */
void udelay(uint32_t count)
{
	for (uint32_t i = 0; i < count*1; i++);
}


/*
 * write the 4 bits to the data register
 * LSB -> MSB
 * right shift each bit to LSB and bit mask to only retain LSB
 */
static void write_4_DR_bits(uint8_t value)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT2, LCD_GPIO_D4, (value & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_D5, ((value >> 1) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_D6, ((value >> 2) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORT2, LCD_GPIO_D7, ((value >> 3) & 0x1));

	// need to make transition on enable pin from high to low after sending each 4 bits
	// see timing diagram
	lcd_enable();
}

static void lcd_enable(void)
{
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_EN, GPIO_PIN_SET);
	udelay(10);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_EN, GPIO_PIN_RESET);
	msdelay(2); // execution time > 37us
}

//RS-PA3
//R/W-PA2
//EN-PA10
//D4-PB3
//D5-PA1
//D6-PA4
//D7-PB0

/*
 * Initialize LCD board
 */
void lcd_init(void)
{
	//1. Configure the gpio pins that are used for lcd connections

	GPIO_Handle_t lcd_signal;

	memset(&lcd_signal, 0, sizeof(lcd_signal));

	lcd_signal.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// start peripheral clock
	GPIO_PeriClockControl(LCD_GPIO_PORT1, ENABLE);
	GPIO_PeriClockControl(LCD_GPIO_PORT2, ENABLE);

	// RS
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT1;
	GPIO_Init(&lcd_signal);


	// R/W
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT1;
	GPIO_Init(&lcd_signal);

	// EN
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT1;
	GPIO_Init(&lcd_signal);

	// D4
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT2;
	GPIO_Init(&lcd_signal);

	// D5
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT1;
	GPIO_Init(&lcd_signal);

	// D6
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT1;
	GPIO_Init(&lcd_signal);

	// D7
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	lcd_signal.p_GPIOx = LCD_GPIO_PORT2;
	GPIO_Init(&lcd_signal);

	// keep all the pins low initally
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RS, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RW, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_EN, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT2, LCD_GPIO_D4, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_D5, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_D6, GPIO_PIN_RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORT2, LCD_GPIO_D7, GPIO_PIN_RESET);

	//2. LCD initialization - look at flow chart in datasheet
	msdelay(40);

	// RS = 0, For LCD instruction command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RS, GPIO_PIN_RESET);

	// RW = 0, Writing to LCD
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RW, GPIO_PIN_RESET);

	write_4_DR_bits(0x3); // 0011

	msdelay(5);

	write_4_DR_bits(0x3); // 0011

	udelay(150);

	write_4_DR_bits(0x3); // 0011

	write_4_DR_bits(0x2); // 0011

	// Custom commands

	// 1. function set
	lcd_send_command(LCD_CMD_4DL_2N_5x8F);

	// 2. display control
	lcd_send_command(LCD_CMD_DON_CURON);

	//3. display clear
	lcd_display_clear();

	//4. entry mode set -> Activate increment address as and when data written,  no shift
	lcd_send_command(LCD_CMD_INCADD);
}

void lcd_display_clear(void)
{
	lcd_send_command(LCD_CMD_DIS_CLEAR);

	// need to wait for 2ms as per user manual
	msdelay(2);
}

void lcd_display_return_home(void)
{
	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

	msdelay(2);
}

/*
 * Send Command instructions to LCD
 */
void lcd_send_command(uint8_t cmd)
{

	// RS = 0, For LCD instruction command
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RS, GPIO_PIN_RESET);

	// RW = 0, Writing to LCD
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RW, GPIO_PIN_RESET);

	// send the upper 4 bits first
	write_4_DR_bits(cmd >> 4);

	// send the lower 4 bits next
	write_4_DR_bits(cmd & 0x0F);

}

/*
 * send data instructions to LCD
 */

void lcd_send_char(uint8_t data)
{
	// RS = 1, For LCD data
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RS, GPIO_PIN_SET);

	// RW = 0, Writing to LCD
	GPIO_WriteToOutputPin(LCD_GPIO_PORT1, LCD_GPIO_RW, GPIO_PIN_RESET);

	// send the upper 4 bits first
	write_4_DR_bits(data >> 4);

	// send the lower 4 bits next
	write_4_DR_bits(data & 0x0F);
}

void lcd_write_string(char *string)
{
	while (*string != '\0')
	{
		lcd_send_char((uint8_t) *string++);

	}
}

/**
  *   Set Lcd to a specified location given by row and column information
  *   Row Number (1 to 2)
  *   Column Number (1 to 16) Assuming a 2 X 16 characters display
  */
void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      // we send the set DDRAM address command
      lcd_send_command((column |= 0x80));
      break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
        lcd_send_command((column |= 0xC0));
      break;
    default:
      break;
  }
}
