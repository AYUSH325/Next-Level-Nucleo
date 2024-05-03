/*
 * LCD.h
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */

#ifndef LCD_H_
#define LCD_H_

#include "STM32f401RE.h"

//RS-PA3
//R/W-PA2
//EN-PA10
//D4-PB3
//D5-PA1
//D6-PA4
//D7-PB0

/*
 * Application configurable items
 */
#define LCD_GPIO_PORT1			GPIOA
#define LCD_GPIO_PORT2			GPIOB
#define LCD_GPIO_RS				GPIO_PIN_NO_3
#define LCD_GPIO_RW				GPIO_PIN_NO_2
#define LCD_GPIO_EN				GPIO_PIN_NO_10
#define LCD_GPIO_D4				GPIO_PIN_NO_3
#define LCD_GPIO_D5				GPIO_PIN_NO_1
#define LCD_GPIO_D6				GPIO_PIN_NO_4
#define LCD_GPIO_D7				GPIO_PIN_NO_0

/*
 * LCD commands
 */
#define LCD_CMD_4DL_2N_5x8F		0x28  // Activate 4 bit data length, 2 display lines on LCD,  5*8 font size
#define LCD_CMD_DON_CURON		0x0E  // Display On & Cursor On
#define LCD_CMD_INCADD			0x06  // Increment RAM address
#define LCD_CMD_DIS_CLEAR		0x01
#define LCD_CMD_DIS_RETURN_HOME	0x02


/*
 * BSP exposed APIS
 */
void lcd_init(void);
void lcd_send_command(uint8_t cmd);
void lcd_send_char(uint8_t data);
void lcd_write_string(char *string);
void lcd_display_clear(void);
void lcd_display_return_home(void);
void lcd_set_cursor(uint8_t row, uint8_t column);
void msdelay(uint32_t count);
void udelay(uint32_t count);


#endif /* LCD_H_ */
