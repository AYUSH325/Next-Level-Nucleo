/*
 * DS1307.h
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "STM32f401RE.h"
/*
 * Application configuration items so that we know which
 * I2C peripheral & PIN its connected on the master side
 */
#define DS1307_I2C 				I2C1
#define DS1307_I2C_GPIO_PORT	GPIOB
#define DS1307_I2C_SDA_PIN		GPIO_PIN_NO_7
#define DS1307_I2C_SCL_PIN      GPIO_PIN_NO_6
#define DS1307_I2C_SPEED		I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD			GPIO_PU    // try with internal PUs for OD config

/*
 * Timekeeper register addresses
 */
#define DS1307_ADDR_SEC			0x00
#define DS1307_ADDR_MIN			0x01
#define DS1307_ADDR_HOUR		0x02
#define DS1307_ADDR_DAY			0x03
#define DS1307_ADDR_DATE		0x04
#define DS1307_ADDR_MONTH		0x05
#define DS1307_ADDR_YEAR		0x06

/*
 * Time Format
 */
#define TIME_FORMAT_12HRS_AM	0
#define TIME_FORMAT_12HRS_PM	1
#define TIME_FORMAT_24HRS		2


/*
 * I2C Slave address - from data sheet
 */
#define DS1307_I2C_ADDRESS		0x68

/*
 * Macros for Days
 */
#define SUNDAY					1
#define MONDAY					2
#define TUESDAY					3
#define WEDNESDAY				4
#define THURSDAY				5
#define FRIDAY					6
#define SATURDAY				7

/*
 * value to put in CH field of seconds register to start crystal oscillator
 */
#define DS1307_STARTOSCL		0x00

/*
 * Other bit fields
 */

/*
 * CH bit field in seconds register of DS1307
 */
#define DS1307_SECONDS_CH			7

#define DS1307_HOURS_TIMEFORMATBIT	6
#define DS1307_HOURS_AMPMBIT		5

#define BITS_PER_DIGIT				4

typedef struct
{
	uint8_t date;
	uint8_t month;
	uint8_t year;
	uint8_t day;
} RTC_date_t;

// user will give the values in decimal format, need to convert to BCD
typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
} RTC_time_t;

// Function Prototypes

/*
 * Call to init the ds1307 chip
 */
uint8_t ds1307_init(void);

/*
 * Initialize RTC registers with current time value
 */
void ds1307_set_current_time(RTC_time_t *p_RTC_Time);

/*
 * getter for the current time ticked in register
 */
void ds1307_get_current_time(RTC_time_t *p_RTC_Time);

/*
 * Initialize RTC registers with current date value
 */
void ds1307_set_current_date(RTC_date_t *p_RTC_Date);

/*
 * getter for the current date in register
 */
void ds1307_get_current_date(RTC_date_t *p_RTC_Date);






#endif /* DS1307_H_ */
