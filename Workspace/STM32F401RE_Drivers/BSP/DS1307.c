/*
 * DS1307.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */
#include <stdint.h>
#include <string.h>

#include "DS1307.h"

static void ds1307_I2C_pin_config(void);
static void	ds1307_I2C_config(void);
static void ds1307_write(uint8_t reg_addr, uint8_t value);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t binary_to_bcd(uint8_t value_to_convert);
static uint8_t bcd_to_binary(uint8_t value_to_convert);


I2C_Handle_t ds1307I2CHandle;


static void ds1307_I2C_pin_config(void)
{
	GPIO_Handle_t I2C_SDA, I2C_SCL;

	memset(&I2C_SDA, 0, sizeof(I2C_SDA));
	memset(&I2C_SCL, 0, sizeof(I2C_SCL));

	/*
	 * I2C1.SCL = PB6
	 * I2C1.SDA = PB7
	 */

	I2C_SDA.p_GPIOx = DS1307_I2C_GPIO_PORT;
	I2C_SDA.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_OD;
	I2C_SDA.GPIO_PinConfig.GPIO_PinAltFunMode = ALT_FUN_MODE_4;
	I2C_SDA.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	I2C_SDA.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	I2C_SDA.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// Start peripheral clock
	GPIO_PeriClockControl(DS1307_I2C_GPIO_PORT, ENABLE);

	// Init
	GPIO_Init(&I2C_SDA);

	I2C_SCL.p_GPIOx = DS1307_I2C_GPIO_PORT;
	I2C_SCL.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_OD;
	I2C_SCL.GPIO_PinConfig.GPIO_PinAltFunMode = ALT_FUN_MODE_4;
	I2C_SCL.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	I2C_SCL.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
	I2C_SCL.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	// Init
	GPIO_Init(&I2C_SCL);


}

static void	ds1307_I2C_config(void)
{
	memset(&ds1307I2CHandle, 0, sizeof(ds1307I2CHandle));

	ds1307I2CHandle.p_I2C = DS1307_I2C;
	ds1307I2CHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	ds1307I2CHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_PeriClockControl(DS1307_I2C, ENABLE);

	I2C_Init(&ds1307I2CHandle);
}

/*
 * write the value to a specific register address in the DS1307 module
 */
static void ds1307_write(uint8_t reg_addr, uint8_t value)
{
	uint8_t tx_data[2];
	tx_data[0] = reg_addr;
	tx_data[1] = value;

	I2C_MasterSendData(&ds1307I2CHandle, tx_data, 2, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
}

/*
 * read the data from the DS1307 register
 */
static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t data;

	// First write to the module to intialize the internal pointer to point to the register
	// we want to read from, write the addrss of the register.
	I2C_MasterSendData(&ds1307I2CHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

	// then read the data from that register
	I2C_MasterReceiveData(&ds1307I2CHandle, &data, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

	return data;
}

/*
 * Converts the given value from binary to BCD formart
 * @param1: binary value to convert, always a 2 digit number
 * @return: BCD value
 */
static uint8_t binary_to_bcd(uint8_t value_to_convert)
{
	uint8_t tens, ones;

	if(value_to_convert >= 10)
	{
		tens = value_to_convert / 10;
		ones = value_to_convert % 10;
		return (uint8_t) (tens << BITS_PER_DIGIT) | ones;
	}
	return value_to_convert;
}

/*
 * Converts the given value from BCD to binary formart
 * @param1: BCD value to convert, always a 2 digit number
 * @return: binary value
 */
static uint8_t bcd_to_binary(uint8_t value_to_convert)
{
	uint8_t tens, ones;

	if(value_to_convert >= 10)
	{
		tens = (uint8_t)(value_to_convert >> BITS_PER_DIGIT) * 10;
		ones = value_to_convert & (uint8_t)0x0F;
		return tens + ones;

	}
	return value_to_convert;
}


/*
 * initialize the DS1307 module with the necessary
 * I2C configs and I2C AF configs for GPIO pins
 * @return: CH = 1; init failed
 * 			CH = 0; init success - crystal oscillator has begun
 */
uint8_t ds1307_init(void)
{
	//1. init the I2C pins
	ds1307_I2C_pin_config();

	//2. initialize the I2C peripheral
	ds1307_I2C_config();

	//3 Enable the I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	//4. make clock hault (CH) as 0 to start crystal oscillator in the RTC module
	ds1307_write(DS1307_ADDR_SEC, DS1307_STARTOSCL);

	//5. Read back clock hault bit to verify it is set to 0
	uint8_t osc_clock_state =  ds1307_read(DS1307_ADDR_SEC);

	// since clock state is the 7th bit of register 0
	return (osc_clock_state >> DS1307_SECONDS_CH) & 0x1;

}

/*
 * Initialize RTC registers with current time value
 */
void ds1307_set_current_time(RTC_time_t *p_RTC_Time)
{
	// program seconds
	uint8_t seconds, hrs;
	seconds = binary_to_bcd(p_RTC_Time->seconds);

	// clear bit field 7 because we don't disable the oscillator
	seconds &= ~(1 << DS1307_SECONDS_CH);
	//write to seconds reg
	ds1307_write(DS1307_ADDR_SEC, seconds);

	// write to minutes reg
	ds1307_write(DS1307_ADDR_MIN, binary_to_bcd(p_RTC_Time->minutes));

	// program hours
	hrs = binary_to_bcd(p_RTC_Time->hours);
	if (p_RTC_Time->time_format == TIME_FORMAT_24HRS)
	{
		// 24 time format
		hrs &= ~(1 << DS1307_HOURS_TIMEFORMATBIT);
	}
	else
	{
		// 12 hour format
		hrs |= (1 << DS1307_HOURS_TIMEFORMATBIT);
		hrs  = (p_RTC_Time->time_format == TIME_FORMAT_12HRS_PM) ? hrs | (1 << DS1307_HOURS_AMPMBIT)
				: hrs & ~(1 << DS1307_HOURS_AMPMBIT);
	}

	ds1307_write(DS1307_ADDR_HOUR, hrs);

}

/*
 * getter for the current time ticked in register
 * read from the rtc chip registers and fill it in the struct
 */
void ds1307_get_current_time(RTC_time_t *p_RTC_Time)
{
	uint8_t seconds, hrs;

	seconds = ds1307_read(DS1307_ADDR_SEC);
	// clear the CH field we do not need that info
	seconds &= ~(1 << DS1307_SECONDS_CH);

	// Set secs & mins
	p_RTC_Time->seconds = bcd_to_binary(seconds);
	p_RTC_Time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	hrs = ds1307_read(DS1307_ADDR_HOUR);
	if(hrs & ( 1 << DS1307_HOURS_TIMEFORMATBIT))
	{
		// 12 hour format
		if (hrs &= (1 << DS1307_HOURS_AMPMBIT))
		{
			// PM mode
			p_RTC_Time->time_format = TIME_FORMAT_12HRS_PM;
		}
		else
		{
			// AM mode
			p_RTC_Time->time_format = TIME_FORMAT_12HRS_AM;
		}
		// clear bit 5 & 6
		hrs &= ~(0x3 << DS1307_HOURS_AMPMBIT);
	}
	else
	{
		// 24 hour format
		p_RTC_Time->time_format = TIME_FORMAT_24HRS;
	}

	// set the hours
	p_RTC_Time->hours = bcd_to_binary(hrs);

}

/*
 * Initialize RTC registers with current date value
 */
void ds1307_set_current_date(RTC_date_t *p_RTC_Date)
{
	//program day register
	ds1307_write(DS1307_ADDR_DAY, binary_to_bcd(p_RTC_Date->day));

	// program date register
	ds1307_write(DS1307_ADDR_DATE, binary_to_bcd(p_RTC_Date->date));

	// program month register
	ds1307_write(DS1307_ADDR_MONTH, binary_to_bcd(p_RTC_Date->month));

	// program the year register
	ds1307_write(DS1307_ADDR_YEAR, binary_to_bcd(p_RTC_Date->year));
}

/*
 * getter for the current date in register
 * read from the rtc chip registers and fill it in the struct
 */
void ds1307_get_current_date(RTC_date_t *p_RTC_Date)
{

	p_RTC_Date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	p_RTC_Date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	p_RTC_Date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	p_RTC_Date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}



