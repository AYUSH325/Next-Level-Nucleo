/*
 * 015RTC_LCD_interface.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */

#include <stdio.h>

#include "DS1307.h"
#include "LCD.h"

//the systick timer clock runs at the same speed as HSI clock of the MCU
#define SYSTICK_TIM_CLK		16000000

RTC_date_t current_date;
RTC_time_t current_time;

/*
 * @param: no. of interrupts raised every 1second
 */
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
	// down counter which will call the interrupt after counting down to 0
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}


char* get_day_of_week(uint8_t code)
{
	char *days_of_week[] = {
			"Sunday",
			"Monday",
			"Tuesday",
			"Wednesday",
			"Thursday",
			"Friday",
			"Saturday"
	};

	return days_of_week[code - 1];
}

void number_to_string(char* buf, uint8_t num)
{
	if (num < 10)
	{
		buf[0] = '0';
		buf[1] = num + 48;
	}
	else
	{
		buf[0] = (num/10) + 48;
		buf[1] = (num%10) + 48;
	}
}

//hh:mm:ss
char* time_to_string(RTC_time_t *p_rtc_time )
{
	// since we return buf from the function & not want it to be a dangling pointer, make it static and available in global scope
	static char buf[9];

	buf[2]= ':';
	buf[5]= ':';

	number_to_string(buf, p_rtc_time->hours);
	number_to_string(&buf[3], p_rtc_time->minutes);
	number_to_string(&buf[6],p_rtc_time->seconds);

	buf[8] = '\0';

	return buf;
}

// dd//mm/yy
char* date_to_string(RTC_date_t *p_date_time)
{
	static char buf[9];

	buf[2]= '/';
	buf[5]= '/';

	number_to_string(buf, p_date_time->date);
	number_to_string(&buf[3], p_date_time->month);
	number_to_string(&buf[6],p_date_time->year);

	buf[8] = '\0';

	return buf;
}

int main(void)
{

	printf("RTC test\n");

	lcd_init();

	lcd_write_string("RTC TEST...");

	// to return the cursor to (0,0)
	msdelay(2000);

	void lcd_display_clear();
	void lcd_display_return_home();


	if (ds1307_init())
	{
		printf("RTC init has failed\n");
		while(1);
	}

	current_date.day = WEDNESDAY;
	current_date.date = 24;
	current_date.month = 4;
	current_date.year = 24;

	current_time.hours = 9;
	current_time.minutes = 36;
	current_time.seconds = 50;
	current_time.time_format = TIME_FORMAT_12HRS_PM;

	init_systick_timer(1);

	ds1307_set_current_time(&current_time);
	ds1307_set_current_date(&current_date);

	char *am_pm;

	lcd_set_cursor(1,1);
	if(current_time.time_format != TIME_FORMAT_24HRS)
	{
		am_pm = current_time.time_format ? "PM" : "AM";
//		printf("Current time = %s %s\n", time_to_string(&current_time), am_pm); // 09:36:50 PM
		lcd_write_string(time_to_string(&current_time));
		lcd_write_string(am_pm);

	}
	else
	{
//		printf("Current time = %s\n", time_to_string(&current_time)); //9:36:50
		lcd_write_string(time_to_string(&current_time));
	}
	// 24/04/24 <Wednesday>
//	printf("Current date = %s <%s>", date_to_string(&current_date), get_day_of_week(current_date.day));
	lcd_set_cursor(2,1); // date in second row from first column
	lcd_write_string(date_to_string(&current_date));
	lcd_send_char(' ');
	lcd_write_string(get_day_of_week(current_date.day));



	return 0;
}

void SysTick_Handler(void)
{
	ds1307_get_current_time(&current_time);
	ds1307_get_current_date(&current_date);
}
