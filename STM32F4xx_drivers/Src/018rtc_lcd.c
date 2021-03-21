/*
 * 018rtc_lcd.c
 *
 *  Created on: 08-Mar-2021
 *      Author: Sayan Rana
 */


#include "ds1307.h"
#include "lcd.h"
#include <stdio.h>


/*
 * PB10-------> SCL
 * PB11-------> SDA
 */


char* time_to_string(RTC_time_t *time);
char* date_to_string(RTC_date_t *date);
char* get_day_of_week(uint8_t day);
void number_to_string(uint8_t num, char *buff);
void init_systick_timer(uint32_t tick_hz);

#define SYSTICK_TIMER_CLK			16000000UL
#define TICK_HZ						1


void mdelay(uint32_t cnt)
{
	for(uint32_t i = 0; i < (cnt * 1000); i++);
}


int main()
{
	RTC_time_t current_time;
	RTC_date_t current_date;

	printf("RTC test\n");

	lcd_init();

	lcd_print_string("RTC LCD test...");
	mdelay(2000);
	lcd_display_clear();
	lcd_display_return_home();


	if(DS1307_init()){
		printf("Initialization has failed\n");
		while(1);		// Hang with infinite while loop
	}

	current_date.date = 11;
	current_date.month = 3;
	current_date.year = 21;
	current_date.day = THURSDAY;

	current_time.hours = 11;
	current_time.minutes = 59;
	current_time.seconds = 30;
	current_time.time_format = DS1307_TIME_FORMAT_12HRS_PM;

	DS1307_set_current_date(&current_date);
	DS1307_set_current_time(&current_time);

	init_systick_timer(TICK_HZ);

	// Program will hang here
	while(1);
	return 0;
}


// HH:MM:SS
char* time_to_string(RTC_time_t *time)
{
	static char buff[9];
	buff[2] = ':';
	buff[5] = ':';

	number_to_string(time->hours, buff);
	number_to_string(time->minutes, &buff[3]);
	number_to_string(time->seconds, &buff[6]);
	buff[8] = '\0';
	return buff;
}


// dd/mm/yy
char* date_to_string(RTC_date_t *date)
{
	static char buff[9];
	buff[2] = '/';
	buff[5] = '/';

	number_to_string(date->date, buff);
	number_to_string(date->month, &buff[3]);
	number_to_string(date->year, &buff[6]);
	buff[8] = '\0';
	return buff;
}


char* get_day_of_week(uint8_t day)
{
	char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
	return days[day-1];
}


void number_to_string(uint8_t num, char *buff)
{
	if(num < 10)
	{
		buff[0] = '0';
		buff[1] = num + 48;
	}else if(num >=10 && num <= 99)
	{
		buff[0] = ((num / 10) + 48);
		buff[1] = ((num % 10) + 48);
	}
}


// Enable SysTick timer interrupt
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSYST_CSR	= (uint32_t*)0xE000E010;
	uint32_t *pSYST_RVR = (uint32_t*)0xE000E014;

	uint32_t reloadValue = (SYSTICK_TIMER_CLK/tick_hz) - 1;

	// Clearing the value of SYST_RVR register
	*pSYST_RVR &= ~(0x00FFFFFF);

	// Load the reload value in SYST_RVR register
	*pSYST_RVR |= reloadValue;

	// Enable the systick exception request
	*pSYST_CSR |= (1 << 1);

	// Select the clock source as processor clock
	*pSYST_CSR |= (1 << 2);

	// Enable the systick counter
	*pSYST_CSR |= (1 << 0);
}



void SysTick_Handler(void)
{
	lcd_display_clear();
	lcd_display_return_home();


	RTC_time_t current_time;
	RTC_date_t current_date;
	char *am_pm;

	DS1307_get_current_time(&current_time);

	if(current_time.time_format != DS1307_TIME_FORMAT_24HRS){
		// 12Hrs format
		am_pm = (current_time.time_format) ? "PM" : "AM";

		//printf("Current time : %s %s\n", time_to_string(&current_time), am_pm);		// 11:21:42 PM
		lcd_print_string(time_to_string(&current_time));
		lcd_print_string(am_pm);
	}else{
		// 24Hrs format;
		//printf("Current time : %s\n", time_to_string(&current_time));				// 23:21:42
		lcd_print_string(time_to_string(&current_time));
	}

	DS1307_get_current_date(&current_date);

	// Date 09/03/21 <Tuesday>
	//printf("Current date : %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
	lcd_set_cursor(2, 1);
	lcd_print_string(date_to_string(&current_date));
	lcd_print_string(get_day_of_week(current_date.day));
}







