/*
 * ds1307.h
 *
 *  Created on: 08-Mar-2021
 *      Author: Sayan Rana
 */

#ifndef DS1307_H_
#define DS1307_H_

#include "stm32f407xx.h"



// Some application configurable items
#define DS1307_I2C							I2C2
#define DS1307_I2C_GPIO_PORT				GPIOB
#define DS1307_I2C_SDA_PIN					GPIO_PIN_NO_11
#define DS1307_I2C_SCL_PIN					GPIO_PIN_NO_10
#define DS1307_I2C_SPEED					I2C_SCL_SPEED_SM
#define DS1307_I2C_PUPD						GPIO_PIN_PU		//Using internal pull-up register


// Register addresses of DS1307 RTC module
#define DS1307_ADDR_SEC 					0x00
#define DS1307_ADDR_MIN 					0x01
#define DS1307_ADDR_HRS 					0x02
#define DS1307_ADDR_DAY 					0x03
#define DS1307_ADDR_DATE 					0x04
#define DS1307_ADDR_MONTH 					0x05
#define DS1307_ADDR_YEAR 					0x06

// Time format
#define DS1307_TIME_FORMAT_12HRS_AM			0
#define DS1307_TIME_FORMAT_12HRS_PM			1
#define DS1307_TIME_FORMAT_24HRS 			2


// Slave address
#define DS1307_I2C_ADDRESS					0x68


// Day information
#define SUNDAY								1
#define MONDAY								2
#define TUESDAY								3
#define WEDNESDAY							4
#define THURSDAY							5
#define FRIDAY								6
#define SATURDAY							7

// Data structure to hold date information
typedef struct {
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}RTC_date_t;


// Data structure to hold time information
typedef struct {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t time_format;
}RTC_time_t;




// Function prototypes
uint8_t DS1307_init(void);

void DS1307_set_current_time(RTC_time_t *);
void DS1307_get_current_time(RTC_time_t *);

void DS1307_set_current_date(RTC_date_t *);
void DS1307_get_current_date(RTC_date_t *);


#endif /* DS1307_H_ */
