/*
 * ds1307.c
 *
 *  Created on: 08-Mar-2021
 *      Author: Sayan Rana
 */
#include "ds1307.h"
#include <stdint.h>
#include <string.h>


// Some private helper function definition
static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);
static uint8_t bcd_to_binary(uint8_t value);
static uint8_t binary_to_bcd(uint8_t value);

I2C_Handle_t g_ds1307i2cHandle;



/*
 * Note : if this function returns 1 then CH = 1; Initialization failed
 * 		  if this function returns 0 then CH = 0; Initialization success
 */
uint8_t DS1307_init(void)
{
	// 1. Initialize I2C pins
	ds1307_i2c_pin_config();

	// 2. Initialize I2C peripheral
	ds1307_i2c_config();

	// 3. Enable I2C peripheral
	I2C_PeripheralControl(DS1307_I2C, ENABLE);

	// 4. Enable acking
	I2C_ManageAcking(DS1307_I2C, ENABLE);


	// 5. Set Clock halt bit as 0
	ds1307_write(0x00, DS1307_ADDR_SEC);

	// 6. Read back clock halt bit
	uint8_t clk_state = ds1307_read(DS1307_ADDR_SEC);


	return ((clk_state >> 7) & 0x01);
}



void DS1307_set_current_time(RTC_time_t *pRTC_time)
{
	uint8_t seconds, hours;
	seconds = binary_to_bcd(pRTC_time->seconds);
	seconds &= ~(1 << 7);
	ds1307_write(seconds, DS1307_ADDR_SEC);

	ds1307_write(binary_to_bcd(pRTC_time->minutes), DS1307_ADDR_MIN);

	hours = binary_to_bcd(pRTC_time->hours);

	if(pRTC_time->time_format == DS1307_TIME_FORMAT_24HRS)
	{
		hours &= ~(1 << 6);
	}else{
		hours |= (1 << 6);
		hours = (pRTC_time->time_format == DS1307_TIME_FORMAT_12HRS_PM) ? hours | (1 << 5) : hours & ~(1 << 5);
	}
	ds1307_write(hours, DS1307_ADDR_HRS);
}



void DS1307_get_current_time(RTC_time_t *pRTC_time)
{
	uint8_t seconds, hrs;
	seconds = ds1307_read(DS1307_ADDR_SEC);
	seconds &= ~(1 << 7);		// Clearing 7th bit of seconds register because it is not relevant
	pRTC_time->seconds = bcd_to_binary(seconds);

	pRTC_time->minutes = bcd_to_binary(ds1307_read(DS1307_ADDR_MIN));

	hrs = ds1307_read(DS1307_ADDR_HRS);

	// Check 12 Hrs format or 24 Hrs format
	if(hrs & (1 << 6))
	{
		// 12Hrs format

		// Checking 5th bit, whether it is AM or PM
		pRTC_time->time_format = !((hrs & (1 << 5)) == 0);
		hrs &= ~(0x3 << 5);		// Clearing 5th and 6th bit of HOURS register of DS1307
	}else{
		// 24Hrs format
		pRTC_time->time_format = DS1307_TIME_FORMAT_24HRS;
	}
	pRTC_time->hours = bcd_to_binary(hrs);
}



void DS1307_set_current_date(RTC_date_t *pRTC_date)
{
	ds1307_write(binary_to_bcd(pRTC_date->day), DS1307_ADDR_DAY);
	ds1307_write(binary_to_bcd(pRTC_date->date), DS1307_ADDR_DATE);
	ds1307_write(binary_to_bcd(pRTC_date->month),DS1307_ADDR_MONTH);
	ds1307_write(binary_to_bcd(pRTC_date->year), DS1307_ADDR_YEAR);
}



void DS1307_get_current_date(RTC_date_t *pRTC_date)
{
	pRTC_date->day = bcd_to_binary(ds1307_read(DS1307_ADDR_DAY));
	pRTC_date->date = bcd_to_binary(ds1307_read(DS1307_ADDR_DATE));
	pRTC_date->month = bcd_to_binary(ds1307_read(DS1307_ADDR_MONTH));
	pRTC_date->year = bcd_to_binary(ds1307_read(DS1307_ADDR_YEAR));
}



static void ds1307_i2c_pin_config(void)
{
	GPIO_Handle_t i2c_sda, i2c_scl;

	memset(&i2c_sda, 0, sizeof(i2c_sda));
	memset(&i2c_scl, 0, sizeof(i2c_scl));

	/*
	 * I2C_SCL ===> PB6
	 * I2C_SDA ===> PB7
	 */


	i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_sda.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_ALTFN_AF4;
	i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
	i2c_sda.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	i2c_sda.GPIO_PinConfig.GPIO_PuPdControl = DS1307_I2C_PUPD;
	i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_sda);

	i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
	i2c_scl.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_ALTFN_AF4;
	i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
	i2c_scl.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	i2c_scl.GPIO_PinConfig.GPIO_PuPdControl = DS1307_I2C_PUPD;
	i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_Init(&i2c_scl);
}



static void ds1307_i2c_config(void)
{
	g_ds1307i2cHandle.pI2Cx = DS1307_I2C;
	g_ds1307i2cHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	g_ds1307i2cHandle.I2C_Config.I2C_DeviceAddress = DS1307_I2C_ADDRESS;
	g_ds1307i2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;

	I2C_Init(&g_ds1307i2cHandle);
}



static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
	uint8_t tx[2];

	tx[0] = reg_addr;
	tx[1] = value;

	I2C_MasterSendData(&g_ds1307i2cHandle, tx, 2, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);
}



static uint8_t ds1307_read(uint8_t reg_addr)
{
	uint8_t data;

	// Send Reg_addr to set the address pointer of DS1307
	I2C_MasterSendData(&g_ds1307i2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

	// Receiving the data from DS1307(selected register address)
	I2C_MasterReceiveData(&g_ds1307i2cHandle, &data, 1, DS1307_I2C_ADDRESS, I2C_DISABLE_SR);

	return data;
}



static uint8_t bcd_to_binary(uint8_t value)
{
	uint8_t m,n;
	m = (uint8_t) ((value >> 4) * 10);
	n = value & (uint8_t) 0x0F;
	return (m+n);
}



static uint8_t binary_to_bcd(uint8_t value)
{
	uint8_t m,n,bcd;

	bcd = value;
	if(value >= 10)
	{
		m = value / 10;
		n = value % 10;
		bcd = (m << 4) | n;
	}
	return bcd;
}








