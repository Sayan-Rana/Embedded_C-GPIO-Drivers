/*
 * 006SPI_tx_testing.c
 *
 *  Created on: 13-Dec-2020
 *      Author: Sayan Rana
 */

#include <string.h>
#include "stm32f407xx.h"

/*
 * SPI2 pin definition
 *
 * PB12------> SPI2 NSS
 * PB13------> SPI2 SCLK
 * PB14------> SPI2 MISO
 * PB15------> SPI2 MOSI
 * Alt fnc---> AF5
 */

void SPI2_GPIOInit(void);
void SPI2_Init(void);


int main(void)
{
	char user_data[] = "Hello World!";
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	// This function is used to initialize the SPI peripheral parameters
	SPI2_Init();

	// This makes the NSS signal internall;y high and avoid MODF error
	SPI_SSIConfig(SPI2, ENABLE);

	//	Enable SPI2 peripheral
	SPI_PeripheralControl(SPI2, ENABLE);

	// To send data
	SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

	// Confirm that SPI is not busy in communication
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

	//	Disable SPI2 peripheral
	SPI_PeripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}


void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_ALTFN_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// NSS pin
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_12;
	//GPIO_Init(&SPIPins);

	// SCLK pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO pin
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	//GPIO_Init(pGPIOHandle);


	//MOSI pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);
}


void SPI2_Init(void)
{
	SPI_Handle_t SPI2handle;

	SPI2handle.pSPIx = SPI2;
	SPI2handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;	// Generate sclk 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_EN;	// Software slave management enable

	SPI_Init(&SPI2handle);
}
