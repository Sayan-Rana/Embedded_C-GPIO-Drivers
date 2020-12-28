/*
 * 007SPI_txonly_arduino.c
 *
 *  Created on: 14-Dec-2020
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
void GPIO_ButtonInit(void);
void delay();	//Software delay for testing purpose (Button Debouncing)



int main(void)
{
	char user_data[] = "Hello World!";
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	// This function is used to initialize the SPI peripheral
	SPI2_Init();

	//This function is used to initialize the GPIO pins to behave as button input pins
	GPIO_ButtonInit();

	/*
	 * Making SSOE 1 does NSS output enable.
	 * The NSS pin is automatically managed by the hardware
	 * i.e when SPE = 1, NSS will be pulled to LOW
	 * and NSS pin will high when SPE = 0
	 */
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );
		delay();
		//	Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// First send the length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2, &dataLen, 1);

		// To send data
		SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// Confirm that SPI is not busy in communication
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//	Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioBtn;
	//Button(Input) pin configuration.
	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioBtn);
}


void SPI2_GPIOInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_ALTFN_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// NSS pin
	SPIPins.GPIO_PinConfig.GPIO_PinNumber  = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;	// Generate sclk 8MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;	// Hardware Slave management enable for NSS pin

	SPI_Init(&SPI2handle);
}
/*
 * Software delay for testing purpose (Button Debouncing)
 */
void delay()
{
	for(uint32_t i = 0; i < 500000/2; i++);
}
