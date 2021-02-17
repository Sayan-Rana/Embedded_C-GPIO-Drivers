/*
 * 008SPI_cmd_handling.c
 *
 *  Created on: 23-Dec-2020
 *      Author: sayan Rana
 */


#include <string.h>
#include <stdbool.h>
#include <stdio.h>
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

// Command codes
#define COMMAND_LED_CTRL					0x50
#define COMMAND_SENSOR_READ					0x51
#define COMMAND_LED_READ					0x52
#define COMMAND_PRINT						0x53
#define COMMAND_ID_READ						0x54

// ACK or NACK code
#define ACK						0xF5
#define NACK					0xA5

// Slave LED state
#define LED_ON					1
#define LED_OFF					0

// Arduino analog pins
#define ANALOG_PIN_0			0
#define ANALOG_PIN_1			1
#define ANALOG_PIN_2			2
#define ANALOG_PIN_3			3
#define ANALOG_PIN_4			4

// Arduino LED pin
#define LED_PIN					9


void SPI2_GPIOInit(void);
void SPI2_Init(void);
void GPIO_ButtonInit(void);
void delay();	//Software delay for testing purpose (Button Debouncing)
bool is_ack_or_nacka(uint8_t ackOrNackByte);



int main(void)
{
	printf("Application is running... \n");
	// This function is used to initialize the GPIO pins to behave as SPI2 pins
	SPI2_GPIOInit();

	// This function is used to initialize the SPI peripheral
	SPI2_Init();

	printf("SPI init done \n");

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
		uint8_t dummyWrite = 0xff, dummyRead, ackOrNackByte, args[2];

		// Wait till button is pressed
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		//	Enable SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);



		//1. CMD_LED_CTLR   <pin no(1)>   <value(1)>

		uint8_t commandCode = COMMAND_LED_CTRL;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Dummy read to clear the RXNE bit in SR
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// Send dummy byte(1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// Read the ACK byte received
		SPI_ReceiveData(SPI2, &ackOrNackByte, 1);

		if( is_ack_or_nacka(ackOrNackByte) )
		{
			// Loading the arguments in the array
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// Send arguments
			SPI_SendData(SPI2, args, 2);
			printf("COMMAND_LED_CTLR executed.\n");
		}
		// End of COMMAND_LED_CTRL


		//2. CMD_SENSOR_READ   <Analog pin number(1)>

		// Wait till button is pressed
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_SENSOR_READ;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Dummy read
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// Send dummy byte(1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// Read the ACK byte received
		SPI_ReceiveData(SPI2, &ackOrNackByte, 1);

		if( is_ack_or_nacka(ackOrNackByte) )
		{
			// Loading the arguments in the array
			args[0] = ANALOG_PIN_0;

			// Send arguments
			SPI_SendData(SPI2, args, 1);

			// Dummy read to clear the RXNE bit in SR
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			// Insert some delay so that slave can ready with the data
			delay();

			// Send dummy byte(1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummyWrite, 1);

			uint8_t analog_raed;
			SPI_ReceiveData(SPI2, &analog_raed, 1);
			printf("Analog read value : %d\n",analog_raed);
		}
		// End of COMMAND_SENSOR_READ


		//3. CMD_LED_READ   <LED pin number(1)>

		// Wait till button is pressed
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_LED_READ;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Dummy read
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// Send dummy byte(1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// Read the ACK byte received
		SPI_ReceiveData(SPI2, &ackOrNackByte, 1);

		if( is_ack_or_nacka(ackOrNackByte) )
		{
			// Loading the arguments in the array
			args[0] = LED_PIN;

			// Send arguments
			SPI_SendData(SPI2, args, 1);

			// Dummy read to clear the RXNE bit in SR
			SPI_ReceiveData(SPI2, &dummyRead, 1);

			// Insert some delay so that slave can ready with the data
			delay();

			// Send dummy byte(1byte) to fetch the response from the slave
			SPI_SendData(SPI2, &dummyWrite, 1);

			uint8_t LED_status;
			SPI_ReceiveData(SPI2, &LED_status, 1);
			printf("COMMAND_READ_LED : %d\n", LED_status);
		}
		// End of COMMAND_LED_READ


		//4. CMD_PRINT   <Len>   <Message>

		// Wait till button is pressed
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_PRINT;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Dummy read
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// Send dummy byte(1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// Read the ACK byte received
		SPI_ReceiveData(SPI2, &ackOrNackByte, 1);

		char message[] = "I am learning Embedded Programming\n";

		if( is_ack_or_nacka(ackOrNackByte) )
		{
			// Loading the arguments in the array
			args[0] = strlen(message);

			// Send arguments
			SPI_SendData(SPI2, args, 1);

			// Send message
			SPI_SendData(SPI2, (uint8_t*)message, args[0]);

			printf("COMMAND_PRINT Executed\n");
		}
		//End of COMMAND_PRINT


		//5. COMMAND_ID_READ

		// Wait till button is pressed
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = COMMAND_ID_READ;

		// Send command
		SPI_SendData(SPI2, &commandCode, 1);

		// Dummy read
		SPI_ReceiveData(SPI2, &dummyRead, 1);

		// Send dummy byte(1byte) to fetch the response from the slave
		SPI_SendData(SPI2, &dummyWrite, 1);

		// Read the ACK byte received
		SPI_ReceiveData(SPI2, &ackOrNackByte, 1);

		uint8_t id[11];

		if( is_ack_or_nacka(ackOrNackByte) )
		{
			for(uint32_t i = 0; i < 10; i++)
			{
				// Send dummy bytes to fetch data from slave
				SPI_SendData(SPI2, &dummyWrite, 1);
				SPI_ReceiveData(SPI2, &id[i], 1);
			}

			id[10] = '\0';
			printf("COMMAND_ID_READ : %s \n", id);
		}
		// End of COMMAND_ID_READ


		// Confirm that SPI is not busy in communication
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG) );

		//	Disable SPI2 peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI communication closed \n");

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
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);


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
	SPI2handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV16;	// Generate sclk 1MHz
	SPI2handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;
	SPI2handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2handle.SPIConfig.SPI_SSM = SPI_SSM_DI;	// Hardware Slave management enable for NSS pin

	SPI_Init(&SPI2handle);
}

/*
 * This function will test the slave response, whether it is ACK or NACK
 */
bool is_ack_or_nacka(uint8_t ackOrNackByte)
{
	if(ackOrNackByte == ACK)
	{
		return true;
	}

	return false;
}


/*
 * Software delay for testing purpose (Button Debouncing)
 */
void delay()
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

