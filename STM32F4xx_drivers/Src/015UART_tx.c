/*
 * 015UART_tx.c
 *
 *  Created on: 01-Mar-2021
 *      Author: sayan
 */


#include <stdio.h>
#include <string.h>
#include "stm32f407xx.h"

/*
 * UART communication pins
 * PA2----> TX
 * PA3----> RX
 */

char msg[1024] = "USART Tx testing...\n\r";
USART_Handle_t USART2_handle;

void USART2_Init(void)
{
	USART2_handle.pUSARTx = USART2;
	USART2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2_handle.USART_Config.USART_Mode = USART_MODE_ONLY_Tx;
	USART2_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART2_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART2_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&USART2_handle);
}


void USART2_GPIOInit(void)
{
	GPIO_Handle_t usart2_gpios;

	usart2_gpios.pGPIOx = GPIOA;
	usart2_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart2_gpios.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_ALTFN_AF7;
	usart2_gpios.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	usart2_gpios.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	usart2_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// USART2 Tx
	usart2_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&usart2_gpios);

	// USART2 Rx
	usart2_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&usart2_gpios);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOBtn,GpioLed;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);

	GPIO_Init(&GpioLed);
}


void delay(void)
{
	for(uint32_t i=0; i<500000/2; i++);
}


int main(void)
{
	GPIO_ButtonInit();
	USART2_GPIOInit();
	USART2_Init();

	USART_PeripheralControl(USART2, ENABLE);

	while(1)
	{
		// Wail till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		// To avoid button de-bouncing related issue, 200ms of delay
		delay();

		USART_SendData(&USART2_handle, (uint8_t*)msg, strlen(msg) );

	}

	return 0;
}






