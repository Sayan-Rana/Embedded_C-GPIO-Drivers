/*
 * 016UART_case_change.c
 *
 *  Created on: 03-Mar-2021
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

// Three different message will transmit to arduino

char *msg[3] = {"hihihihihihihi123", "Hello How are you?", "Today is Monday!!"};

// Reply from arduino will be stored here
char rx_buf[1024];

USART_Handle_t USART2_handle;

// This flag indicates reception completion
uint8_t rxCmplt = RESET;

void USART2_Init(void)
{
	USART2_handle.pUSARTx = USART2;
	USART2_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	USART2_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART2_handle.USART_Config.USART_Mode = USART_MODE_TxRx;
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
	GPIO_Handle_t GPIOBtn;

	//this is btn gpio configuration
	GPIOBtn.pGPIOx = GPIOA;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GPIOBtn);
}


void delay(void)
{
	for(uint32_t i=0; i<500000/2; i++);
}


int main(void)
{
	printf("UART Tx Rx with arduino\n");
	uint32_t cnt = 0;

	GPIO_ButtonInit();
	USART2_GPIOInit();
	USART2_Init();

	USART_IRQ_InterruptConfig(IRQ_NO_USART2, ENABLE);

	USART_PeripheralControl(USART2, ENABLE);

	printf("Application is running\n");

	while(1)
	{
		// Wail till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

		// To avoid button de-bouncing related issue, 200ms of delay
		delay();

		// Next message index, cnt value doesn't cross 2
		cnt = cnt % 3;

		// First enable the reception in interrupt mode (Because UART is a full duplex communication)
		// this code enable the receive interrupt
		while ( USART_ReceiveDataIT(&USART2_handle, (uint8_t*)rx_buf, strlen(msg[cnt])) != USART_READY);

		// Send the message indexed by cnt in blocking mode
		USART_SendData(&USART2_handle, (uint8_t*)msg[cnt], strlen(msg[cnt]) );

		printf("Transmitted : %s\n", msg[cnt]);

		// Wait until all bytes are received from the arduino
		// When all bytes are received rxCmplt will be SET in application event call back

		while(rxCmplt != SET);

		// Make sure that last byte should be null
		rx_buf[strlen(msg[cnt])] = '\0';

		// Print what we received from the arduino
		printf("Received : %s\n", rx_buf);

		// Invalidate the flag
		rxCmplt = RESET;

		// Move on to next message index in msg[]
		cnt++;

	}

	return 0;
}


void USART2_IRQHandler(void)
{
	USART_IRQHandling(&USART2_handle);
}


void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
	if(AppEvent == USART_EVENT_RX_CMPLT)
	{
		rxCmplt = SET;
	}else if(AppEvent == USART_EVENT_TX_CMPLT)
	{
		;
	}
}







