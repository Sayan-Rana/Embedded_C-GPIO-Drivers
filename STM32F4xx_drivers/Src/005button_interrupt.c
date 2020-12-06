/*
 * 005button_interrupt.c
 *
 *  Created on: 07-Dec-2020
 *      Author: Sayan Rana
 * For more details check	:	 STM32F4xx_drivers/drivers/inc/stm32f407xx.h
 * 							:	 STM32F4xx_drivers/drivers/inc/stm32f407xx_GPIO_driver.h
 * 							:	 STM32F4xx_drivers/drivers/src/stm32f407xx_GPIO_driver.c
 */


#include <string.h>
#include "stm32f407xx.h"


/*
 * Software delay for testing purpose
 * this will introduce ~200ms delay when the system clock is 16MHz
 */
void delay()
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main()
{
	GPIO_Handle_t gpioLed, gpioButton;
	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioButton, 0, sizeof(gpioButton));

	// Led output pin configuration
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	gpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD,ENABLE);
	GPIO_Init(&gpioLed);

	// GPIO external button pin configuration
	gpioButton.pGPIOx = GPIOD;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INT_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinOpType =
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioButton);

	// IRQ configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
	GPIO_IRQ_IntConfig(IRQ_NO_EXTI9_5, ENABLE);

	while(1);
}


void EXTI9_5_IRQHandler(void)
{
	delay();		// Small delay to ignore button debouncing
	GPIO_IRQHandling(GPIO_PIN_NO_5);		//Clear the bit of  EXTI PR register corresponding to the pin number
	GPIO_ToggleOutputPin(GPIOD, 12);		//Toggling the output pin
}
