/*
 * 001led_toggle.c
 *
 *  Created on: 30-Nov-2020
 *      Author: Sayan Rana
 */

#include "stm32f407xx.h"
/*
 * Software delay for testing purpose
 */
void delay()
{
	for(uint32_t i = 0; i < 500000; i++);
}


int main(void)
{
	GPIO_Handle_t gpioled;

	gpioled.pGPIOx = GPIOD;
	gpioled.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioled.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioled.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioled.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	gpioled.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioled);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
