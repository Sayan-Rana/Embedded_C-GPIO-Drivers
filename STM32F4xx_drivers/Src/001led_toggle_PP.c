/*
 * 001led_toggle.c
 *
 *  Created on: 30-Nov-2020
 *      Author: Sayan Rana
 * For more details check	:	 STM32F4xx_drivers/drivers/inc/stm32f407xx.h
 * 							:	 STM32F4xx_drivers/drivers/inc/stm32f407xx_GPIO_driver.h
 * 							:	 STM32F4xx_drivers/drivers/src/stm32f407xx_GPIO_driver.c
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
	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
