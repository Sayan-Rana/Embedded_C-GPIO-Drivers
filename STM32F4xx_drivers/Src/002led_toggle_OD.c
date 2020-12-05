/*
 * 002led_toggle_OD.c
 *
 *  Created on: 05-Dec-2020
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
	gpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;  // Using external pull_up resistor of 470ohm instead of using internal pull_up of 40kohm.

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}
	return 0;
}
