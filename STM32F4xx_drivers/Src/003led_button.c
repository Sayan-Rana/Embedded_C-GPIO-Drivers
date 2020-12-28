/*
 * 003led_button.c
 *
 *  Created on: 05-Dec-2020
 *      Author: Sayan Rana
 * For more details check	:	 STM32F4xx_drivers/drivers/inc/stm32f407xx.h
 * 							:	 STM32F4xx_drivers/drivers/inc/stm32f407xx_GPIO_driver.h
 * 							:	 STM32F4xx_drivers/drivers/src/stm32f407xx_GPIO_driver.c
 */

#include "stm32f407xx.h"

#define BUTTON_PRESSED								1
/*
 * Software delay for testing purpose
 */
void delay()
{
	for(uint32_t i = 0; i < 500000/2; i++);
}


int main(void)
{
	GPIO_Handle_t gpioLed, gpioBtn;

	//Led(Output) pin configuration.
	gpioLed.pGPIOx = GPIOD;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&gpioLed);

	//Button(Input) pin configuration.
	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&gpioBtn);

	while(1)
	{
		if(GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) == BUTTON_PRESSED) // Read the state of the button
		{
			delay();  // Small delay to ignore button debouncing
			GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		}
	}
	return 0;
}
