/*
 * stm32f407xx_GPIO_driver.c
 *
 *  Created on: 29-Nov-2020
 *      Author: Sayan Rana
 */


#include "stm32f407xx_GPIO_driver.h"



/****************************************************************************************
 *	@fn                  - GPIO_PeriClockEnable
 *
 *	@brief               - This function will enable or disable peripheral clock for given GPIO port
 *
 *	@param[in]           - Base address of the GPIO peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enable the clock for given GPIO port address

		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else{
		// Disable the clock for given GPIO port address

		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}



/****************************************************************************************
 *	@fn                  - GPIO_Init
 *
 *	@brief               - This function will initialize a given GPIO port
 *
 *	@param[in]           -
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; 			// Temp. register

	// 1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// Its a non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing the required bit
		pGPIOHandle->pGPIOx->MODER |= temp;	//Setting the required bit

	}else{
		// Interrupt mode

	}

	temp = 0;

	// 2. Configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing the required bit
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;	//Setting the required bit

	temp = 0;

	// 3. Configure the Pull up Pull down settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing the required bit
	pGPIOHandle->pGPIOx->PUPDR |= temp;	//Setting the required bit

	temp = 0;

	// 4. Configure the output type
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOpType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x01 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);	//Clearing the required bit
	pGPIOHandle->pGPIOx->OTYPER |= temp;	//Setting the required bit

	temp = 0;

	// 5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1, temp2;

		temp1 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		temp2 = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0x0F << (4 * temp2));		//Clearing the required bit
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << (4 * temp2));	//Setting the required bit
	}
}


/****************************************************************************************
 *	@fn                  - GPIO_DeInit
 *
 *	@brief               - This function will de-initialize a given GPIO port
 *
 *	@param[in]           -
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}



/****************************************************************************************
 *	@fn                  - GPIO_ReadFromInputPin
 *
 *	@brief               - This function will read data from input data register of a selected GPIO input pin
 *
 *	@param[in]           - Base address of the GPIO peripheral
 *	@param[in]           - Pin number, whose value we want to read
 *
 *	@return              - True or False (0 or 1)
 *
 *	@note                - none
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);		// Bit right shifting and bit masking
	return value;
}


/****************************************************************************************
 *	@fn                  - GPIO_ReadFromInputPort
 *
 *	@brief               - This function will read data from input data register of a selected GPIO input port
 *
 *	@param[in]           - Base address of the GPIO peripheral
 *
 *	@return              - 16 bit value
 *
 *	@note                - none
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t) pGPIOx->IDR;		// Copying the whole port
	return value;
}


/****************************************************************************************
 *	@fn                  - GPIO_WriteToOutputPin
 *
 *	@brief               - This function will write data to output data register of a selected GPIO out pin
 *
 *	@param[in]           - Base address of the GPIO peripheral
 *	@param[in]           - Pin number of the selected output pin
 *	@param[in]			 - The value/data that we want to write to the selected pin
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		// Write 1 to the output data register at the bit field, corresponding to the pin number

		pGPIOx->ODR |= (1 << PinNumber);		//Setting the Bit position corresponding to the pin number in the ODR register

	}else{
		// Write 0 to the output data register at the bit field, corresponding to the pin number

		pGPIOx->ODR &= ~(1 << PinNumber);		//Clearing the Bit position corresponding to the pin number in the ODR register
	}
}


/****************************************************************************************
 *	@fn                  - GPIO_WriteToOutputPort
 *
 *	@brief               - This function will write data to output data register of a selected GPIO out port
 *
 *	@param[in]           - Base address of the GPIO peripheral
 *	@param[in]           - The value/data that we want to write to the selected port
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	// Just copy "Value" in to the ODR register
	pGPIOx->ODR = Value;
}


/****************************************************************************************
 *	@fn                  - GPIO_ToggleOutputPin
 *
 *	@brief               - This function will toggle(switching between high and low) the output  GPIO pin
 *
 *	@param[in]           - Base address of the GPIO peripheral
 *	@param[in]           - Pin number of selected output pin
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);		// ^ bitwise EXOR operator
}



/****************************************************************************************
 *	@fn                  - GPIO_IRQConfig
 *
 *	@brief               - This function will configure the IRQ by selecting IRQ number, IRQ priority and
 *						   IRQ status(either on or off)
 *
 *	@param[in]           - IRQ number which will execute
 *	@param[in]           - IRQ priority number
 *	@param[in]           - IRQ SET or RESET macro
 *
 *	@return              - none
 *
 *	@note                - none
 */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnOrDis)
{

}


/****************************************************************************************
 *	@fn                  - GPIO_IRQHandling
 *
 *	@brief               - This function will handle the IRQ
 *
 *	@param[in]           - Input pin number which will receive the IRQ request
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void GPIO_IRQHandling(uint8_t PinNumber)
{

}

