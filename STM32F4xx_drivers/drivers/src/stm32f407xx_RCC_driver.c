/*
 * stm32f407xx_RCC_driver.c
 *
 *  Created on: 26-Feb-2021
 *      Author: sayan
 */



#include "stm32f407xx_RCC_driver.h"


// Array to hold AHB Prescaler values
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};

// Array to hold APB1 Prescaler values
uint8_t APB1_PreScaler[4] = {2,4,8,16};

// Array to hold APB2 Prescaler values
uint8_t APB2_PreScaler[4] = {2,4,8,16};



uint32_t RCC_GetPLLOutputClock()
{
	// Not implemented in this program
	return 0;
}






/****************************************************************************************
 *	@fn                  - RCC_GetPCLK1Value
 *
 *	@brief               - This function will return the peripheral clock value(APB1 Bus)
 *
 *	@param[in]           - none
 *
 *	@return              - Clock Frequency in MHz
 *
 *	@note                - none
 *
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1, SystemClk;
	uint8_t clkSrc, temp, ahbp, apb1p;

	// Checking 2nd and 3rd bit of RCC Clock configuration register for Clock source.
	clkSrc = ((RCC->CFGR >> 2) & 0x03);

	if(clkSrc == 0)
	{
		// HSI oscillator used as the system clock
		SystemClk = 16000000;
	}else if(clkSrc == 1)
	{
		// HSE oscillator used as the system clock
		SystemClk = 8000000;
	}else if(clkSrc == 2)
	{
		// PLL used as the system clock
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHB
	// Checking 4th to 7th bit of RCC Clock configuration register for AHB prescaler.
	temp = ((RCC->CFGR >> 4) & 0x0F);

	if(temp < 8)
	{
		// system clock not divided
		ahbp = 1;
	}else
	{
		// System clock is divided
		ahbp = AHB_PreScaler[temp - 8];
	}

	// APB1
	// Checking 10th to 12th bit of RCC Clock configuration register for APB1 prescaler.
	temp = ((RCC->CFGR >> 10) & 0x07);

	if(temp < 4)
	{
		// AHB bus clock not divided
		apb1p = 1;
	}else
	{
		// AHB bus clock is divided
		apb1p = APB1_PreScaler[temp - 4];
	}

	pclk1 = ((SystemClk / ahbp) / apb1p);

	return pclk1;
}



/****************************************************************************************
 *	@fn                  - RCC_GetPCLK2Value
 *
 *	@brief               - This function will return the peripheral clock value(APB2 Bus)
 *
 *	@param[in]           - none
 *
 *	@return              - Clock Frequency in MHz
 *
 *	@note                - none
 *
 */
uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t pclk2, SystemClk;
	uint8_t clkSrc, temp, ahbp, apb2p;

	// Checking 2nd and 3rd bit of RCC Clock configuration register for Clock source.
	clkSrc = ((RCC->CFGR >> 2) & 0x03);

	if(clkSrc == 0)
	{
		// HSI oscillator used as the system clock
		SystemClk = 16000000;
	}else if(clkSrc == 1)
	{
		// HSE oscillator used as the system clock
		SystemClk = 8000000;
	}else if(clkSrc == 2)
	{
		// PLL used as the system clock
		SystemClk = RCC_GetPLLOutputClock();
	}

	// AHB
	// Checking 4th to 7th bit of RCC Clock configuration register for AHB prescaler.
	temp = ((RCC->CFGR >> 4) & 0x0F);

	if(temp < 8)
	{
		// system clock not divided
		ahbp = 1;
	}else
	{
		// System clock is divided
		ahbp = AHB_PreScaler[temp - 8];
	}

	// APB2
	// Checking 13th to 15th bit of RCC Clock configuration register for APB2 prescaler.
	temp = ((RCC->CFGR >> 13) & 0x07);

	if(temp < 4)
	{
		// AHB bus clock not divided
		apb2p = 1;
	}else
	{
		// AHB bus clock is divided
		apb2p = APB2_PreScaler[temp - 4];
	}

	pclk2 = ((SystemClk / ahbp) / apb2p);

	return pclk2;
}


