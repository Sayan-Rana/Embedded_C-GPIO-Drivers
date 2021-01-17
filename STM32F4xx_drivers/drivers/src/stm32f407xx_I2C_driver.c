/*
 * stm32f407xx_I2C_driver.c
 *
 *  Created on: 08-Jan-2021
 *      Author: Sayan Rana
 */


#include "stm32f407xx_I2C_driver.h"


static void I2CGenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);


// Array to hold AHB Prescaler values
uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};

// Array to hold APB Prescaler values
uint8_t APB1_PreScaler[4] = {2,4,8,16};

/*
 **************************************************************************************************************************************
 *												Some helper functions private to stm32f407xx_I2C_driver.c
 **************************************************************************************************************************************
 */
// Helper function to Generate the start condition
static void I2CGenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

// Helper function to send 7 bit slave address with 1 bit r/w information.(total 8 bit)
static void I2C_ExecuteAddressPhase(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);		// Left shift the 7 bit slave address by 1
	SlaveAddr &= ~(1);					// Clearing the r/W bit to initiate the WRITE operation
										// SlaveAddr is 7 bit slave address + r/w bit

	pI2Cx->I2C_DR = SlaveAddr;			// Copying the Slave address byte into I2C DR
}

// This helper function will clear the ADDR flag by reading SR1 followed by SR2
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
	uint32_t dummyRead = pI2Cx->I2C_SR1;
	dummyRead = pI2Cx->I2C_SR2;
	(void)dummyRead;
}

// Helper function to generate stop condition
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}
/*
 **************************************************************************************************************************************
 */




/****************************************************************************************
 *	@fn                  - I2C_PeriClockControl
 *
 *	@brief               - This function will enable or disable peripheral clock for given I2C port
 *
 *	@param[in]           - Base address of the I2C peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enable the clock for given I2C port address

		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}else{
		// Disable the clock for given I2C port address

		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}



uint32_t RCC_GetPLLOutputClock()
{
	// Not implemented in this program
	return 0;
}



/****************************************************************************************
 *	@fn                  - RCC_GetPCLK1Value
 *
 *	@brief               - This function will return the peripheral clock value
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
 *	@fn                  - I2C_Init
 *
 *	@brief               - This function will initialize a given I2C port
 *
 *	@param[in]           - Will take the handle structure address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
	// Enabling the peripheral clock
	I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	// Temp. variable declaration
	uint32_t tempReg = 0;

	// Configure the I2C CR1 register ACK bit
	tempReg |= (pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->I2C_CR1 = tempReg;

	tempReg = 0;
	// Configure the I2C CR2 register FREQ bits by putting APB1 bus clock frequency
	tempReg |= (RCC_GetPCLK1Value() / 1000000U);  // Divided by zero because FREQ bat can only take value. Example, for 16000000(16MHz) it will take 16 only.
	pI2CHandle->pI2Cx->I2C_CR2 = (tempReg & 0x3F);  // Further masking for safety purpose, only 6bit is valid.

	tempReg = 0;
	// Program the device own address on I2C OAR1 register
	tempReg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD7_1);
	tempReg |= (1 << 14); // Ref. by the reference manual
	pI2CHandle->pI2Cx->I2C_OAR1 = tempReg;

	// CCR calculation
	uint16_t ccr_value = 0;
	tempReg = 0;

	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard
		ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		tempReg |= (ccr_value & 0xFFF);
	}else
	{
		// Mode is fast
		// Setting the F/S(Fast mode) bit on I2C CCR register
		tempReg |= (1<< I2C_CCR_F_OR_S);
		tempReg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		// Checking the first the duty cycle
		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
		{
			// Configuring the CCR bit when DUTY bit is reset
			ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));

		}else if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_16_9)
		{
			// Configuring the CCR bit when DUTY bit is set
			ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
		}

		tempReg |= (ccr_value & 0xFFF);
	}

	pI2CHandle->pI2Cx->I2C_CCR = tempReg;
}



/****************************************************************************************
 *	@fn                  - I2C_DeInit
 *
 *	@brief               - This function will de-initialize a given I2C port
 *
 *	@param[in]           - Base address of the I2C Peripheral
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
	if(pI2Cx == I2C1)
	{
		I2C1_REG_RESET();
	}else if(pI2Cx == I2C2)
	{
		I2C2_REG_RESET();
	}else if(pI2Cx == I2C3)
	{
		I2C3_REG_RESET();
	}
}



/****************************************************************************************
 *	@fn                  - I2C_MasterSendData
 *
 *	@brief               - This function will send data for I2C master device over I2C bus
 *
 *	@param[in]           - Handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *	@param[in]           - User given slave address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t SlaveAddr)
{
	// 1. Generate the START condition
	I2CGenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched(pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// 3. Send the address of the slave with r/w bit set to 0(Write) (Total 8 bit)
	I2C_ExecuteAddressPhase(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// 5. Clear the ADDR flag according to the software sequence
	// Note: Until ADDR is cleared SCL will be stretched(Pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

	// 6. Send the data until length become 0
	while(len > 0)
	{
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE) );	// Wait till TxE is set
		pI2CHandle->pI2Cx->I2C_DR = *pTxbuffer;							// Content of TxBuffer copied to DR
		pTxbuffer++;													// Incrementing address of TxBuffer
		len--;															// Decrementing length
	}

	// 7. When length become zero wait for TXE=1 and BTF=1 before generating the stop condition
	// Note: TXE=1, BTF=1 means that both SR(Shift register) and DR(Data register) are empty and next transmission should begin
	// When BTF=1 SCL will be stretched(Pulled to LOW)
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE));
	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

	// 8. Generate STOP condition and master need not to wait for the completion of STOP condition
	// Note: Generating STOP, automatically clears the BTF
	I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}
























/****************************************************************************************
 *	@fn                  - I2C_GetFlagStatus
 *
 *	@brief               - This function will return the flag status of status register
 *
 *	@param[in]           - Base address of the I2C peripheral
 *	@param[in]           - In stm32f407xx_I2C_driver.h  @I2C related status flags definitions
 *
 *	@return              - Either flag reset or set
 *
 *	@note                - none
 *
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
	if(pI2Cx->I2C_SR1 & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*
 * Data send and receive
 */



/****************************************************************************************
 *	@fn                  - I2C_IRQ_IntConfig
 *
 *	@brief               - This function will configure the IRQ by selecting IRQ number, IRQ priority and
 *						   IRQ status(either on or off) inside processor(NVIC)
 *
 *	@param[in]           - IRQ number which will execute
 *	@param[in]           - IRQ SET or RESET macro
 *
 *	@return              - none
 *
 *	@note                - none
 */
void I2C_IRQ_IntConfig(uint8_t IRQNumber, uint8_t EnOrDis)
{
	// Enabling or disabling Interrupt in NVIC ISER or ICER registers
	if(EnOrDis == ENABLE)
	{
		if(IRQNumber >= 0 && IRQNumber <= 31)
		{
			// Program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			// Program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// Program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber % 32));
		}
	}else{
		if(IRQNumber >= 0 && IRQNumber <= 31)
		{
			// Program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}else if(IRQNumber > 31 && IRQNumber <= 63)
		{
			// Program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber % 32));
		}else if(IRQNumber > 63 && IRQNumber <= 95)
		{
			// Program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber % 32));
		}
	}
}



/****************************************************************************************
 *	@fn                  - I2C_IRQPriorityConfig
 *
 *	@brief               - This function will configure the priority of IRQ
 *
 *	@param[in]           - IRQ number
 *	@param[in]           - IRQ Priority number
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. Find out IPR Register
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	// 2. Configure the IPR Register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/****************************************************************************************
 *	@fn                  - I2C_PeripheralControl
 *
 *	@brief               - This function will enable or disable I2C communication by setting or reseting PE Bit in CR1
 *
 *	@param[in]           - Base address of the I2C peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{

}



// Application Event call back
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	// This is a weak implementation. The application may override this function
}












