/*
 * stm32f407xx_I2C_driver.c
 *
 *  Created on: 08-Jan-2021
 *      Author: Sayan Rana
 */


#include "stm32f407xx_I2C_driver.h"


static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
// static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);


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
static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_START);
}

// Helper function to send 7 bit slave address with + r/w bit = 0.(total 8 bit for master write to slave)
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);		// Left shift the 7 bit slave address by 1
	SlaveAddr &= ~(1);					// Clearing the r/W bit to initiate the WRITE operation
										// SlaveAddr is 7 bit slave address + r/w bit = 0

	pI2Cx->I2C_DR = SlaveAddr;			// Copying the Slave address byte into I2C DR
}

// This helper function will clear the ADDR flag by reading SR1 followed by SR2
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
	uint32_t dummyRead;
	// Check for device mode
	if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
	{
		// Device in MASTER mode

		// Check for application state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2CHandle->RxSize == 1)
			{
				// 1. Disable ACKING
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// 2. Clear ADDR flag(Read SR1 followed SR2)
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummyRead;
			}else{
				// RxSize > 1
				// 2. Clear ADDR flag(Read SR1 followed SR2)
				dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
				dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
				(void)dummyRead;
			}
		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Clear the ADDR flag(Read SR1 followed SR2)
			dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
			dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
			(void)dummyRead;
		}

	}else{

		// Device in SLAVE mode
		// Clear ADDR flag(Read SR1 followed SR2)
		dummyRead = pI2CHandle->pI2Cx->I2C_SR1;
		dummyRead = pI2CHandle->pI2Cx->I2C_SR2;
		(void)dummyRead;
	}
}

// Helper function to generate stop condition
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->I2C_CR1 |= (1 << I2C_CR1_STOP);
}

// Helper function to send 7 bit slave address with + r/w bit = 1.(total 8 bit for master read from slave)
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx, uint8_t SlaveAddr)
{
	SlaveAddr = (SlaveAddr << 1);		// Left shift the 7 bit slave address by 1
	SlaveAddr |= 1;						// Setting the r/W bit to initiate the READ operation
										// SlaveAddr is 7 bit slave address + r/w bit = 1
	pI2Cx->I2C_DR = SlaveAddr;			// Copying the Slave address byte into I2C DR
}

// Helper function to Handle Interrupt based Master Receive data(RXNE)
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
	// Case1
	if(pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->RxLen--;
	}

	// Case2
	if(pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			// Clear the ACKING
			I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
		}

		// Read DR
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->I2C_DR;
		pI2CHandle->pRxBuffer++;
		pI2CHandle->RxLen--;
	}
	if(pI2CHandle->RxLen == 0)
	{
		// Close I2C data reception and notify the application

		// 1. Generate STOP condition
		if(pI2CHandle->Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// 2. Close the I2C Rx
		I2C_CloseReceiveData(pI2CHandle);

		// 3. Notify the application
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
	}
}

// Helper function to handle interrupt based Master Send data(TXE)
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
	if(pI2CHandle->TxLen > 0)
	{
		// 1. Load data into DR
		pI2CHandle->pI2Cx->I2C_DR = *(pI2CHandle->pTxBuffer);

		// 2. Decrement TxLen
		pI2CHandle->TxLen--;

		// 3. Increment TxBuffer address
		pI2CHandle->pTxBuffer++;
	}
}
/*
 **************************************************************************************************************************************
 */



/****************************************************************************************
 *	@fn                  - I2C_SlaveEnableDisableCallbackEvents
 *
 *	@brief               - This function will enable or disable I2C interrupt control bits in I2C CR2 register
 *						   for triggering I2C_EV and I2C_ER interrupts
 *
 *	@param[in]           - Base address of the I2C peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enable this control bits
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
	}else
	{
		// Disable this control bits
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITERREN);
	}
}
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

	/* Checking the mode(std/fast) */
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

		// Checking first the duty cycle
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

	// TRISE configuration
	/* Checking the mode(std/fast) */
	if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
	{
		// Mode is standard
		tempReg = (RCC_GetPCLK1Value() / 1000000U) + 1;
	}else
	{
		// Mode is fast
		tempReg = ((RCC_GetPCLK1Value() * 300) / 1000000000U) + 1;
	}
	pI2CHandle->pI2Cx->I2C_TRISE = (tempReg & 0x3F);	// Only 6 bits are valid

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
 *	@param[in]           - Repeated Start enable disable macro
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	// 1. Generate the START condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// 2. Confirm that start generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched(pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// 3. Send the address of the slave with r/w bit set to 0(Write) (Total 8 bit)
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	// 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// 5. Clear the ADDR flag according to the software sequence
	// Note: Until ADDR is cleared SCL will be stretched(Pulled to LOW)
	I2C_ClearADDRFlag(pI2CHandle);

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
	if(Sr == I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}



/****************************************************************************************
 *	@fn                  - I2C_MasterReceiveData
 *
 *	@brief               - This function will receive data from I2C slave device over I2C bus
 *
 *	@param[in]           - Handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *	@param[in]           - User given slave address
 *	@param[in]           - Repeated Start enable disable macro
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	// Generate the start condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	// Confirm that START generation is completed by checking the SB flag in the SR1
	// Note: Until SB is cleared SCL will be stretched (Pulled to LOW)
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB) );

	// Send the address of the slave with r/w bit set to 1(Read) (Total 8 bit)
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	// Wait till address phase is completed by checking the ADDR flag in the SR1
	while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR) );

	// Procedure to read only 1 byte from slave
	if(len == 1)
	{
		// Disable Acking
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

		// CLead the ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Wait until RXNE become 1
		while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE) );

		// Generate stop condition
		if(Sr == I2C_DISABLE_SR)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

		// Read data into buffer
		*pRxbuffer = pI2CHandle->pI2Cx->I2C_DR;
	}

	// Procedure to read more than 1 byte from slave
	if(len > 1)
	{
		// Clear ADDR flag
		I2C_ClearADDRFlag(pI2CHandle);

		// Read the data until Len become zero
		for(uint32_t i = len; i > 0; i--)
		{
			// Wait until RXNE becomes 1
			while( ! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE) );

			if(i ==  2) // If last 2 bytes are remaining
			{
				// Clear the ACK bit
				I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

				// Generate STOP condition
				if(Sr == I2C_DISABLE_SR)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

			}

			// Read data from the data register in to RxBuffer
			*pRxbuffer = pI2CHandle->pI2Cx->I2C_DR;

			// Increment the buffer address
			pRxbuffer++;
		}
	}

	// Re-enable ACKing
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{	I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);	}

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
 *	@fn                  - I2C_MasterSendDataIT
 *
 *	@brief               - This function will send data for I2C master device over I2C bus
 *
 *	@param[in]           - Handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *	@param[in]           - User given slave address
 *	@param[in]           - Repeated Start enable disable macro
 *
 *	@return              - Return State of communication (I2C_READY/I2C_BUSY_IN_TX)
 *
 *	@note                - none
 *
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t I2C_state = pI2CHandle->TxRxState;

	if( (I2C_state != I2C_BUSY_IN_TX) && (I2C_state != I2C_BUSY_IN_RX) )
	{
		if(len > 0)
		{
			pI2CHandle->pTxBuffer = pTxbuffer;
			pI2CHandle->TxLen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			// Generate START condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			// Enable ITBUFEN control bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

			// Enable ITEVFEN control bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

			// Enable ITERREN control bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		}
	}

	return I2C_state;
}



/****************************************************************************************
 *	@fn                  - I2C_MasterReceiveDataIT
 *
 *	@brief               - This function will receive data from I2C slave device over I2C bus
 *
 *	@param[in]           - Handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *	@param[in]           - User given slave address
 *	@param[in]           - Repeated Start enable disable macro
 *
 *	@return              - Return State of communication (I2C_READY/I2C_BUSY_IN_RX)
 *
 *	@note                - none
 *
 */
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr)
{
	uint8_t I2C_state = pI2CHandle->TxRxState;

	if( (I2C_state != I2C_BUSY_IN_TX) && (I2C_state != I2C_BUSY_IN_RX))
	{
		if(len > 0)
		{
			pI2CHandle->pRxBuffer = pRxbuffer;
			pI2CHandle->RxLen = len;
			pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
			pI2CHandle->RxSize = len;		// RxSize is used in the ISR code to manage the data reception
			pI2CHandle->DevAddr = SlaveAddr;
			pI2CHandle->Sr = Sr;

			// Generate START condition
			I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

			// Enable ITBUFEN control bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITBUFEN);

			// Enable ITEVFEN control bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITEVTEN);

			// Enable ITERREN control bit
			pI2CHandle->pI2Cx->I2C_CR2 |= (1 << I2C_CR2_ITERREN);
		}

	}

	return I2C_state;
}



/****************************************************************************************
 *	@fn                  - I2C_SlaveSendData
 *
 *	@brief               - This function will send data to the I2C master byte by byte after each TXE interrupt
 *
 *	@param[in]           - I2C peripheral base address
 *	@param[in]           - Data which will be transfer
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
	pI2Cx->I2C_DR = data;
}



/****************************************************************************************
 *	@fn                  - I2C_SlaveReceiveData
 *
 *	@brief               - This function will receive data from I2C master byte by byte after each RXNE interrupt
 *
 *	@param[in]           - I2C peripheral base address
 *
 *	@return              - Data which is received in the DR
 *
 *	@note                - none
 *
 */
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->I2C_DR;
}



/****************************************************************************************
 *	@fn                  - I2C_EV_IRQHandling
 *
 *	@brief               - This function will handle all the event's IRQ
 *
 *	@param[in]           - Handle structure address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	// Interrupt handling for both master and slave mode of a device

	uint32_t temp1, temp2, temp3;

	// Check interrupt event related control bits are set or not
	temp1 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->I2C_CR2 & (1 << I2C_CR2_ITBUFEN);



	// Check SB flag is really set or not
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_SB);

	// 1. Handle for interrupt generated by SB event
	// Note: SB flag is only applicable in Master mode
	if(temp1 && temp3)
	{
		// Interrupt is generated because of SB event
		// This block will not be executed in Slave mode, because for slave SB is always zero
		// In this block lets execute the address phase
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	// Check ADDR flag is really set or not
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_ADDR);

	// 2. Handle for interrupt generated by ADDR event
	//Note: When Master mode : Address is sent
	//	  : When Slave mode : Address matched with own address
	if(temp1 && temp3)
	{
		// ADDR flag is set
		I2C_ClearADDRFlag(pI2CHandle);
	}

	// Check BTF flag is really set or not
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_BTF);

	// 3. Handle for interrupt generated by BTF(Byte Transfer Finished) event
	if(temp1 && temp3)
	{
		// Interrupt is generated because of BTF event

		// Check Application state
		if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
		{
			// Make sure that TXE is also set
			if( I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE) )
			{
				// BTF and TXE both are set
				if(pI2CHandle->TxLen == 0)
				{
					// Check repeated start is enable or not
					if(pI2CHandle->Sr == I2C_DISABLE_SR)
					{
						// 1. Generate STOP condition
						I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
					}

					// 2. Reset all the member elements of the handle structure
					I2C_CloseSendData(pI2CHandle);

					// Notify the application about transmission complete
					I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
				}
			}

		}else if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
		{
			// Nothing to do here
			;
		}

	}

	// Check STOPF flag is really set or not
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_STOPF);

	// 4. Handle for interrupt generated by STOPF event
	// Note: Stop detection flag is applicable only in slave mode. For master this flag will never be set
	// The below code block will not be executed by the master, since STOPF will not set in master mode
	if(temp1 && temp3)
	{
		// STOPF flag is set
		// Clear the STOPF (Read SR1 and then write to CR1)
		// Read SR1 is already done in the above line "Check STOPF flag is really set or not"
		pI2CHandle->pI2Cx->I2C_CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);

	}

	// Check TXE flag is really set or not
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_TxE);

	// 5. Handle for interrupt generated by TXE event
	if(temp1 && temp2 && temp3)
	{
		// TXE flag is set

		// Check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// MASTER mode
			// Check for device state
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
			{
				// We have to do data transmission
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			// SLAVE mode
			// Make sure that SLAVE is really in transmitter mode
			if( pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA) )
			{
				// SLAVE is in transmitter mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
			}
		}
	}

	// Check RXNE flag is really set or not
	temp3 = pI2CHandle->pI2Cx->I2C_SR1 & (1 << I2C_SR1_RxNE);

	// 6. Handle for interrupt generated by RXNE event
	if(temp1 && temp2 && temp3)
	{
		// Check for device mode
		if(pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_MSL))
		{
			// MASTER mode
			// RXNE flag is set
			if(pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
			{
				// We have to do data reception
				I2C_MasterHandleRXNEInterrupt(pI2CHandle);
			}
		}else
		{
			// SLAVE mode
			// Make sure that SLAVE is really in receiver mode
			if(! (pI2CHandle->pI2Cx->I2C_SR2 & (1 << I2C_SR2_TRA)) )
			{
				// SLAVE is in receiver mode
				I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
			}
		}
	}
}



/****************************************************************************************
 *	@fn                  - I2C_ER_IRQHandling
 *
 *	@brief               - This function will handle all the error's IRQ
 *
 *	@param[in]           - Handle structure address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{
	uint32_t temp1,temp2;

	// Know the status of  ITERREN control bit in the CR2
	temp2 = (pI2CHandle->pI2Cx->I2C_CR2) & ( 1 << I2C_CR2_ITERREN);


/***********************Check for Bus error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		// This is Bus error

		// Clear the buss error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_BERR);

		// Notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
	}

/***********************Check for arbitration lost error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		// This is arbitration lost error

		// Clear the arbitration lost error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_ARLO);

		// Notify the application about the error
	   I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
	}

/***********************Check for ACK failure  error************************************/

	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		// This is ACK failure error

		// Clear the ACK failure error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_AF);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
	}

/***********************Check for Overrun/underrun error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		// This is Overrun/underrun

		// Clear the Overrun/underrun error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_OVR);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
	}

/***********************Check for Time out error************************************/
	temp1 = (pI2CHandle->pI2Cx->I2C_SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		// This is Time out error

		// Clear the Time out error flag
		pI2CHandle->pI2Cx->I2C_SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

		// Notify the application about the error
		I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
	}

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
	if(EnOrDis == ENABLE)
	{
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_PE);
		//pI2cBaseAddress->CR1 |= I2C_CR1_PE_Bit_Mask;
	}else
	{
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_PE);
	}
}



/****************************************************************************************
 *	@fn                  - I2C_ManageAcking
 *
 *	@brief               - This function will enable or disable the ACK bit of I2C CR1 register
 *
 *	@param[in]           - Base address of the I2C peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enabling acknowledge by setting ACK bit in CR1
		pI2Cx->I2C_CR1 |= (1 << I2C_CR1_ACK);
	}else
	{
		// Disabling acknowledge by clearing ACK bit in CR1
		pI2Cx->I2C_CR1 &= ~(1 << I2C_CR1_ACK);
	}
}



/****************************************************************************************
 *	@fn                  - I2C_CloseReceiveData
 *
 *	@brief               - This function will close the I2C Interrupt based data reception by resetting all the interrupt bits
 *							and handle structure members
 *
 *	@param[in]           - Address of I2C handle structure
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset handle structure members
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pRxBuffer = NULL;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;

	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
		I2C_ManageAcking(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
}



/****************************************************************************************
 *	@fn                  - I2C_CloseSendData
 *
 *	@brief               - This function will close the I2C Interrupt based data sending by resetting all the interrupt bits
 *							and handle structure members
 *
 *	@param[in]           - Address of I2C handle structure
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
	// Disable ITBUFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	// Disable ITEVFEN control bit
	pI2CHandle->pI2Cx->I2C_CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	// Reset handle structure members
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->pTxBuffer = NULL;
	pI2CHandle->TxLen = 0;

}



// Application Event call back
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	// This is a weak implementation. The application may override this function
}










