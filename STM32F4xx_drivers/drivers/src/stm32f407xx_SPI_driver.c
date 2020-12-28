/*
 * stm32f407xx_SPI_driver.c
 *
 *  Created on: 12-Dec-2020
 *      Author: Sayan Rana
 */

#include "stm32f407xx_SPI_driver.h"

// Helper functions for SPI communication
static void spi_txe_interrput_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_interrput_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrput_handle(SPI_Handle_t *pSPIHandle);



/****************************************************************************************
 *	@fn                  - SPI_PeriClockControl
 *
 *	@brief               - This function will enable or disable peripheral clock for given SPI port
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enable the clock for given SPI port address

		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}else{
		// Disable the clock for given SPI port address

		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}



/****************************************************************************************
 *	@fn                  - SPI_Init
 *
 *	@brief               - This function will initialize a given SPI port
 *
 *	@param[in]           - Will take the handle structure address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	// Enable the peripheral clock
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// First lets configure the SPI_CR1 register

	uint32_t tempReg = 0;

	// 1. Configure the device mode
	tempReg |= (pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR);

	// 2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode should be set
		tempReg |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared
		tempReg &= ~(1 << SPI_CR1_BIDIMODE);

		// RXONLY bit must be set
		tempReg |= (1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the SPI serial clock speed (baud rate)
	tempReg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR);

	// 4. Configure the DFF
	tempReg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF);

	// 5. Configure the CPOL
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL);

	// 6. Configure the CPHA
	tempReg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA);

	// 7. Configure the SSM
	tempReg |= (pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = tempReg;

}



/****************************************************************************************
 *	@fn                  - SPI_DeInit
 *
 *	@brief               - This function will de-initialize a given SPI port
 *
 *	@param[in]           - Base address of the SPI Peripheral
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
}



/****************************************************************************************
 *	@fn                  - SPI_GetFlagStatus
 *
 *	@brief               - This function will return the flag status of status register
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Address of the data (type casted to (uint8_t*)) which we want to send to the slave
 *	@param[in]           - Size of the data which we want to send to the slave
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/****************************************************************************************
 *	@fn                  - SPI_SendData
 *
 *	@brief               - This function will send the given data to slave
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Address of the data (type casted to (uint8_t*)) which we want to send to the slave
 *	@param[in]           - Size of the data which we want to send to the slave
 *
 *	@return              - none
 *
 *	@note                - This is Blocking Call(Polling type function)
 *
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
{
	// Loop until length become zero
	while(len)
	{
		// 1. Wait until TXE is set
		while( SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG) == FLAG_RESET );

		// 2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			// 16 Bit DFF
			// Load the data into the DR
			pSPIx->DR = *((uint16_t*) pTxBuffer);
			len -= 2;
			(uint16_t*)pTxBuffer++;
		}else
		{
			// 8 Bit DFF
			// Load the data into the DR
			pSPIx->DR = *pTxBuffer;
			len--;
			pTxBuffer++;
		}

	}
}



/****************************************************************************************
 *	@fn                  - SPI_ReceiveData
 *
 *	@brief               - This function will receive data from slave
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Address where we want to store the receive data
 *	@param[in]           - Size of the receive data
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
{
	// Loop until length become zero
	while(len)
	{
		// 1. Wait until TXE is set
		while( SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG) == FLAG_RESET );

		// 2. Check the DFF bit in CR1
		if( (pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			// 16 Bit DFF
			// Load the data from DR to RxBuffer address
			*((uint16_t*) pRxBuffer) = pSPIx->DR;
			len -= 2;
			(uint16_t*)pRxBuffer++;
		}else
		{
			// 8 Bit DFF
			// Load the data from DR to RxBuffer address
			*pRxBuffer = pSPIx->DR;
			len--;
			pRxBuffer++;
		}

	}
}



/****************************************************************************************
 *	@fn                  - SPI_PeripheralControl
 *
 *	@brief               - This function will enable or disable SPI communication by setting or reseting SPE Bit in CR1
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}



/****************************************************************************************
 *	@fn                  - SPI_SSIConfig
 *
 *	@brief               - This function will enable or disable the internal slave select by setting or reseting SSI Bit in CR1
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}



/****************************************************************************************
 *	@fn                  - SPI_SSOEConfig
 *
 *	@brief               - This configuration is used only when the device operates in master mode. The
 *						   NSS signal is driven low when the master starts the communication and is kept
 *						   low until the SPI is disabled.
 *
 *	@param[in]           - Base address of the SPI peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}



/****************************************************************************************
 *	@fn                  - SPI_IRQ_IntConfig
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
void SPI_IRQ_IntConfig(uint8_t IRQNumber, uint8_t EnOrDis)
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
 *	@fn                  - SPI_IRQPriorityConfig
 *
 *	@brief               - This function will configure the priority of IRQ
 *
 *	@param[in]           - IRQ Priority number
 *	@param[in]           - IRQ number
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	// 1. Find out IPR Register
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	// 2. Configure the IPR Register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/****************************************************************************************
 *	@fn                  - SPI_SendDataIT
 *
 *	@brief               - This function will the store length info, TxBuffer address in a global variable
 *						   and change the mode to busy and set the TXEIE bit to enable the TXE interrupt
 *
 *	@param[in]           - SPI handle structure address
 *	@param[in]           - Address of the data (type casted to (uint8_t*)) which we want to send to the slave
 *	@param[in]           - Size of the data which we want to send to the slave
 *
 *	@return              - State of the communication (Either busy or ready)
 *
 *	@note                -
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and length information in some global variable
		pSPIHandle->pTXBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;

		// 2. Make the SPI state as busy in transmission so that
		//    no other code can take over the same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in the status register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);

	}

	return state;
}



/****************************************************************************************
 *	@fn                  - SPI_ReceiveDataIT
 *
 *	@brief               - This function will the store length info, RxBuffer address in a global variable
 *						   and change the mode to busy and set the RXNEIE bit to enable the RXNE interrupt
 *
 *	@param[in]           - SPI handle structure address
 *  @param[in]           - Address where we want to store the receive data
 *	@param[in]           - Size of the receive data
 *
 *	@return              - State of the communication (Either busy or ready)
 *
 *	@note                - none
 *
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		// 1. Save the Rx buffer address and length information in some global variable
		pSPIHandle->pRXBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;

		// 2. Make the SPI state as busy in reception so that
		//    no other code can take over the same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever RXNE flag is set in the status register
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

	}
	return state;
}



/****************************************************************************************
 *	@fn                  - SPI_IRQHandling
 *
 *	@brief               - This function will handle the IRQ
 *
 *	@param[in]           - SPI handle structure address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp1, temp2;

	// Check for TXE
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE));			// Checking TXE bit of SR(Set or not)
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE));		// Checking TXEIE bit of CR2(Set or not)

	if(temp1 && temp2)
	{
		// Handle TXE
		spi_txe_interrput_handle(pSPIHandle);
	}


	// Check for RXNE
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE));			// Checking RXNE bit of SR(Set or not)
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE));		// Checking RXNEIE bit of CR2(Set or not)

	if(temp1 && temp2)
	{
		// Handle RXNE
		spi_rxne_interrput_handle(pSPIHandle);
	}


	// Check for Over run flag
	temp1 = (pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR));			// Checking OVR bit of SR(Set or not)
	temp2 = (pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE));		// Checking ERRIR bit of CR2(Set or not)

	if(temp1 && temp2)
	{
		// Handle Over_run error
		spi_ovr_err_interrput_handle(pSPIHandle);
	}

}

// Helper function implementations
static void spi_txe_interrput_handle(SPI_Handle_t *pSPIHandle)
{
	// Check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
	{
		// 16 Bit DFF
		// Load the data into the DR
		pSPIHandle->pSPIx->DR = *((uint16_t*) pSPIHandle->pTXBuffer);
		pSPIHandle->TxLen -= 2;
		(uint16_t*)pSPIHandle->pTXBuffer++;
	}else
	{
		// 8 Bit DFF
		// Load the data into the DR
		pSPIHandle->pSPIx->DR = *pSPIHandle->pTXBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTXBuffer++;
	}

	if(!pSPIHandle->TxLen)
	{
		// If TxLen is zero, close the SPI transmission and inform the application that TX is over

		SPI_CloseTransmission(pSPIHandle);

		// Inform application that TX is complete
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interrput_handle(SPI_Handle_t *pSPIHandle)
{
		// Check the DFF bit in CR1
		if( (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF) ) )
		{
			// 16 Bit DFF
			// Load the data from DR to RxBuffer address
			*((uint16_t*) pSPIHandle->pRXBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen -= 2;
			(uint16_t*)pSPIHandle->pRXBuffer++;
		}else
		{
			// 8 Bit DFF
			// Load the data from DR to RxBuffer address
			*(pSPIHandle->pRXBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
			pSPIHandle->RxLen--;
			pSPIHandle->pRXBuffer++;
		}

		if(!pSPIHandle->RxLen)
		{
			// If RxLen is zero, close the SPI reception and inform the application that RX is over

			SPI_CloseReception(pSPIHandle);

			// Inform application that RX is complete
			SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		}
}


static void spi_ovr_err_interrput_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;

	// Clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{

		temp = pSPIHandle->pSPIx->DR;			// Read the data register
		temp = pSPIHandle->pSPIx->SR;			// Read the status register
	}
	(void)temp;

	// Inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}


// Function for clearing overrun flag by the application code
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;

	// Clear the OVR flag
	temp = pSPIx->DR;			// Read the data register
	temp = pSPIx->SR;			// Read the status register

	(void)temp;
}


// Function for closing SPI interrupt based transmission
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);		// Disable further SPI TXEIE interrupt
	pSPIHandle->pTXBuffer = NULL;							// Initialize pTXBuffer as NULL
	pSPIHandle->TxLen = 0;									// Setting TxLen to zero
	pSPIHandle->TxState = SPI_READY;						// Setting TxState to Ready

}


// Function for closing SPI interrupt based reception
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);		// Disable further SPI RXNEIE interrupt
	pSPIHandle->pRXBuffer = NULL;							// Initialize pRXBuffer as NULL
	pSPIHandle->RxLen = 0;									// Setting RxLen to zero
	pSPIHandle->RxState = SPI_READY;						// Setting RxState to Ready
}


// Application Event call back
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent)
{
	// This is a weak implementation. The application may override this function
}



