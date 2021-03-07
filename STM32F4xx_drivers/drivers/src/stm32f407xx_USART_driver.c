/*
 * stm32f407xx_USART_driver.c
 *
 *  Created on: 17-Feb-2021
 *      Author: Sayan Rana
 */


#include "stm32f407xx_USART_driver.h"

// Clear the IDLE flag
static void USART_ClearIDLEflag(USART_RegDef_t *pUSARTx);






// A static function to clear the IDLE flag
static void USART_ClearIDLEflag(USART_RegDef_t *pUSARTx)
{
	uint32_t dummyRead;

	dummyRead = pUSARTx->USART_SR;
	dummyRead = pUSARTx->USART_DR;

	(void)dummyRead;
}



/****************************************************************************************
 *	@fn                  - USART_PeriClockControl
 *
 *	@brief               - This function will enable or disable peripheral clock for given USART port
 *
 *	@param[in]           - Base address of the USART peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enable the clock for given USART port address

		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}else
	{
		// Disable the clock for given USART port address

		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}else if(pUSARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}



/****************************************************************************************
 *	@fn                  - USART_SetBaudRate
 *
 *	@brief               - This function will configure the baud rate by configuring the USART Baud Rate register
 *
 *	@param[in]           - Base address of the USART peripheral
 *	@param[in]           - Desired baud rate value
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate)
{
	// Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartDiv;

	// Variable to hold the Mantissa and Fraction value
	uint32_t M_part, F_part;

	uint32_t tempreg = 0;

 //	Get the value of APB bus clock into the variable PCLKx
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		// USART1 and USART6 are hanging on APB2 bus
		PCLKx = RCC_GetPCLK2Value();
	}else{
		// USART2, USART3, USART4 and USART5 all are hanging on APB1 bus
		PCLKx = RCC_GetPCLK1Value();
	}

	// Check for OVER8 configuration bit
	if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
	{
		// OVER8 = 1, Over sampling by 8
		usartDiv = ( (25 * PCLKx)/(2 * BaudRate) );  // usartDiv = ( (PCLKx/(8*BaudRate)) * 100)
	}else {
		// OVER8 = 0, Over sampling by 16
		usartDiv = ( (25 * PCLKx)/(4 * BaudRate) );  // usartDiv = ( (PCLKx/(16*BaudRate)) * 100)
	}

	// Calculating the Mantissa Part
	M_part = usartDiv/100;

	// Place the mantissa part in appropriate bit position. @USART_BRR
	tempreg |= (M_part << 4);

	// Extract the fraction part
	F_part = (usartDiv - (M_part * 100));

	// Calculate the fractional part
	if(pUSARTx->USART_CR1 & (1 << USART_CR1_OVER8))
	{
		// OVR8 = 1, over sampling by 8
		F_part = ( ( (F_part * 8) + 50) / 100) & ((uint8_t)0x07);
	}else
	{
		// OVER = 0, over sampling by 16
		F_part = ( ( (F_part * 16) + 50) / 100) & ((uint8_t)0x0F);
	}

	// Placing the fractional part in appropriate bit position. @USART_BRR
	tempreg |= F_part;

	// Copy the value of tempreg in to USART_BRR register
	pUSARTx->USART_BRR = tempreg;
}



/****************************************************************************************
 *	@fn                  - USART_Init
 *
 *	@brief               - This function will initialize a given USART port
 *
 *	@param[in]           - Will take the handle structure address
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{
	// Enable the clock for given USART peripheral
	USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

/******************************** Configuration of CR1******************************************/
	// Temporary variable
	uint32_t tempreg = 0;

	// Enable the USART Tx and Rx engines according to the USART mode configuration item
	if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_Rx) {
		// Configure the RE bit
		tempreg |= (1 << USART_CR1_RE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_Tx) {
		// Configure the TE bit
		tempreg |= (1 << USART_CR1_TE);
	}else if(pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TxRx) {
		// Configure both TE and RE bits
		tempreg |= ( (1 << USART_CR1_TE) | (1 << USART_CR1_RE) );
	}

	// Word length configuration
	tempreg |= (pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M);

	// Configuration of parity control bit
	if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN) {
		// Enable the parity control
		tempreg |= (1 << USART_CR1_PCE);
		// No need to configure the EVEN parity, because by default EVEN parity is enabled
	}else if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD) {
		// Enable the parity control
		tempreg |= (1 << USART_CR1_PCE);
		// Enable the ODD parity
		tempreg |= (1 << USART_CR1_PS);
	}

	// Program the CR1 register
	pUSARTHandle->pUSARTx->USART_CR1 = tempreg;

/******************************** Configuration of CR2******************************************/
	tempreg = 0;

	// Configure the number of stop bits inserted during USART frame transmission
	tempreg |= (pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP);

	// Program the CR2 register
	pUSARTHandle->pUSARTx->USART_CR2 = tempreg;

/******************************** Configuration of CR3******************************************/
	tempreg =0;

	// Configure the USART hardware flow control
	if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS) {
		// Enable the CTS flow control
		tempreg |= (1 << USART_CR3_CTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HE_FLOW_CTRL_RTS) {
		// Enable RTS flow control
		tempreg |= (1 << USART_CR3_RTSE);
	}else if(pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS) {
		// Enable both CTS and RTS
		tempreg |= ( (1 << USART_CR3_CTSE) | (1 << USART_CR3_RTSE) );
	}
	pUSARTHandle->pUSARTx->USART_CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	// Configure the baud rate
	USART_SetBaudRate(pUSARTHandle->pUSARTx, pUSARTHandle->USART_Config.USART_Baud);
}



/****************************************************************************************
 *	@fn                  - USART_DeInit
 *
 *	@brief               - This function will de-initialize a given USART port
 *
 *	@param[in]           - Base address of the USART Peripheral
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_DeInit(USART_RegDef_t *pUSARTx)
{
	if(pUSARTx == USART1)
	{
		USART1_REG_RESET();
	}else if(pUSARTx == USART2)
	{
		USART2_REG_RESET();
	}else if(pUSARTx == USART3)
	{
		USART3_REG_RESET();
	}else if(pUSARTx == UART4)
	{
		UART4_REG_RESET();
	}else if(pUSARTx == UART5)
	{
		UART5_REG_RESET();
	}else if(pUSARTx == USART6)
	{
		USART6_REG_RESET();
	}
}



/****************************************************************************************
 *	@fn                  - USART_SendData
 *
 *	@brief               - This function will transmit data
 *
 *	@param[in]           - USART handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint16_t *pData;

	// Loop over until "Len" number of bytes are transfered
	for(uint32_t i=0; i<Len; i++) {
		// Wait until TXE flag is set
		while( !USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TxE) );

		// Check the USART_WordLength member for 9BIT or 8BIT in a frame
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			// If 9BIT, load the DR with 2Bytes. Masking the bits other than first 9BITs
			pData = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->USART_DR = (*pData & (uint16_t)0x01FF);

			// Check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity is used in this transfer. So 9 bits of user data will be sent.

				// Increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}else{
				// Parity bit is used in this transfer. So 8 bits of user data will be sent

				// The 9th bit will be replaced by parity bit by hardware
				pTxBuffer++;
			}
		}else{
			// 8 Bit communication is set

			// This is 8 Bit data transfer
			pUSARTHandle->pUSARTx->USART_DR = (*pTxBuffer & (uint8_t)0xFF);

			// Increment the buffer address
			pTxBuffer++;
		}
	}

	// Wait till TC fag is set in the SR
	while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC) );
}



/****************************************************************************************
 *	@fn                  - USART_ReceiveData
 *
 *	@brief               - This function will receive data
 *
 *	@param[in]           - USART handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_ReceiveData(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	// Loop until "Len" number of bytes are received
	for(uint32_t i=0; i<Len; i++) {
		// Wait until RXNE flag is set in the SR
		while(! USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RxNE) );

		// Check USART_WordLength to decide whether we are going to receive 9 bit data in a frame or 8 bit
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS) {
			// We are going to receive 9 bit data in a frame

			// Check we are using USART_ParityControl or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity is used, all 9 bits will be of user data

				// Read only first 9 Bits. So mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x01FF);
				// Now increment pRxBuffer two times
				pRxBuffer++;
				pRxBuffer++;
			}else{
				// Parity is used, so 8 bit will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
				// Increment pRxBuffer once
				pRxBuffer++;
			}
		}else{
			// We are going to receive 8 bit data in a frame

			// Check we are using USART_ParityControl or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) {
				// No parity is used, all 8 bits will be of user data

				// Read 8 Bits from DR
				*pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
			}else{
				// Parity is used, So 7 Bits will be of user data and one bit is parity

				// Read only 7 bits, Hence mask the DR with 0x7F
				*pRxBuffer = (uint8_t)(pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
			}

			// Increment pRxBuffer
			pRxBuffer++;
		}
	}
}



/****************************************************************************************
 *	@fn                  - USART_SendDataIT
 *
 *	@brief               - This function will transmit data
 *
 *	@param[in]           - Handle structure address
 *	@param[in]           - Transmit buffer address(Where the user given data is stored)
 *	@param[in]           - Length of the data in byte
 *
 *	@return              - Return State of communication (READY/BUSY_IN_TX)
 *
 *	@note                - Interrupt based transmission (Non blocking method)
 *
 */
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t txState = pUSARTHandle->TxState;

	if(txState != USART_BUSY_IN_TX)
	{
		pUSARTHandle->TxLen = Len;
		pUSARTHandle->pTxBuffer = pTxBuffer;
		pUSARTHandle->TxState = USART_BUSY_IN_TX;

		// Enable the interrupt for TXE
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TXEIE);

		// Enable interrupt for TC
		pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_TCIE);
	}
	return txState;
}



/****************************************************************************************
 *	@fn                  - USART_ReceiveDataIT
 *
 *	@brief               - This function will receive data
 *
 *	@param[in]           - Handle structure address
 *	@param[in]           - Receive buffer address(Where the received data is stored)
 *	@param[in]           - Length of the data in byte
 *
 *	@return              - Return State of communication (READY/BUSY_IN_RX)
 *
 *	@note                - Interrupt based reception (Non blocking method)
 *
 */
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t rxState = pUSARTHandle->RxState;

	 if(rxState != USART_BUSY_IN_RX)
	 {
		 pUSARTHandle->RxLen = Len;
		 pUSARTHandle->pRxBuffer = pRxBuffer;
		 pUSARTHandle->RxState = USART_BUSY_IN_RX;

		 (void)pUSARTHandle->pUSARTx->USART_DR;

		 // Enable interrupt for RXNE
		 pUSARTHandle->pUSARTx->USART_CR1 |= (1 << USART_CR1_RXNEIE);
	 }
	return rxState;
}



/****************************************************************************************
 *	@fn                  - USART_IRQ_InterruptConfig
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
void USART_IRQ_InterruptConfig(uint8_t IRQNumber, uint8_t EnOrDis)
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
 *	@fn                  - USART_IRQPriorityConfig
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	// 1. Find out IPR Register
	uint8_t iprx = (IRQNumber / 4);
	uint8_t iprx_section = (IRQNumber % 4);

	// 2. Configure the IPR Register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount);
}



/****************************************************************************************
 *	@fn                  - USART_IRQHandling
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
void USART_IRQHandling(USART_Handle_t *pUSARTHandle)
{
	uint32_t temp1, temp2, temp3;
	uint16_t *pdata;

/*****************************************  Check for TC flag ******************************************/

	// Check the state of TC bit in the SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TC);
	// Check the state of TCIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TCIE);

	if(temp1 && temp2)
	{
		// This interrupt is because of TC

		// Close transmission and call application callback if TxLen is zero
		if(pUSARTHandle->TxState == USART_BUSY_IN_TX)
		{
			// Check TxLen. If it is zero then close the data transmission
			if(! pUSARTHandle->TxLen)
			{
				// Clear the TC flag
				USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_TC);

				// Clear the TCIE control bit
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TCIE);

				// Reset the application state
				pUSARTHandle->TxState = USART_READY;

				// Reset buffer address to NULL
				pUSARTHandle->pTxBuffer = NULL;

				// Reset the length to zero
				pUSARTHandle->TxLen = 0;

				// Call the application callback with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

/*****************************************  Check for TXE flag ******************************************/

	// Check the state of TXE bit in SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_TxE);

	// Check the state of TXEIE bit CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_TXEIE);

	if(temp1 && temp2)
	{
		// This interrupt is because of TXE

		if(pUSARTHandle->TxState == USART_BUSY_IN_TX)
		{
			// Keep sending the data until TxLen reaches to zero
			if(pUSARTHandle->TxLen > 0)
			{
				// Check USART_WordLength member for 9BIT or 8BIT in a frame
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// if 9BIT, load the DR with 2 bytes. Masking the bits other than first 9 bits
					pdata = (uint16_t*) pUSARTHandle->pTxBuffer;

					// Load only first 9 bits, so we have to mask with the value 0x01FF;
					pUSARTHandle->pUSARTx->USART_DR = (*pdata & (uint16_t)0x01FF);

					// Check for USART_ParityControl
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used in this transfer, so 9 bits of user data will be sent
						// Increment the TxBuffer twice
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->pTxBuffer++;

						// Decrement the length twice
						pUSARTHandle->TxLen -= 2;
					}else
					{
						// Parity bit is used in this transfer. So, 8 bits of user data will be sent
						// The 9th bit will be replaces by parity bit by the hardware
						pUSARTHandle->pTxBuffer++;
						pUSARTHandle->TxLen--;
					}
				}else
				{
					// This is 8 bit data transfer
					pUSARTHandle->pUSARTx->USART_DR = (*pUSARTHandle->pTxBuffer & (uint8_t)0xFF);

					// Increment buffer address
					pUSARTHandle->pTxBuffer++;

					// Decrement the length
					pUSARTHandle->TxLen--;
				}
			}

			// If TxLen reaches to zero
			if(pUSARTHandle->TxLen == 0)
			{
				// TxLen is zero
				// Clear TXEIE bit (disable interrupt for TXE flag)
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

/*****************************************  Check for RXNE flag ******************************************/
	// Check the state of RXNE bit in SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_RxNE);

	// Check the state of RXNEIE bit CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		// This interrupt is because of RXNE

		if(pUSARTHandle->RxState == USART_BUSY_IN_RX)
		{
			// Keep receiving the data until RxLen reaches to zero
			if(pUSARTHandle->RxLen > 0)
			{
				// Check USART_WordLength member to decide whether we are going to receive 9BIT of data in a frame or 8BIT
				if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					// We are going to receive 9 bit data in a frame

					// Now, check are we using USART_ParityControl or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used. So all 9 bits will be our user data

						// Read only first 9 bits, So mask the DR with 0x01FF
						*((uint16_t*)pUSARTHandle->pRxBuffer) = (pUSARTHandle->pUSARTx->USART_DR & (uint16_t)0x01FF);

						// Increment the pRxBuffer two times
						pUSARTHandle->pRxBuffer++;
						pUSARTHandle->pRxBuffer++;

						// Decrement the length
						pUSARTHandle->RxLen -= 2;
					}else
					{
						// Parity is used. So, 8 bits will be of user data and 1 bit is parity
						*pUSARTHandle->pRxBuffer = (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);

						// Increment the pRxBuffer
						pUSARTHandle->pRxBuffer++;

						// Decrement the RxLen
						pUSARTHandle->RxLen--;
					}
				}else
				{
					// We are going to receive 8 bit of data in a frame

					// Now, check are we using USART_ParityControl or not
					if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						// No parity is used, so all 8 bits will be of user data

						// Read 8 bits from DR
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0xFF);
					}else
					{
						// Parity is used, So 7 bits will be of user data and 1 bit is parity

						// Read only 7 bits, Hence mask the DR with 0x7F
						*pUSARTHandle->pRxBuffer = (uint8_t) (pUSARTHandle->pUSARTx->USART_DR & (uint8_t)0x7F);
					}

					// Increment the pRxBuffer
					pUSARTHandle->pRxBuffer++;

					// Decrement the RxLen
					pUSARTHandle->RxLen--;
				}
			}

			// If RxLen reaches to zero
			if(pUSARTHandle->RxLen == 0)
			{
				// RxLen is zero

				// Clear RXNEIE bit (disable interrupt for RXNE flag)
				pUSARTHandle->pUSARTx->USART_CR1 &= ~(1 << USART_CR1_RXNEIE);

				// Reset the application state
				pUSARTHandle->RxState = USART_READY;

				// Reset buffer address to NULL
				pUSARTHandle->pRxBuffer = NULL;

				// Call the application callback with event USART_EVENT_RX_CMPLT
				USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}

/*****************************************  Check for CTS flag ******************************************/
//	Note: CTS feature is not applicable for UART4 and UART5

	// Check the status of the CTS bit in SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_CTS);

	// Check the state of CTSE bit in CR3
	temp2 = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSE);

	// Check the state of CTSIE bit in CR3
	temp3 = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_CTSIE);

	if(temp1 && temp2 && temp3)
	{
		// This interrupt is because of CTS

		// Clear CTS flag in SR
		USART_ClearFlag(pUSARTHandle->pUSARTx, USART_FLAG_CTS);

		// Call the application callback with event USART_EVENT_CTS
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_CTS);
	}

/*****************************************  Check for IDLE flag ******************************************/

	// Check the status of IDLE flag bit in SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_IDLE);

	// Check the state of IDLEIE bit in CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_IDLEIE);

	if(temp1 && temp2)
	{
		// This interrupt is because of IDLE

		// Clear the IDLE flag
		USART_ClearIDLEflag(pUSARTHandle->pUSARTx);

		// Call the application callback with event USART_EVENT_IDLE
		USART_ApplicationEventCallback(pUSARTHandle, USART_EVENT_IDLE);
	}

/*****************************************  Check for Overrun detection flag ******************************************/

	// Check the status of ORE flag in SR
	temp1 = USART_GetFlagStatus(pUSARTHandle->pUSARTx, USART_FLAG_ORE);

	// Check the state of the RXNEIE bit in the CR1
	temp2 = pUSARTHandle->pUSARTx->USART_CR1 & (1 << USART_CR1_RXNEIE);

	if(temp1 && temp2)
	{
		// This interrupt is because of Overrun Error

		// Need not to clear the ORE flag here, instead give an API for the application to clear the ORE flag
		// Call the application callback with the event USART_ERR_ORE
		USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
	}

/*****************************************  Check for Error flag ******************************************/
// Noise Flag, Overrun Error and Framing Error in multibuffer communication
// The below code will get executed in only if multibuffer mode is used

	temp2 = pUSARTHandle->pUSARTx->USART_CR3 & (1 << USART_CR3_EIE);

	if(temp2)
	{
		temp1 = pUSARTHandle->pUSARTx->USART_SR;

		if(temp1 & (1 << USART_SR_FE))
		{
			/*
			 * This bit is set by hardware when a de-synchronization, excessive noise or a break character
			 * is detected. It is cleared by a software sequence (an read to the USART_SR register followed
			 * by a read to the USART_DR register).
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_FE);
		}

		if(temp1 & (1 << USART_SR_NF))
		{
			/*
			 * This bit is set by hardware when noise is detected on a received frame. It is cleared by a
			 * software sequence (an read to the USART_SR register followed by a read to the USART_DR
			 * register
			 */
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_NF);
		}

		if(temp1 & (1 << USART_SR_ORE))
		{
			USART_ApplicationEventCallback(pUSARTHandle, USART_ERR_ORE);
		}
	}
}



/****************************************************************************************
 *	@fn                  - USART_PeripheralControl
 *
 *	@brief               - This function will enable or disable USART peripheral by setting or reseting UE Bit in CR1
 *
 *	@param[in]           - Base address of the USART peripheral
 *	@param[in]           - Enable or Disable macros
 *
 *	@return              - none
 *
 *	@note                - none
 *
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDis)
{
	if(EnOrDis == ENABLE)
	{
		// Enable USART peripheral
		pUSARTx->USART_CR1 |= (1 << USART_CR1_UE);
	}else
	{
		// Disable USART peripheral
		pUSARTx->USART_CR1 &= ~(1 << USART_CR1_UE);
	}
}



/****************************************************************************************
 *	@fn                  - USART_GetFlagStatus
 *
 *	@brief               - This function will return the flag status of status register
 *
 *	@param[in]           - Base address of the USART peripheral
 *	@param[in]           - In stm32f407xx_USART_driver.h  @USART related status flags definitions
 *
 *	@return              - Either flag reset or set
 *
 *	@note                - none
 *
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint32_t FlagName)
{
	if(pUSARTx->USART_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}



/*********************************************************************
 * @fn      		  - USART_ClearFlag
 *
 * @brief             -
 *
 * @param[in]         - Base address of the USART peripheral
 * @param[in]         - In stm32f407xx_USART_driver.h  @USART related status flags definitions
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - Applicable to only USART_FLAG_CTS , USART_FLAG_LBD
 * 						USART_FLAG_TC
 *

 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t FlagName)
{
	pUSARTx->USART_SR &= ~(FlagName);
}




__weak void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle, uint8_t AppEvent)
{
	// This is a weak implementation. The application may override this function
}





















