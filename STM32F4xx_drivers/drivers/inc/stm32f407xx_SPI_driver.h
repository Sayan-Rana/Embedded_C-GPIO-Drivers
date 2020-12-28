/*
 * stm32f407xx_SPI_driver.h
 *
 *  Created on: 12-Dec-2020
 *      Author: Sayan Rana
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"


/*
 * Configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;									// Possible value from @SPI_DeviceMode
	uint8_t SPI_BusConfig;									// Possible value from @SPI_BusConfig
	uint8_t SPI_SclkSpeed;									// Possible value from @SPI_SclkSpeed
	uint8_t SPI_DFF;										// Possible value from @SPI_DFF
	uint8_t SPI_CPOL;										// Possible value from @SPI_CPOL
	uint8_t SPI_CPHA;										// Possible value from @SPI_CPHA
	uint8_t SPI_SSM;										// Possible value from @SPI_SSM
}SPI_Config_t;


/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t 	*pSPIx;				/*!< This holds the base address of SPIx(1,2,3) peripheral >*/
	SPI_Config_t 	SPIConfig;			/*!< This holds SPI peripheral configuration settings >*/
	uint8_t 		*pTXBuffer;			/*!< To store the application Tx buffer address >*/
	uint8_t			*pRXBuffer;			/*!< To store the application Rx buffer address >*/
	uint8_t			TxLen;				/*!< To store Tx Length >*/
	uint8_t			RxLen;				/*!< To store Rx Length >*/
	uint8_t			TxState;			/*!< To store Tx State >*/
	uint8_t			RxState;			/*!< To store Rx State >*/
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER						1
#define SPI_DEVICE_MODE_SLAVE						0

/*
 * @SPI_BusConfig
 */
#define SPI_BUS_CONFIG_FD							1					// Full-duplex mode
#define SPI_BUS_CONFIG_HD							2					// Half-duplex mode
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY				3					// Simplex receive only mode

/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2							0					// Peripheral clock divided by 2
#define SPI_SCLK_SPEED_DIV4							1					// Peripheral clock divided by 4
#define SPI_SCLK_SPEED_DIV8							2					// Peripheral clock divided by 8
#define SPI_SCLK_SPEED_DIV16						3					// Peripheral clock divided by 16
#define SPI_SCLK_SPEED_DIV32						4					// Peripheral clock divided by 32
#define SPI_SCLK_SPEED_DIV64						5					// Peripheral clock divided by 64
#define SPI_SCLK_SPEED_DIV128						6					// Peripheral clock divided by 128
#define SPI_SCLK_SPEED_DIV256						7					// Peripheral clock divided by 256

/*
 * @SPI_DFF
 */
#define SPI_DFF_8BIT								0					// Data frame format 8 bit
#define SPI_DFF_16BIT								1					// Data frame format 16 bit

/*
 * @SPI_CPOL
 */
#define SPI_CPOL_LOW								0					// CK to Low when idle
#define SPI_CPOL_HIGH								1					// CK to High when idle

/*
 * @SPI_CPHA
 */
#define SPI_CPHA_LOW								0					// The first clock transition is the first data capture edge
#define SPI_CPHA_HIGH								1					// The second clock transition is the first data capture edge

/*
 * @SPI_SSM
 */
#define SPI_SSM_DI									0					// Software slave management disabled
#define SPI_SSM_EN									1					// Software slave management enabled


/*
 * SPI related status flags definitions
 */
#define SPI_RXNE_FLAG 								(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG 								(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG 							(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG 								(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG 							(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG 								(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG 								(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG 								(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG 								(1 << SPI_SR_FRE)


/*
 * SPI Application State
 */
#define SPI_READY									0
#define SPI_BUSY_IN_RX								1
#define SPI_BUSY_IN_TX								2



/*
 * Possible SPI application events
 */
#define SPI_EVENT_TX_CMPLT					1
#define SPI_EVENT_RX_CMPLT					2
#define SPI_EVENT_OVR_ERR					3
#define SPI_EVENT_CRC_ERR					4


/*********************************************************************************************************************
 * 												APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 * *******************************************************************************************************************
 */


/*
 * Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);


/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 * Test status register flag status
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName);

/*
 * Data send and receive
 */

// Blocking or Polling method
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);


// Interrupt method
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);


/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQ_IntConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);



/*
 * Application call back
 */
__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEvent);


/*
 * Other peripheral control APIs
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDis);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);


#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
