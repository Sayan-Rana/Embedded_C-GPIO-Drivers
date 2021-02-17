/*
 * stm32f407xx_I2C_driver.h
 *
 *  Created on: 08-Jan-2021
 *      Author: Sayan Rana
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "stm32f407xx.h"



/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t	    I2C_SCLSpeed;									// Possible value from @I2C_SCLSpeed
	uint8_t 		I2C_DeviceAddress;								// User will provide this value
	uint8_t 		I2C_ACKControl;									// Possible value from @I2C_ACKControl
	uint16_t 		I2C_FMDutyCycle;								// Possible value from @I2C_FMDutyCycle
}I2C_Config_t;


/*
 * Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;				/*!< This holds the base address of I2Cx(1,2,3) peripheral >*/
	I2C_Config_t 	I2C_Config;			/*!< This holds I2C peripheral configuration settings >*/
	uint8_t 		*pTxBuffer;			/*!< To store the application Tx Buffer address >*/
	uint8_t			*pRxBuffer;			/*!< To store the application Rx buffer address >*/
	uint32_t		TxLen;				/*!< To store Tx length >*/
	uint32_t		RxLen;				/*!< To store Rx length >*/
	uint8_t			TxRxState;			/*!< To store communication state @I2C_application_state >*/
	uint8_t			DevAddr;			/*!< To store slave/device address >*/
	uint32_t		RxSize;				/*!< To store Rx size >*/
	uint8_t			Sr;					/*!< To store repeated start value >*/
}I2C_Handle_t;


/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_SM						100000UL
#define I2C_SCL_SPEED_FM2K						200000UL
#define I2C_SCL_SPEED_FM4K						400000UL


/*
 * @I2C_ACKControl
 */
#define I2C_ACK_ENABLE							1
#define I2C_ACK_DISABLE							0


/*
 * @I2C_FMDutyCycle
 */
#define I2C_FM_DUTY_2							0
#define I2C_FM_DUTY_16_9						1


/*
 * @I2C related status flags definitions
 */
#define I2C_FLAG_SB									(1 << I2C_SR1_SB)
#define I2C_FLAG_ADDR								(1 << I2C_SR1_ADDR)
#define I2C_FLAG_BTF								(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADD10								(1 << I2C_SR1_ADD10)
#define I2C_FLAG_STOPF								(1 << I2C_SR1_STOPF)
#define I2C_FLAG_RxNE								(1 << I2C_SR1_RxNE)
#define I2C_FLAG_TxE								(1 << I2C_SR1_TxE)
#define I2C_FLAG_BERR								(1 << I2C_SR1_BERR)
#define I2C_FLAG_ARLO								(1 << I2C_SR1_ARLO)
#define I2C_FLAG_AF									(1 << I2C_SR1_AF)
#define I2C_FLAG_OVR								(1 << I2C_SR1_OVR)
#define I2C_FLAG_PECERR								(1 << I2C_SR1_PECERR)
#define I2C_FLAG_TIMEOUT							(1 << I2C_SR1_TIMEOUT)
#define I2C_FLAG_SMBALERT							(1 << I2C_SR1_SMBALERT)

/*
 * @I2C Repeated Start Enable/Disable macros
 */
#define I2C_ENABLE_SR					SET
#define I2C_DISABLE_SR					RESET

/*
 * @I2C_application_state
 */
#define I2C_READY						0
#define I2C_BUSY_IN_RX					1
#define I2C_BUSY_IN_TX					2

/*
 * Possible I2C application events macros
 */
#define I2C_EV_TX_CMPLT					0
#define I2C_EV_RX_CMPLT					1
#define I2C_EV_STOP						2
#define I2C_ERROR_BERR					3
#define I2C_ERROR_ARLO					4
#define I2C_ERROR_AF					5
#define I2C_ERROR_OVR   				6
#define I2C_ERROR_TIMEOUT 				7
#define I2C_EV_DATA_REQ					8
#define I2C_EV_DATA_RCV					9


/*********************************************************************************************************************
 * 												APIs supported by this driver
 * 							For more information about the APIs check the function definitions
 * *******************************************************************************************************************
 */


/*
 * Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);


/*
 * Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Test status register flag status
 */
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName);

/*
 * Data send and receive(Polling Based)
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);

/*
 * Data send and receive(Interrupt Based)
 */
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxbuffer, uint32_t len, uint8_t SlaveAddr, uint8_t Sr);


void I2C_SlaveSendData(I2C_RegDef_t *pI2Cx, uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2Cx);


void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);


/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQ_IntConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle);



/*
 * Application callback
 */
__weak void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent);


/*
 * Other peripheral control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);
void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDis);


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
