/*
 * 013I2C_slave_tx_string.c
 *
 *  Created on: 10-Feb-2021
 *      Author: Sayan Rana
 *
 *      Interrupt based communication
 */

#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>

/*
 * PB6-------> SCL
 * PB7-------> SDA
 */

// Own device address(Only used in slave mode)
#define MY_ADDR				0x68


I2C_Handle_t I2C1Handle;

// SLAVE Tx buffer
char Tx_buf[32] = "STM32 Slave mode testing..";


// Software delay for testing purpose
void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}


void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpioBtn;
	//Button(Input) pin configuration.
	gpioBtn.pGPIOx = GPIOA;
	gpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioBtn.GPIO_PinConfig.GPIO_PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&gpioBtn);
}


void I2C1_GPIOInit(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunction = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}


void I2C1_Init(void)
{
	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);
}


int main()
{
	printf("Hello World!\n");

	//GPIO Button init
	GPIO_ButtonInit();

	// I2C pin init
	I2C1_GPIOInit();

	// I2C Peripheral configuration
	I2C1_Init();

	// I2C IRQ configuration
	I2C_IRQ_IntConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQ_IntConfig(IRQ_NO_I2C1_ER, ENABLE);

	// Enabling interrupt control bits in I2C CR2 register
	I2C_SlaveEnableDisableCallbackEvents(I2C1, ENABLE);

	// Enable the I2C peripheral
	I2C_PeripheralControl(I2C1, ENABLE);

	// Enabling ACK after PE = 1;
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	// Wait for button press
	while(1);
}


void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	static uint8_t commandCode = 0;
	static uint8_t count = 0;

	if(AppEvent == I2C_EV_DATA_REQ)
	{
		// Master wants some data, Slave has to send it
		if(commandCode == 0x51)
		{
			// Send the length information to the master
			I2C_SlaveSendData(pI2CHandle->pI2Cx, strlen((char*)Tx_buf));

		}else if(commandCode == 0x52)
		{
			// Send the content of Tx_buf
			I2C_SlaveSendData(pI2CHandle->pI2Cx, Tx_buf[count++]);
		}

	}else if(AppEvent == I2C_EV_DATA_RCV)
	{
		// Data is waiting for the slave to read, Slave has to read it
		commandCode = I2C_SlaveReceiveData(pI2CHandle->pI2Cx);

	}else if(AppEvent == I2C_ERROR_AF)
	{
		// This happens only in slave transmission.
		// Master has sent NACK, Slave should understand master does not need more data.
		commandCode = 0xFF;		// Invalidate the commandCode after it is bing used
		count = 0;

	}else if(AppEvent == I2C_EV_STOP)
	{
		// This happens only during slave reception.
		// Master has ended the I2C communication with slave.

	}
}













