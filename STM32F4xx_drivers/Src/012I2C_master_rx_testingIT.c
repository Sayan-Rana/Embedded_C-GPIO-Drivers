/*
 * 012I2C_master_rx_testingIT.c
 *
 *  Created on: 04-Feb-2021
 *      Author: Sayan Rana
 *
 *      Interrupt based communication
 */

#include <stdio.h>
#include <string.h>
#include <stm32f407xx.h>

/*
 * PB10-------> SCL
 * PB11-------> SDA
 */

// Flag Variable
uint8_t RxComplt = RESET;

// Own device address(Only used in slave mode)
#define MY_ADDR				0x61
// Slave address(Only used in master mode)
#define SLAVE_ADDR			0x68


I2C_Handle_t I2C2Handle;

// rcv_buffer
char rcv_buf[32];


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


void I2C2_GPIOInit(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOpType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunction = 4;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
	GPIO_Init(&I2CPins);
}


void I2C2_Init(void)
{
	I2C2Handle.pI2Cx = I2C2;
	I2C2Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	I2C2Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C2Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C2Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C2Handle);
}


int main()
{
	uint8_t commandCode, len;

	printf("Communication started\n");

	//GPIO Button init
	GPIO_ButtonInit();

	// I2C pin init
	I2C2_GPIOInit();

	// I2C Peripheral configuration
	I2C2_Init();

	// I2C IRQ configuration
	I2C_IRQ_IntConfig(IRQ_NO_I2C2_EV, ENABLE);
	I2C_IRQ_IntConfig(IRQ_NO_I2C2_ER, ENABLE);

	// Enable the I2C peripheral
	I2C_PeripheralControl(I2C2, ENABLE);

	// Enabling ACK after PE = 1;
	I2C_ManageAcking(I2C2, I2C_ACK_ENABLE);

	// Wait for button press
	while(1)
	{
		// Wait till button is pressed
		while(!( GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) ) );

		// to avoid button de-bouncing related issues 200ms of delay
		delay();

		commandCode = 0x51;		// Command code to get length information from slave

		// Master send command code to slave for length information
		while(I2C_MasterSendDataIT(&I2C2Handle, &commandCode, 1, SLAVE_ADDR,I2C_ENABLE_SR) != I2C_READY);

		// Master reading response(Length info) from slave
		while(I2C_MasterReceiveDataIT(&I2C2Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		commandCode = 0x52;		// Command code to initiate the data reception from slave

		// Master sends command code to slave for length number of data reception
		while(I2C_MasterSendDataIT(&I2C2Handle, &commandCode, 1, SLAVE_ADDR, I2C_ENABLE_SR) != I2C_READY);

		// Master receiving data from slave(length of data = len)
		while(I2C_MasterReceiveDataIT(&I2C2Handle, (uint8_t*)rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR) != I2C_READY);

		RxComplt = RESET;

		// Wait till RX complete
		while(RxComplt != SET);

		rcv_buf[len+1] = '\0';

		printf("Data : %s\n",rcv_buf);

		RxComplt = RESET;

	}

}


void I2C2_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C2Handle);
}


void I2C2_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C2Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEvent)
{
	if(AppEvent == I2C_EV_TX_CMPLT)
	{
		printf("Tx is completed\n");

	}else if(AppEvent == I2C_EV_RX_CMPLT)
	{
		printf("Rx is completed\n");
		RxComplt = SET;

	}else if(AppEvent == I2C_ERROR_AF)
	{
		printf("ERROR : Ack Failure\n");
		// In master ACK failure happens when slave fails to send ACK for the byte
		// sent from master.
		I2C_CloseSendData(&I2C2Handle);

		// Generate stop condition to release the bus
		I2C_GenerateStopCondition(I2C2);

		// Hang in infinite loop
		while(1);
	}
}













