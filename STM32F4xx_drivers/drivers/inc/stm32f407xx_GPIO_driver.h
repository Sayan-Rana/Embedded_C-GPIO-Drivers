/*
 * stm32f407xx_GPIO_driver.h
 *
 *  Created on: 29-Nov-2020
 *      Author: Sayan Rana
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


#include "stm32f407xx.h"

/*
 * This is a Configuration structure for GPIO pins
 */

typedef struct
{
	uint8_t GPIO_PinNumber;								// Possible value from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;								// Possible value from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;								// Possible value from @GPIO_PIN_SPEED
	uint8_t GPIO_PuPdControl;							// Possible value from @GPIO_PIN_PU_PD_CONTROL
	uint8_t GPIO_PinOpType;								// Possible value from @GPIO_PIN_OP_TYPE
	uint8_t GPIO_PinAltFunction;						// Possible value from @GPIO_PIN_ALTFN
}GPIO_PinConfig_t;


/*
 * This is a Handle structure for GPIO pins.
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;   // This holds the base address of the GPIO post to which the pin belong.
	GPIO_PinConfig_t GPIO_PinConfig;  // This holds GPIO pin configuration settings.
}GPIO_Handle_t;


/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN							0	// Input mode
#define GPIO_MODE_OUT							1	// General purpose output mode
#define GPIO_MODE_ALTFN							2	// Alternate function mode
#define GPIO_MODE_ANALOG						3	// Analog mode

/*
 * @GPIO_PIN_MODE_INT
 */
#define GPIO_MODE_INT_FT						4	// Interrupt input mode falling edge trigger
#define GPIO_MODE_INT_RT						5	// Interrupt input mode	rising edge trigger
#define GPIO_MODE_INT_RFT						6	// Interrupt input mode	rising and falling edge trigger

/*
 * @GPIO_PIN_OP_TYPE
 * GPIO possible output types
 */
#define GPIO_OP_TYPE_PP							0	// Output type push-pull
#define GPIO_OP_TYPE_OD							1	// Output type open drain

/*
 * @GPIO_PIN_SPEED
 * GPIO possible output speed
 */
#define GPIO_SPEED_LOW							0	// Output speed low
#define GPIO_SPEED_MEDIUM						1	// Output speed medium
#define GPIO_SPEED_FAST							2	// Output speed fast
#define GPIO_SPEED_HIGH							3	// Output speed high

/*
 * @GPIO_PIN_PU_PD_CONTROL
 * GPIO pins pull up and pull down configuration macros
 */
#define GPIO_NO_PUPD							0	// No pull up pull down
#define GPIO_PIN_PU								1	// Pull up
#define GPIO_PIN_PD								2	// Pull down

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0							0
#define GPIO_PIN_NO_1							1
#define GPIO_PIN_NO_2							2
#define GPIO_PIN_NO_3							3
#define GPIO_PIN_NO_4							4
#define GPIO_PIN_NO_5							5
#define GPIO_PIN_NO_6							6
#define GPIO_PIN_NO_7							7
#define GPIO_PIN_NO_8							8
#define GPIO_PIN_NO_9							9
#define GPIO_PIN_NO_10							10
#define GPIO_PIN_NO_11							11
#define GPIO_PIN_NO_12							12
#define GPIO_PIN_NO_13							13
#define GPIO_PIN_NO_14							14
#define GPIO_PIN_NO_15							15


/*
 * @GPIO_PIN_ALTFN
 * GPIO possible alternate functions
 */
#define GPIO_ALTFN_AF0							0		//Alternate Function 0
#define GPIO_ALTFN_AF1							1		//Alternate Function 1
#define GPIO_ALTFN_AF2							2		//Alternate Function 2
#define GPIO_ALTFN_AF3							3		//Alternate Function 3
#define GPIO_ALTFN_AF4							4		//Alternate Function 4
#define GPIO_ALTFN_AF5							5		//Alternate Function 5
#define GPIO_ALTFN_AF6							6		//Alternate Function 6
#define GPIO_ALTFN_AF7							7		//Alternate Function 7
#define GPIO_ALTFN_AF8							8		//Alternate Function 8
#define GPIO_ALTFN_AF9							9		//Alternate Function 9
#define GPIO_ALTFN_AF10							10		//Alternate Function 10
#define GPIO_ALTFN_AF11							11		//Alternate Function 11
#define GPIO_ALTFN_AF12							12		//Alternate Function 12
#define GPIO_ALTFN_AF13							13		//Alternate Function 13
#define GPIO_ALTFN_AF14							14		//Alternate Function 14
#define GPIO_ALTFN_AF15							15		//Alternate Function 15



/*************************************************************************************************
 *                                 APIs supported by the driver
 *                 For more information about the APIs check the function definitions
 *************************************************************************************************
 */

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDis);


/*
 * Init and De-Init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data Read Write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);


/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQ_IntConfig(uint8_t IRQNumber, uint8_t EnOrDis);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);


#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */

