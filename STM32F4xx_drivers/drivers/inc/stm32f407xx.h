/*
 * stm32f407xx.h
 *
 *  Created on: Nov 28, 2020
 *      Author: Sayan Rana
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo				volatile

/******************************************** START : Processor Specific Details *************************************************
 *
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0								((__vo uint32_t*) 0xE000E100)
#define NVIC_ISER1								((__vo uint32_t*) 0xE000E104)
#define NVIC_ISER2								((__vo uint32_t*) 0xE000E108)
#define NVIC_ISER3								((__vo uint32_t*) 0xE000E10C)


/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0								((__vo uint32_t*) 0XE000E180)
#define NVIC_ICER1								((__vo uint32_t*) 0XE000E184)
#define NVIC_ICER2								((__vo uint32_t*) 0XE000E188)
#define NVIC_ICER3								((__vo uint32_t*) 0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address
 */

#define NVIC_IPR_BASE_ADDR						((__vo uint32_t*) 0xE000E400)

/*
 * ARM Cortex Mx Processor, number of priority bits implemented in priority register
 * This is vendor specific, in STs case it's 4 for TI it's 3
 */
#define NO_PR_BITS_IMPLEMENTED					4


/**********************************************************************************************************************************
 *
 */



/*
 * BAse address of Flash and SRAM memories
 */

#define FLASH_BASEADDR					0x08000000U							// Base address of flash memory
#define SRAM1_BASEADDR					0x20000000U							// Base address of SRAM1
#define SRAM1_SIZE						(112 * 1024)U						// SRAM1 size is 112KB
#define SRAM2_BASEADDR					(SRAM1_BASEADDR + SRAM1_SIZE)		// SRAM2 base address and it's size is 16KB
#define ROM_BASEADDR					0x1FFF0000U							// System memory(ROM) base address


/*
 * AHBx and APBx Bus peripheral base addresses
 */

#define PERIPH_BASEADDR						0x40000000U
#define APB1PERIPH_BASEADDR					PERIPH_BASEADDR					// Base address of APB1 bus related peripherals
#define APB2PERIPH_BASEADDR					0x40010000U						// Base address of APB2 bus related peripherals
#define AHB1PERIPH_BASEADDR					0x40020000U						// Base address of AHB1 bus related peripherals
#define AHB2PERIPH_BASEADDR					0x50000000U						// Base address of AHB2 bus related peripherals


/*
 * Base addresses of peripherals which are hanging on AHB1 Bus
 */

#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000)		// GPIOA is hanging on the 1st position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400)		// GPIOB is hanging on the 2nd position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800)		// GPIOC is hanging on the 3rd position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00)		// GPIOD is hanging on the 4th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000)		// GPIOE is hanging on the 5th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400)		// GPIOF is hanging on the 6th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800)		// GPIOG is hanging on the 7th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00)		// GPIOH is hanging on the 8th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)
#define GPIOI_BASEADDR					(AHB1PERIPH_BASEADDR + 0x2000)		// GPIOI is hanging on the 9th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)

#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800)		// RCC is hanging on the 13th position of AHB1 Bus (AHB1 Bus peripheral base address + ofFset)

/*
 * Base addresses of peripherals which are hanging on APB1 Bus
 */

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400)		// Base address of APB1 bus peripherals + offset I2C1
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800)		// Base address of APB1 bus peripherals + offset I2C2
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00)		// Base address of APB1 bus peripherals + offset I2C3
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800)		// Base address of APB1 bus peripherals + offset SPI2
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00)		// Base address of APB1 bus peripherals + offset SPI3
#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400)		// Base address of APB1 bus peripherals + offset USART2
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800)		// Base address of APB1 bus peripherals + offset USART3
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00)		// Base address of APB1 bus peripherals + offset UART4
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000)		// Base address of APB1 bus peripherals + offset UART5


/*
 * Base addresses of peripherals which are hanging on APB2 Bus
 */

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000)		// Base address of APB2 bus peripherals + offset SPI1
#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000)		// Base address of APB2 bus peripherals + offset USART1
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400)		// Base address of APB2 bus peripherals + offset USART6
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)		// Base address of APB2 bus peripherals + offset EXT Interrupt
#define	SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800)		// Base address of APB2 bus peripherals + offset SYS Configuration


/************************************Peripheral registers definition structure******************************************/
/*
 * This structure is only for STM32F407VG microcontroller AHB1 Bus GPIO peripheral
 */

typedef struct
{
	__vo uint32_t MODER;				/* GPIO port mode register,																					 Address offset : 0x00 */
	__vo uint32_t OTYPER;				/* GPIO port output type register,																			 Address offset : 0x04 */
	__vo uint32_t OSPEEDR;				/* GPIO port output speed register,																			 Address offset : 0x08 */
	__vo uint32_t PUPDR;				/* GPIO port pull-up/pull-down register																		 Address offset : 0x0C */
	__vo uint32_t IDR;					/* GPIO port input data register,																			 Address offset : 0x10 */
	__vo uint32_t ODR;					/* GPIO port output data register,																			 Address offset : 0x14 */
	__vo uint32_t BSRR;					/* GPIO port bit set/reset register,																	   	 Address offset : 0x18 */
	__vo uint32_t LCKR;					/* GPIO port configuration lock register,																	 Address offset : 0x1C */
	__vo uint32_t AFR[2];				/* AFR[0] : GPIO alternate function low register, AFR[1] : GPIO alternate function high register,			 Address offset: 0x20 and 0x24 */
}GPIO_RegDef_t;


/*
 * This structure is only for STM32F407VG microcontroller AHB1 Bus RCC peripheral
 */

typedef struct
{
	__vo uint32_t CR;					/* RCC clock control register,					Address offset : 0x00 */
	__vo uint32_t PLLCFGR;				/* RCC PLL configuration register,				Address offset : 0x04 */
	__vo uint32_t CFGR;					/* RCC clock configuration register,			Address offset : 0x08 */
	__vo uint32_t CIR;					/* RCC clock interrupt register,				Address offset : 0x0C */
	__vo uint32_t AHB1RSTR;				/* RCC AHB1 peripheral reset register,			Address offset : 0x10 */
	__vo uint32_t AHB2RSTR;				/* RCC AHB2 peripheral reset register,			Address offset : 0x14 */
	__vo uint32_t AHB3RSTR;				/* RCC AHB3 peripheral reset register,			Address offset : 0x18 */
	__vo uint32_t Reserved0;			/* Reserved 4 Byte*/
	__vo uint32_t APB1RSTR;				/* RCC APB1 peripheral reset register,			Address offset : 0x20 */
	__vo uint32_t APB2RSTR;				/* RCC APB2 peripheral reset register,			Address offset : 0x24 */
	__vo uint32_t Reserved1[2];			/* Reserved 8 Byte */
	__vo uint32_t AHB1ENR;				/* RCC AHB1 peripheral clock enable register,	Address offset : 0x30 */
	__vo uint32_t AHB2ENR;				/* RCC AHB2 peripheral clock enable register,	Address offset : 0x34 */
	__vo uint32_t AHB3ENR;				/* RCC AHB3 peripheral clock enable register,	Address offset : 0x38 */
	__vo uint32_t Reserved2;			/* Reserved 4 Byte */
	__vo uint32_t APB1ENR;				/* RCC APB1 peripheral clock enable register,	Address offset : 0x40 */
	__vo uint32_t APB2ENR;				/* RCC APB2 peripheral clock enable register,	Address offset : 0x44 */
	__vo uint32_t Reserved3[2];			/* Reserved 8 Byte */
	__vo uint32_t AHB1LPENR;			/* RCC AHB1 peripheral clock enable in low power mode register,		Address offset : 0x50 */
	__vo uint32_t AHB2LPENR;			/* RCC AHB2 peripheral clock enable in low power mode register,		Address offset : 0x54 */
	__vo uint32_t AHB3LPENR;			/* RCC AHB3 peripheral clock enable in low power mode register,		Address offset : 0x58 */
	__vo uint32_t Reserved4;			/* Reserved  4 Byte */
	__vo uint32_t APB1LPENR;			/* RCC APB1 peripheral clock enable in low power mode register,		Address offset : 0x60 */
	__vo uint32_t APB2LPENR;			/* RCC APB2 peripheral clock enable in low power mode register,		Address offset : 0x64 */
	__vo uint32_t Reserved5[2];			/* Reserved 8 Byte */
	__vo uint32_t BDCR;					/* RCC Backup domain control register,			Address offset : 0x70 */
	__vo uint32_t CSR;					/* RCC clock control & status register,			Address offset : 0x74 */
	__vo uint32_t Reserved6[2];			/* Reserved  8 Byte */
	__vo uint32_t SSCGR;				/* RCC spread spectrum clock generation register,					Address offset : 0x80 */
	__vo uint32_t PLLI2SCFGR;			/* RCC PLLI2S configuration register,			Address offset : 0x84 */
}RCC_RegDef_t;


/*
 * This structure is only for STM32F407VG microcontroller APB2 Bus EXTI peripheral
 */

typedef struct
{
	__vo uint32_t EXTI_IMR;					/* EXTI Interrupt mask register,							Address offset : 0x00 */
	__vo uint32_t EXTI_EMR;					/* EXTI Event mask register,								Address offset : 0x04 */
	__vo uint32_t EXTI_RTSR;					/* EXTI Rising trigger selection register,					Address offset : 0x08 */
	__vo uint32_t EXTI_FTSR;					/* EXTI Falling trigger selection register,					Address offset : 0x0C */
	__vo uint32_t EXTI_SWIER;				/* EXTI Software interrupt event register,					Address offset : 0x10 */
	__vo uint32_t EXTI_PR;					/* EXTI Pending register,									Address offset : 0x14 */
}EXTI_RegDef_t;


/*
 * This structure is only for STM32F407VG microcontroller APB2 Bus SYSCFG peripheral
 */

typedef struct
{
	__vo uint32_t MEMRMP;					/* SYSCFG memory remap register,																		Address offset : 0x00 */
	__vo uint32_t PMC;						/* SYSCFG peripheral mode configuration register,														Address offset : 0x04 */
	__vo uint32_t EXTICR[4];				/* SYSCFG external interrupt configuration register [0] =reg1, [1] = reg2, [2] = reg3, [3] = reg4,		Address offset : 0x08 */
	uint32_t 	  RESERVED1[2];
	__vo uint32_t CMPCR;					/* SYSCFG Compensation cell control register,															Address offset : 0x0C */
}SYSCFG_RegDef_t;


/*
 * Peripheral definition (Peripheral base addresses tpyecasted to xxx_RegDef_t)
 */

//GPIO
#define GPIOA							((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB							((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC							((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD							((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE							((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF							((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG							((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH							((GPIO_RegDef_t*) GPIOH_BASEADDR)
#define GPIOI							((GPIO_RegDef_t*) GPIOI_BASEADDR)

//RCC
#define RCC								((RCC_RegDef_t*) RCC_BASEADDR)

//EXTI
#define EXTI							((EXTI_RegDef_t*) EXTI_BASEADDR)

//SYSCFG
#define SYSCFG							((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)


/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()					(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()					(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()					(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()					(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()					(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()					(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()					(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()					(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()					(RCC->AHB1ENR |= (1 << 8))


/*
 * Clock Enable Macro for I2Cx peripheral
 */

#define I2C1_PCLK_EN()					(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()					(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()					(RCC->APB1ENR |= (1 << 23))


/*
 * Clock Enable Macro for SPIx peripheral
 */

#define SPI1_PCLK_EN()					(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()					(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()					(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable Macro for USARTx peripheral
 */

#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1 << 18))
#define USART6_PCLK_EN()				(RCC->APB2ENR |= (1 << 5))


/*
 * Clock Enable Macro for SYSCNFG peripheral
 */

#define SYSCFG_PCLK_EN()				(RCC->APB2ENR |= (1 << 14))


/*
 * Clock Disable Macro for GPIOx peripheral
 */

#define GPIOA_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()					(RCC->AHB1ENR &= ~(1 << 8))


/*
 * Clock Disable Macro for I2Cx peripheral
 */

#define I2C1_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 23))


/*
 * Clock Disable Macro for SPIx peripheral
 */

#define SPI1_PCLK_DI()					(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 15))


/*
 * Clock Disable Macro for USARTx peripheral
 */

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 18))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable Macro for SYSCNFG peripheral
 */

#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))


/*
 * Macros to reset GPIOx peripherals
 * Using do while condition zero loop
 */
#define GPIOA_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 0);	RCC->AHB1RSTR &= ~(1 << 0);	}while(0)			/* By setting the bit position we can reset(disable) the GPIOx peripheral clock */
#define GPIOB_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 1);	RCC->AHB1RSTR &= ~(1 << 1);	}while(0)						/* and after that we must have to clear the bit position */
#define GPIOC_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 2);	RCC->AHB1RSTR &= ~(1 << 2);	}while(0)
#define GPIOD_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 3);	RCC->AHB1RSTR &= ~(1 << 3);	}while(0)
#define GPIOE_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 4);	RCC->AHB1RSTR &= ~(1 << 4);	}while(0)
#define GPIOF_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 5);	RCC->AHB1RSTR &= ~(1 << 5);	}while(0)
#define GPIOG_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 6);	RCC->AHB1RSTR &= ~(1 << 6);	}while(0)
#define GPIOH_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 7);	RCC->AHB1RSTR &= ~(1 << 7);	}while(0)
#define GPIOI_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 8);	RCC->AHB1RSTR &= ~(1 << 8);	}while(0)


/*
 * Macro to convert GPIO base address to port code of SYSCFG external interrupt configuration register
 */
#define GPIO_BASEADDR_to_CODE(x)				  ( (x == GPIOA)?0:\
													(x == GPIOB)?1:\
													(x == GPIOC)?2:\
													(x == GPIOD)?3:\
													(x == GPIOE)?4:\
													(x == GPIOF)?5:\
													(x == GPIOG)?6:\
													(x == GPIOH)?7:\
													(x == GPIOI)?8:0 )


/*
 * IRQ(Interrupt Request) Numbers of STM32F407x MCU
 */

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define	IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40


/*
 * NVIC IRQ priority macros
 */

#define NVIC_IRQ_PRI0				0
#define NVIC_IRQ_PRI1				1
#define NVIC_IRQ_PRI2				2
#define NVIC_IRQ_PRI3				3
#define NVIC_IRQ_PRI4				4
#define NVIC_IRQ_PRI5				5
#define NVIC_IRQ_PRI6				6
#define NVIC_IRQ_PRI7				7
#define NVIC_IRQ_PRI8				8
#define NVIC_IRQ_PRI9				9
#define NVIC_IRQ_PRI10				10
#define NVIC_IRQ_PRI11				11
#define NVIC_IRQ_PRI12				12
#define NVIC_IRQ_PRI13				13
#define NVIC_IRQ_PRI14				14
#define NVIC_IRQ_PRI15				15


/*
 * Some generic macros
 */

#define ENABLE						1
#define DISABLE						0
#define SET							ENABLE
#define RESET						DISABLE
#define GPIO_PIN_SET				SET
#define GPIO_PIN_RESET				RESET

#include "stm32f407xx_GPIO_driver.h"


#endif /* INC_STM32F407XX_H_ */
