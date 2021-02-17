/*
 * stm32f407xx.h
 *
 *  Created on: Nov 28, 2020
 *      Author: Sayan Rana
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stddef.h>
#include <stdint.h>

#define __vo				volatile
#define __weak				__attribute__((weak))

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
 * Base address of Flash and SRAM memories
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
#define SPI4_BASEADDR					(APB2PERIPH_BASEADDR + 0x3400)		// Base address of APB2 bus peripherals + offset SPI4
#define SPI5_BASEADDR					(APB2PERIPH_BASEADDR + 0x5000)		// Base address of APB2 bus peripherals + offset SPI5
#define SPI6_BASEADDR					(APB2PERIPH_BASEADDR + 0x5400)		// Base address of APB2 bus peripherals + offset SPI6
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
	__vo uint32_t EXTI_RTSR;				/* EXTI Rising trigger selection register,					Address offset : 0x08 */
	__vo uint32_t EXTI_FTSR;				/* EXTI Falling trigger selection register,					Address offset : 0x0C */
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
 * This structure is only for STM32F407VG microcontroller SPIx peripheral
 */
typedef struct
{
	__vo uint32_t CR1;					/* SPI control register 1,							Address offset : 0x00 */
	__vo uint32_t CR2;					/* SPI control register 2,							Address offset : 0x04 */
	__vo uint32_t SR;					/* SPI status register,								Address offset : 0x08 */
	__vo uint32_t DR;					/* SPI data register,								Address offset : 0x0C */
	__vo uint32_t CRCPR;				/* SPI CRC polynomial register,						Address offset : 0x10 */
	__vo uint32_t RXCRCR;				/* SPI RX CRC register,								Address offset : 0x14 */
	__vo uint32_t TXCRCR;				/* SPI_I2S configuration register,					Address offset : 0x18 */
	__vo uint32_t I2SCFGR;				/* SPI_I2S configuration register,					Address offset : 0x1C */
	__vo uint32_t I2SPR;				/* SPI_I2S prescaler register,						Address offset : 0x20 */
}SPI_RegDef_t;


/*
 * This structure is only for STM32F407VG microcontroller I2Cx peripheral
 */
typedef struct
{
	__vo uint32_t I2C_CR1;				/* I2C Control register 1,							Address offset : 0x00 */
	__vo uint32_t I2C_CR2;				/* I2C Control register 2,							Address offset : 0x04 */
	__vo uint32_t I2C_OAR1;				/* I2C Own address register 1,						Address offset : 0x08 */
	__vo uint32_t I2C_OAR2;				/* I2C Own address register 2,						Address offset : 0x0C */
	__vo uint32_t I2C_DR;				/* I2C Data register,								Address offset : 0x10 */
	__vo uint32_t I2C_SR1;				/* I2C Status register 1,							Address offset : 0x14 */
	__vo uint32_t I2C_SR2;				/* I2C Status register 2,							Address offset : 0x18 */
	__vo uint32_t I2C_CCR;				/* I2C Clock control register,						Address offset : 0x1C */
	__vo uint32_t I2C_TRISE;			/* I2C_TRISE register,								Address offset : 0x20 */
	__vo uint32_t I2C_FLTR;				/* I2C_FLTR register,								Address offset : 0x24 */
}I2C_RegDef_t;


/*
 * This structure is only for STM32F407VG microcontroller USART/UART peripheral
 */
typedef struct
{
	__vo uint32_t USART_SR;				/* USART Status register,							Address offset : 0x00 */
	__vo uint32_t USART_DR;				/* USART Data register,								Address offset : 0x04 */
	__vo uint32_t USART_BRR;			/* USART Baud rate register,						Address offset : 0x08 */
	__vo uint32_t USART_CR1;			/* USART Control register 1,						Address offset : 0x0C */
	__vo uint32_t USART_CR2;			/* USART Control register 2,						Address offset : 0x10 */
	__vo uint32_t USART_CR3;			/* USART Control register 3,						Address offset : 0x14 */
	__vo uint32_t USART_GTPR;			/* USART Guard time and prescaler register,			Address offset : 0x18 */
}USART_RegDef_t;


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

//SPI
#define SPI1							((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2							((SPI_RegDef_t*) SPI2_BASEADDR)
#define SPI3							((SPI_RegDef_t*) SPI3_BASEADDR)

// I2C
#define I2C1							((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2							((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3							((I2C_RegDef_t*) I2C3_BASEADDR)

// USART
#define USART1							((USART_Regdef_t*) USART1_BASEADDR)
#define USART2							((USART_Regdef_t*) USART2_BASEADDR)
#define USART3							((USART_Regdef_t*) USART3_BASEADDR)
#define UART4							((USART_Regdef_t*)  UART4_BASEADDR)
#define UART5							((USART_Regdef_t*)  UART5_BASEADDR)
#define USART6							((USART_Regdef_t*) USART6_BASEADDR)


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
 * Clock Enable Macro for USARTx/UARTx peripheral
 */

#define USART1_PCLK_EN()				(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()				(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()				(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()					(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()					(RCC->APB1ENR |= (1 << 20))
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
 * Clock Disable Macro for USARTx/UARTx peripheral
 */

#define USART1_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()				(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()					(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 5))


/*
 * Clock Disable Macro for SYSCNFG peripheral
 */

#define SYSCFG_PCLK_DI()				(RCC->APB2ENR &= ~(1 << 14))


/*
 * Macros to reset GPIOx peripheral
 * Using do while condition zero loop
 */
#define GPIOA_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 0);	RCC->AHB1RSTR &= ~(1 << 0);	}while(0)			/* By setting the bit position we can reset(disable) the GPIOx peripheral clock */
#define GPIOB_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 1);	RCC->AHB1RSTR &= ~(1 << 1);	}while(0)			/* and after that we must have to clear the bit position */
#define GPIOC_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 2);	RCC->AHB1RSTR &= ~(1 << 2);	}while(0)
#define GPIOD_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 3);	RCC->AHB1RSTR &= ~(1 << 3);	}while(0)
#define GPIOE_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 4);	RCC->AHB1RSTR &= ~(1 << 4);	}while(0)
#define GPIOF_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 5);	RCC->AHB1RSTR &= ~(1 << 5);	}while(0)
#define GPIOG_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 6);	RCC->AHB1RSTR &= ~(1 << 6);	}while(0)
#define GPIOH_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 7);	RCC->AHB1RSTR &= ~(1 << 7);	}while(0)
#define GPIOI_REG_RESET()	do{	RCC->AHB1RSTR |= (1 << 8);	RCC->AHB1RSTR &= ~(1 << 8);	}while(0)


/*
 * Macros to reset SPIx peripheral
 * Using do while condition zero loop
 */
#define SPI1_REG_RESET()	do{	RCC->APB2RSTR |= (1 << 12);	RCC->APB2RSTR &= ~(1 << 12);	}while(0)			/* By setting the bit position we can reset(disable) the SPIx peripheral clock */
#define SPI2_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 14);	RCC->APB1RSTR &= ~(1 << 14);	}while(0)			/* and after that we must have to clear the bit position */
#define SPI3_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 15);	RCC->APB1RSTR &= ~(1 << 15);	}while(0)


/*
 * Macros to reset I2Cx peripheral
 * Using do while condition zero loop
 */
#define I2C1_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 21);	RCC->APB1RSTR &= ~(1 << 21);	}while(0)			/* By setting the bit position we can reset(disable) the I2Cx peripheral clock */
#define I2C2_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 22);	RCC->APB1RSTR &= ~(1 << 22);	}while(0)			/* and after that we must have to clear the bit position */
#define I2C3_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 23);	RCC->APB1RSTR &= ~(1 << 23);	}while(0)


/*
 * Macros to reset USARTx/UARTx peripheral
 * Using do while condition zero loop
 */
#define USART1_REG_RESET()	do{	RCC->APB2RSTR |= (1 << 4);	RCC->APB2RSTR &= ~(1 << 4);		}while(0)			/* By setting the bit position we can reset(disable) the USARTx/UARTx peripheral clock */
#define USART2_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 17);	RCC->APB1RSTR &= ~(1 << 17);	}while(0)			/* and after that we must have to clear the bit position */
#define USART3_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 18);	RCC->APB1RSTR &= ~(1 << 18);	}while(0)
#define UART4_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 19);	RCC->APB1RSTR &= ~(1 << 19);	}while(0)
#define UART5_REG_RESET()	do{	RCC->APB1RSTR |= (1 << 20);	RCC->APB1RSTR &= ~(1 << 20);	}while(0)
#define USART6_REG_RESET()	do{	RCC->APB2RSTR |= (1 << 5);	RCC->APB2RSTR &= ~(1 << 5);		}while(0)


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
 * IRQ(Interrupt Request) Numbers  of EXTI interrupts asper STM32F407x MCU Vector table
 */

#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define	IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_EXTI15_10			40


/*
 * IRQ(Interrupt Request) Numbers  of SPI interrupts asper STM32F407x MCU Vector table
 */

#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define	IRQ_NO_SPI3					51


/*
 * IRQ(Interrupt Request) Numbers  of I2C interrupts asper STM32F407x MCU Vector table
 */

#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73


/*
 * IRQ(Interrupt Request) Numbers  of USART/UART interrupts asper STM32F407x MCU Vector table
 */

#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define	IRQ_NO_USART3				39
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_USART6				71


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
#define FLAG_RESET					RESET
#define FLAG_SET					SET


/*************************************************************************************************************
 * Bit position definition of SPI peripheral registers
 ************************************************************************************************************/
//CR1
#define SPI_CR1_CPHA												0
#define SPI_CR1_CPOL												1
#define SPI_CR1_MSTR												2
#define SPI_CR1_BR													3
#define SPI_CR1_SPE													6
#define SPI_CR1_LSBFIRST											7
#define SPI_CR1_SSI													8
#define SPI_CR1_SSM													9
#define SPI_CR1_RXONLY												10
#define SPI_CR1_DFF													11
#define SPI_CR1_CRCNEXT												12
#define SPI_CR1_CRCEN												13
#define SPI_CR1_BIDIOE												14
#define SPI_CR1_BIDIMODE											15

//CR2
#define SPI_CR2_RXDMAEN												0
#define SPI_CR2_TXDMAEN												1
#define SPI_CR2_SSOE												2
#define SPI_CR2_FRF													4
#define SPI_CR2_ERRIE												5
#define SPI_CR2_RXNEIE												6
#define SPI_CR2_TXEIE												7

//SR
#define SPI_SR_RXNE													0
#define SPI_SR_TXE													1
#define SPI_SR_CHSIDE												2
#define SPI_SR_UDR													3
#define SPI_SR_CRCERR												4
#define SPI_SR_MODF													5
#define SPI_SR_OVR													6
#define SPI_SR_BSY													7
#define SPI_SR_FRE													8

//I2SCFGR
#define SPI_I2SCFGR_CHLEN											0
#define SPI_I2SCFGR_DATLEN											1
#define SPI_I2SCFGR_CKPOL											3
#define SPI_I2SCFGR_I2SSTD											4
#define SPI_I2SCFGR_PCMSYNC											7
#define SPI_I2SCFGR_I2SCFG											8
#define SPI_I2SCFGR_I2SE											10
#define SPI_I2SCFGR_I2SMOD											11

//I2SPR
#define SPI_I2SPR_I2SDIV											0
#define SPI_I2SPR_ODD												8
#define SPI_I2SPR_MCKOE												9


/*************************************************************************************************************
 * Bit position definition of I2C peripheral registers
 ************************************************************************************************************/
//I2C_CR1
#define I2C_CR1_PE													0
#define I2C_CR1_SMBUS												1
#define I2C_CR1_SMBTYPE												3
#define I2C_CR1_ENARP												4
#define I2C_CR1_ENPEC												5
#define I2C_CR1_ENGC												6
#define I2C_CR1_NOSTRETCH											7
#define I2C_CR1_START												8
#define I2C_CR1_STOP												9
#define I2C_CR1_ACK													10
#define I2C_CR1_POS													11
#define I2C_CR1_PEC													12
#define I2C_CR1_ALERT												13
#define I2C_CR1_SWRST												15

//I2C_CR2
#define I2C_CR2_FREQ												0
#define I2C_CR2_ITERREN												8
#define I2C_CR2_ITEVTEN												9
#define I2C_CR2_ITBUFEN												10
#define I2C_CR2_DMAEN												11
#define I2C_CR2_LAST												12

//I2C_OAR1
#define I2C_OAR1_ADD0												0
#define I2C_OAR1_ADD7_1												1
#define I2C_OAR1_ADD9_8												8
#define I2C_OAR1_ADDMODE											15

//I2C_OAR2
#define I2C_OAR2_ENDUAL												0
#define I2C_OAR2_ADD27_1											1

//I2C_SR1
#define I2C_SR1_SB													0
#define I2C_SR1_ADDR												1
#define I2C_SR1_BTF													2
#define I2C_SR1_ADD10												3
#define I2C_SR1_STOPF												4
#define I2C_SR1_RxNE												6
#define I2C_SR1_TxE													7
#define I2C_SR1_BERR												8
#define I2C_SR1_ARLO												9
#define I2C_SR1_AF													10
#define I2C_SR1_OVR													11
#define I2C_SR1_PECERR												12
#define I2C_SR1_TIMEOUT												14
#define I2C_SR1_SMBALERT											15

//I2C_SR2
#define I2C_SR2_MSL													0
#define I2C_SR2_BUSY												1
#define I2C_SR2_TRA													2
#define I2C_SR2_GENCALL												4
#define I2C_SR2_SMBDEFAULT											5
#define I2C_SR2_SMBHOST												6
#define I2C_SR2_DUALF												7
#define I2C_SR2_PEC													8

//I2C_CCR
#define I2C_CCR_CCR11_0												0
#define I2C_CCR_DUTY												14
#define I2C_CCR_F_OR_S												15

//I2C_TRISE
#define I2C_TRISE_TRISE5_0											0

//I2C_FLTR
#define I2C_FLTR_DNF												0
#define I2C_FLTR_ANOFF												4


/*************************************************************************************************************
 * Bit position definition of USART/UART peripheral registers
 ************************************************************************************************************/
//USART_SR
#define USART_SR_PE													0
#define USART_SR_FE													1
#define USART_SR_NF													2
#define USART_SR_ORE												3
#define USART_SR_IDLE												4
#define USART_SR_RXNE												5
#define USART_SR_TC													6
#define USART_SR_TXE												7
#define USART_SR_LBD												8
#define USART_SR_CTS												9

//USART_BRR
#define USART_BRR_DIV_FRACTION										0
#define USART_BRR_DIV_MANTISSA										4

//USART_CR1
#define USART_CR1_SBK												0
#define USART_CR1_RWU												1
#define USART_CR1_RE												2
#define USART_CR1_TE												3
#define USART_CR1_IDLEIE											4
#define USART_CR1_RXNEIE											5
#define USART_CR1_TCIE												6
#define USART_CR1_TXEIE												7
#define USART_CR1_PEIE												8
#define USART_CR1_PS												9
#define USART_CR1_PCE												10
#define USART_CR1_WAKE												11
#define USART_CR1_M													12
#define USART_CR1_UE												13
#define USART_CR1_OVER8												15

//USART_CR2
#define USART_CR2_ADD												0
#define USART_CR2_LBDL												5
#define USART_CR2_LBDIE												6
#define USART_CR2_LBCL												8
#define USART_CR2_CPHA												9
#define USART_CR2_CPOL												10
#define USART_CR2_CLKEN												11
#define USART_CR2_STOP												12
#define USART_CR2_LINEN												14

//USART_CR3
#define USART_CR3_EIE												0
#define USART_CR3_IREN												1
#define USART_CR3_IRLP												2
#define USART_CR3_HDSEL												3
#define USART_CR3_NACK												4
#define USART_CR3_SCEN												5
#define USART_CR3_DMAR												6
#define USART_CR3_DMAT												7
#define USART_CR3_RTSE												8
#define USART_CR3_CTSE												9
#define USART_CR3_CTSIE												10
#define USART_CR3_ONEBIT											11

//USART_GTPR
#define USART_GTPR_PSC												0
#define USART_GTPR_GT												8



#include "stm32f407xx_GPIO_driver.h"
#include "stm32f407xx_SPI_driver.h"
#include "stm32f407xx_I2C_driver.h"
#include "stm32f407xx_USART_driver.h"


#endif /* INC_STM32F407XX_H_ */
