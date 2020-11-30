/**
 ******************************************************************************
 * @file           : main.c
 * @author         : Sayan Rana
 * @brief          : Main program body
 ******************************************************************************
 * @IDE		   : Created by STM32 Cube IDE
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "stm32f407xx.h"
#include "stm32f407xx_GPIO_driver.h"

int main(void)
{
    /* Loop forever */
	for(;;);
}
