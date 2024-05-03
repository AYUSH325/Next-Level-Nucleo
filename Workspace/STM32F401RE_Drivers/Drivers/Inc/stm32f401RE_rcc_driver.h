/*
 * stm32f401RE_rcc_driver.h
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */

#ifndef INC_STM32F401RE_RCC_DRIVER_H_
#define INC_STM32F401RE_RCC_DRIVER_H_

#include "STM32F401RE.h"

#define APB1		0
#define APB2		1

/*
 * Returns the peripheral clock frequency
 */
uint32_t RCC_GetPCLKValue(uint8_t bus);



#endif /* INC_STM32F401RE_RCC_DRIVER_H_ */
