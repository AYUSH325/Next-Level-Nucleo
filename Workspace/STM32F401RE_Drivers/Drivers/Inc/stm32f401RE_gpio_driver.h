/*
 * stm32f401RE_gpio_driver.h
 *
 *  Created on: Mar 10, 2024
 *      Author: ayushganguly
 */

#ifndef INC_STM32F401RE_GPIO_DRIVER_H_
#define INC_STM32F401RE_GPIO_DRIVER_H_

#include "STM32F401RE.h"

/*
 * This is the Configuration structure for a GPIO pin
 */

// 1 byte is enough for storing the information for which pin number, output, resistor type, alt func mode etc.
typedef struct
{
	uint8_t GPIO_PinNumber;     // possible values from @GPIO_PIN_NUMBERS
	uint8_t GPIO_PinMode;		//possible values from @GPIO_PIN_MODES
	uint8_t GPIO_PinSpeed;		//possible values from @GPIO_PIN_SPEEDS
	uint8_t GPIO_PinPuPdControl; // GPIO pull up & pull down resistor config @GPIO_PIN_ResistorConfig
	uint8_t GPIO_OPType;		// possible values from @GPIO_PIN_TYPE
	uint8_t GPIO_PinAltFunMode; // possible values from @GPIO_ALT_TYPE
}GPIO_PinConfig_t;

/*
 * Handle Structure for a GPIO pin
 */

typedef struct
{
	// pointer to hold the base address of the GPIO peripheral
	GPIO_Regdef_t *p_GPIOx; // this holds the base address of the GPIO port (A, B, etc) to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig; // this holds GPIO pin configuration settings

}GPIO_Handle_t;

/*
 * @GPIO_PIN_MODES
 * GPIO pin possible modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
// configure GPIO pin to deliver an interrupt when falling/ rising edge is detected on the pin
#define GPIO_MODE_IT_FE		4 // interrupt mode when falling edge trigger
#define GPIO_MODE_IT_RE		5 // interrupt mode when rising edge trigger
#define GPIO_MODE_IT_RFT	6 // interrupt mode whenrising fallin edge trigger

/*
 * @GPIO_PIN_TYPE
 * GPIO pin possible output types
 */
#define GPIO_OP_TYPE_PP 	0 // push-pull
#define GPIO_OP_TYPE_OD 	1 // open-drain

/*
 * @GPIO_PIN_SPEEDS
 * GPIO pin possible output speeds
 */
#define GPIO_SPEED_LOW		0
#define GPIO_SPEED_MED_HIGH	1
#define GPIO_SPEED_HIGH		2
#define GPIO_SPEED_VHIGH	3

/*
 * @GPIO_PIN_ResistorConfig
 * GPIO pin pull up pull down
 */
#define GPIO_NO_PUPD		0
#define GPIO_PU				1
#define GPIO_PD				2

/*
 * @GPIO_PIN_NUMBERS
 * GPIO pin numbers
 */
#define GPIO_PIN_NO_0		0
#define GPIO_PIN_NO_1		1
#define GPIO_PIN_NO_2		2
#define GPIO_PIN_NO_3		3
#define GPIO_PIN_NO_4		4
#define GPIO_PIN_NO_5		5
#define GPIO_PIN_NO_6		6
#define GPIO_PIN_NO_7		7
#define GPIO_PIN_NO_8		8
#define GPIO_PIN_NO_9		9
#define GPIO_PIN_NO_10		10
#define GPIO_PIN_NO_11		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

/*
 * @GPIO_ALT_TYPE
 * ALT Func Mode
 */
#define ALT_FUN_MODE_0		0
#define ALT_FUN_MODE_1		1
#define ALT_FUN_MODE_2		2
#define ALT_FUN_MODE_3		3
#define ALT_FUN_MODE_4		4
#define ALT_FUN_MODE_5		5
#define ALT_FUN_MODE_6		6
#define ALT_FUN_MODE_7		7
#define ALT_FUN_MODE_8		8
#define ALT_FUN_MODE_9		9
#define ALT_FUN_MODE_10		10
#define ALT_FUN_MODE_11		11
#define ALT_FUN_MODE_12		12
#define ALT_FUN_MODE_13		13
#define ALT_FUN_MODE_14		14
#define ALT_FUN_MODE_15		15



/************************************************************************************
  	  	  	  	  	  Driver API Prototype Requirements

  1) GPIO Init
  2) Enable/Disable GPIO port clock
  3) Read from a GPIO pin
  4) Write to GPIO pin
  5) Configure Alternate Functionality
  6) Interrupt Handling
 ***********************************************************************************/

/*
 * Init and De-init
 */

void GPIO_Init(GPIO_Handle_t *p_GPIOHandle);
void GPIO_DeInit(GPIO_Regdef_t *p_GPIOx);

/*
 * GPIO peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_Regdef_t *p_GPIOx, uint8_t EnorDi);

/*
 * Data read and write
 */

uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *p_GPIOx, uint8_t Pin_number);
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *p_GPIOx);
void GPIO_WriteToOutputPin(GPIO_Regdef_t *p_GPIOx, uint8_t Pin_number, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_Regdef_t *p_GPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_Regdef_t *p_GPIOx, uint8_t Pin_number);

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t Pin_number);


#endif /* INC_STM32F401RE_GPIO_DRIVER_H_ */
