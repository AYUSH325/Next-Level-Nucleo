/*
 * 005button_interrupt.c
 *
 *  Created on: Mar 18, 2024
 *      Author: ayushganguly
 */
#include <string.h>

#include "STM32F401RE.h"
#include "stm32f401RE_gpio_driver.h"


int main(void)
{
	GPIO_Handle_t GpioLED, GPIOBtn;

	// sets all the member variables of the struct to 0 so that we don't take in
	// garbage values if that variables was not set
	memset(&GpioLED, 0, sizeof(GpioLED));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	// Toggle PA5, so output with o/p type pushpull
	GpioLED.p_GPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLED.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need to enable because output type is already pushpull

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLED);

	GPIOBtn.p_GPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FE;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need to enable because schematic has external pull up resistor

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GPIOBtn);

	// IRQ configurations - only 1 interrupt so priority is optional
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI10_15, NVIC_IRQ_PRIO15);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI10_15, ENABLE);


	while(1);

}

// interrupt name is from startup file, it has a weak implementation of this, we are overriding this function
void EXTI15_10_IRQHandler(void)
{
	// debounce using timer interrupts, don't add delays here
	// call the api for interrupt handling - with button pin
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
}
