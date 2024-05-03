/*
 * 003LED_button_ext.c
 *
 *  Created on: Mar 17, 2024
 *      Author: ayushganguly
 */


#include "STM32F401RE.h"
#include "stm32f401RE_gpio_driver.h"

#define PRESSED 0

void delay(void)
{
	// software delay
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLED, GPIOBtn;

	// Toggle PA5, so output with o/p type pushpull
	GpioLED.p_GPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GpioLED.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need to enable because output type is already pushpull


	// PB12 is external button
	GPIOBtn.p_GPIOx = GPIOB;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU; // using internal resistor the 22kohm from breadboard

	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&GPIOBtn);

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLED);


	while(1)
	{
		// active low, since connected to ground
		if (GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == PRESSED)
		{
			// achieve fake debouncing (reading multiple times in a short time) using delay
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		}
	}
}



