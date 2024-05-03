#include "STM32F401RE.h"
#include "stm32f401RE_gpio_driver.h"


void delay(void)
{
	// software delay
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLED;

	// Toggle PA5, so output with o/p type pushpull
	GpioLED.p_GPIOx = GPIOA;
	GpioLED.GPIO_PinConfig.GPIO_PinNumber = 5;
//	GpioLED.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP;
	GpioLED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GpioLED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
//	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need to enable because output type is already pushpull

	// Toggle LED PA5 with o/p type opendrain, since no push pull resistor chosen, LED should not toggle
	GpioLED.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_OD;
	// Solve the issue by activating the pull up resistor, pullup resistor is 40k
	// so light with very small intensity will he shown as current is less, need an external resistor
	GpioLED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	// Can use push-pull instead of openDrain for this reason

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLED);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
		delay();
	}


}
