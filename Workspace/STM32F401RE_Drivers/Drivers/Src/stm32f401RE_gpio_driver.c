/*
 * stm32f401RE_gpio_driver.c
 *
 *  Created on: Mar 10, 2024
 *      Author: ayushganguly
 */

#include "stm32f401RE_gpio_driver.h"

/*
 * Init and De-init
 */

/*
 * initialize the registers of the GPIO registers
 * @param p_GPIOHandle handler which contains the base address of GPIO and info on what registers of GPIO to config
 */
void GPIO_Init(GPIO_Handle_t *p_GPIOHandle)
{
	// 1) Configure the mode of GPIO pin
	uint32_t temp = 0;

	if(p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		temp = p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		// reset what was there in that register at that position before
		p_GPIOHandle->p_GPIOx->MODER = p_GPIOHandle->p_GPIOx->MODER & ~(0x3 << (2*p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		//set
		p_GPIOHandle->p_GPIOx->MODER |= temp;
	}
	else
	{
		// Interrupt mode
		if(p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FE)
		{
			// 1. configure the FTSR register
			EXTI->FTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// reset the corresponding RTSR bit so that only FTSR is set
			EXTI->RTSR = EXTI->RTSR & ~(0x1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RE)
		{
			// 1. configure the RTSR register
			EXTI->RTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			// reset the corresponding FTSR bit so that only RTST is set
			EXTI->FTSR = EXTI->FTSR & ~(0x1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}
		else if(p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// 1. configure the FTSR and RTSR
			EXTI->FTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		// 2. Configure the GPIO port selection and SYSCFG_EXTICR (mux implementation, depending on which pin and port, we have 4 registers)
		// Enable the clock for SYSCFG register
		SYSCFG_PCLK_EN();
		uint8_t sysCfgReg = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t EXTIline = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;

		// reset the 4bits in the specific reg;
		SYSCFG->EXTICR[sysCfgReg] &= ~(0xF << (4*EXTIline));

		// set the 4 bits
		if (p_GPIOHandle->p_GPIOx == GPIOA)
		{
			SYSCFG->EXTICR[sysCfgReg] |= (0x0 << (4*EXTIline));
		}
		else if(p_GPIOHandle->p_GPIOx == GPIOB)
		{
			SYSCFG->EXTICR[sysCfgReg] |= (0x1 << (4*EXTIline));
		}
		else if(p_GPIOHandle->p_GPIOx == GPIOC)
		{
			SYSCFG->EXTICR[sysCfgReg] |= (0x2 << (4*EXTIline));
		}
		else if(p_GPIOHandle->p_GPIOx == GPIOD)
		{
			SYSCFG->EXTICR[sysCfgReg] |= (0x3 << (4*EXTIline));
		}
		else if(p_GPIOHandle->p_GPIOx == GPIOE)
		{
			SYSCFG->EXTICR[sysCfgReg] |= (0x4 << (4*EXTIline));
		}
		else if(p_GPIOHandle->p_GPIOx == GPIOH)
		{
			SYSCFG->EXTICR[sysCfgReg] |= (0x7 << (4*EXTIline));
		}

		// 3. Enable the exti interrupt delivery using EXTI_IMR (mask registor)
		EXTI->IMR |= 1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	temp = 0;
	//2. configure the speed
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	p_GPIOHandle->p_GPIOx->OSPEEDR &= ~( 0x3 << ( 2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	p_GPIOHandle->p_GPIOx->OSPEEDR |= temp;

	//3. configure the pupd settings
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	p_GPIOHandle->p_GPIOx->PUPDR &= ~( 0x3 << ( 2 * p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	p_GPIOHandle->p_GPIOx->PUPDR |= temp;


	//4. configure the optype
	temp = (p_GPIOHandle->GPIO_PinConfig.GPIO_OPType << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	p_GPIOHandle->p_GPIOx->OTYPER &= ~( 0x1 << p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	p_GPIOHandle->p_GPIOx->OTYPER |= temp;

	// 5) Configure the Alt functionality
	if(p_GPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
		{
			//configure the alt function registers.
			uint8_t temp1, temp2;

			temp1 = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
			temp2 = p_GPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
			p_GPIOHandle->p_GPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
			p_GPIOHandle->p_GPIOx->AFR[temp1] |= (p_GPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) );
		}

}

/*
 *  Reset all register contents of the GPIO peripheral
 * @param1: pGPIOx: Base address of the GPIO port to know which bit of the RCC reset
 * 				  bus registers to reset
 *
 */
void GPIO_DeInit(GPIO_Regdef_t *p_GPIOx)
{
	if (p_GPIOx == GPIOA)
	{
		GPIOA_PCLK_RESET();
	}else if (p_GPIOx == GPIOB)
	{
		GPIOB_PCLK_RESET();
	}else if (p_GPIOx == GPIOC)
	{
		GPIOC_PCLK_RESET();
	}else if (p_GPIOx == GPIOD)
	{
		GPIOD_PCLK_RESET();
	}else if (p_GPIOx == GPIOE)
	{
		GPIOE_PCLK_RESET();

	}else if (p_GPIOx == GPIOH)
	{
		GPIOH_PCLK_RESET();
	}
}

/*
 * GPIO peripheral clock enables or disables for a specific port
 * @param1 p_GPIOx: takes in the base address of the GPIO port peripheral to figure out which port's bit to be set to one in RCC clk bus enable reg
 * @param2: Enable/ Disable macros
 */
void GPIO_PeriClockControl(GPIO_Regdef_t *p_GPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(p_GPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if (p_GPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if (p_GPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if (p_GPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if (p_GPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();

		}else if (p_GPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}

	}
	else
	{
		if(p_GPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}else if (p_GPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}else if (p_GPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}else if (p_GPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}else if (p_GPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();

		}else if (p_GPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}

/*
 * Data read and write
 */

/*
 * @param1 p_GPIOx: Base address of the GPIO port
 * @param2: pin number
 * @return: pin state: 0 or 1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Regdef_t *p_GPIOx, uint8_t Pin_number)
{
	uint8_t value;
	// right shift the pin value you want to read from to the LSB and bit mask
	// to get just that bit you want to read
	value = (uint8_t) (p_GPIOx->IDR >> Pin_number) & 0x00000001;
	return value;
}

/*
 * @param1: GPIO base Address
 * @return: 16 bit data from the input port (because other 16 are reserved)
 */
uint16_t GPIO_ReadFromInputPort(GPIO_Regdef_t *p_GPIOx)
{
	// read all 16 bits of that register, this is specific to the GPIO port
	uint16_t value;
	value = (uint16_t) p_GPIOx->IDR;
	return value;
}

/*
 * @param1 p_GPIOx: Base address of the GPIO port
 * @param2: pin number
 * @param3: set or reset -> 00000001 or 00000000
 */
void GPIO_WriteToOutputPin(GPIO_Regdef_t *p_GPIOx, uint8_t Pin_number, uint8_t value)
{
	// reset but at that specific bit of register depending on pin no.
	p_GPIOx->ODR = p_GPIOx->ODR & ~(0x1 << Pin_number);
	// set
	p_GPIOx->ODR |= value << Pin_number;
}

/*
 * @param1 p_GPIOx: Base address of the GPIO port
 * @param2: value
 */
void GPIO_WriteToOutputPort(GPIO_Regdef_t *p_GPIOx, uint16_t value)
{
	// clear the lower 16bits keep the upper ones as is
	p_GPIOx->ODR = p_GPIOx->ODR & 0xFFFF0000;
	// this will set the lower 16bits to value because they got reset to 0, the upper 16 bits ones of
	// value are 0's so the reserved bits will not be touched
	p_GPIOx->ODR |= (uint32_t)value;

}

/*
 * @param1 p_GPIOx: Base address of the GPIO port
 * @param2: pin number
 */
void GPIO_ToggleOutputPin(GPIO_Regdef_t *p_GPIOx, uint8_t Pin_number)
{
	// Bitwise xor for toggling the pinnumber in the o/p registor
	p_GPIOx->ODR ^= (1 << Pin_number);
}

// IRQ Configuration and ISR handling - Processor specific and side

/*
 * @param1: IRQNumber
 * @param2: Enable or Disable
 */
// configure the NVIC registers of the ARM Cortex processor - set the IRQ number
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	// Enable Interrupts on IRQ numbers
	if (EnorDi == ENABLE )
	{
		if (IRQNumber <= 31)
		{
			// program ISER0 register
			// dereference for storing value
			*NVIC_ISER0 |= (1 << IRQNumber);

		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber - 32));

		}
		else if(IRQNumber > 65 && IRQNumber < 96)
		{
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber - 64));

		}

	}
	// Disable Interrupts on IRQ numbers
	else
	{
		if (IRQNumber <= 31)
		{
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber - 32));
		}
		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber - 64));
		}
	}
}

/*
 * Configure the IRQ Priority number for the interrupt number
 * @param1: IRQNumber
 * @param2: Priority of IRQ - possible values from @GPIO_PRIORITY_VALUES
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. findout the ipr reg
	uint8_t IRQPriorityReg = IRQNumber / 4;
	uint8_t IRQPriorityRegSection = IRQNumber % 4;

	// each section is 8 bits long in a register and only upper 4 bits in each section are implemented
	uint8_t shift_amount = (8*IRQPriorityRegSection) + (8 - NO_PRIORITY_BITS_IMPLEMENTED);

	// clear
	*(NVIC_IPR_BASE_ADDR + (IRQPriorityReg)) &= ~(0xF << (shift_amount));

	// adding two 32 bits which is 4 bytes, each addition is an addition of a byte
	*(NVIC_IPR_BASE_ADDR + (IRQPriorityReg)) |= (IRQPriority << (shift_amount));

}



/*
 * Whenever we have an interrupt trigger, we have this function to process the interrupt
 * @param1 Pin_number: to process which pin's interrupt we have to handle
 */
void GPIO_IRQHandling(uint8_t Pin_number)
{
	// clear the EXTI PR (pending request) register corresponding to the pin number
	// the register will be set when an interrupt is triggered, the main
	// handling is done on the application side, here we just need to clear it, to say interrupt handling is done

	if (EXTI->PR & (1 << Pin_number))
	{
		// Setting it to 1 clears the pending interrupt
		EXTI->PR |= (1 << Pin_number);
	}


}
