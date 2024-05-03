/*
 * 014USART_TX_Msg.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */
#include <string.h>
#include <stdio.h>

#include "STM32F401RE.h"

#define PRESSED			0

char txString[] = "UART_TX_TESTING\n\r";
USART_Handle_t USART1Handle;


void delay(void)
{
	// software delay
	for(uint32_t i = 0; i < 500000; i++);
}

void USART1_Init(void)
{
	USART_Handle_t USART1Handle;
	memset(&USART1Handle, 0, sizeof(USART1Handle));


	USART1Handle.p_USARTx = USART1;
	USART1Handle.USART_Config.USART_Baud = 115200;
	USART1Handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	USART1Handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
	USART1Handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	USART1Handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART1Handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_PeriClockControl(USART1, ENABLE);

	USART_Init(&USART1Handle);
}

// GPIO Pins to use
// PA9 - USART1_TX
// PA10 - USART1_RX
void USART1_GPIOInit(void)
{
	GPIO_Handle_t GPIO_USART1_handle;
	memset(&GPIO_USART1_handle, 0, sizeof(GPIO_USART1_handle));

	GPIO_USART1_handle.p_GPIOx = GPIOA;
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_PinAltFunMode = ALT_FUN_MODE_7;
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	// Init TX PIN
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
	GPIO_Init(&GPIO_USART1_handle);

	// Init RX PIN
	GPIO_USART1_handle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
	GPIO_Init(&GPIO_USART1_handle);

}

void GPIO_ButtonInit(void)
{

	GPIO_Handle_t GPIOBtn;
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));


	// BTN GPIO Config
	GPIOBtn.p_GPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need to enable because schematic has external pull up resistor

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);
}


int main(void)
{

	USART1_GPIOInit();

	USART1_Init();

	//Button GPIO
	GPIO_ButtonInit();

	USART_PeripheralControl(USART1, ENABLE);

	while(1)
	{
		// wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != PRESSED);

		// to avoid debouncing issues 200ms of delay
		delay();

		USART_SendData(&USART1Handle, (uint8_t*) txString, strlen(txString));
	}

	return 0;
}


