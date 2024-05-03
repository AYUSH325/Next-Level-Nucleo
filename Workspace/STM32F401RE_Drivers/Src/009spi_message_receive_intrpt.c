/*
 * 009spi_message_receive_intrpt.c
 *
 *  Created on: Apr 9, 2024
 *      Author: ayushganguly
 */


/*
 * This application receives and prints the user message received from the Arduino peripheral in SPI interrupt mode
 * User sends the message through Arduino IDE's serial monitor tool
 * Monitor the message received in the SWV itm data console
 */
/*
 * Note : Follow the instructions to test this code
 * 1. Download this code on to STM32 board , acts as Master
 * 2. Download Slave code (003SPISlaveUartReadOverSPI.ino) on to Arduino board (Slave)
 * 3. Reset both the boards
 * 4. Enable SWV ITM data console to see the message
 * 5. Open Arduino IDE serial monitor tool
 * 6. Type anything and send the message (Make sure that in the serial monitor tool line ending set to carriage return)
 */

#include <stdio.h>
#include <string.h>

#include "STM32F401RE.h"
#include "stm32f401RE_gpio_driver.h"
#include "stm32f401RE_spi_driver.h"

SPI_Handle_t SPI2Handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile uint8_t readByte;

volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2 MOSI
 * PB13 --> SCLK
 * PB12 --> NSS
 * ALT function mode: 05
 */

void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;
	memset(&SPIPins, 0, sizeof(SPIPins));

	SPIPins.p_GPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = ALT_FUN_MODE_5;
	SPIPins.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP; // for I2C we use OD as per specifications, but for SPI we use PP
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//  First Enable GPIO peripheral clock
	GPIO_PeriClockControl(GPIOB, ENABLE);

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIO_Init(&SPIPins);

	// MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&SPIPins);

}

void SPI2_Inits(void)
{
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

	SPI2Handle.p_SPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV32; // generates sclk of 0.5MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management for NSS Pin

	// first enable peripheral clock then initialize SPI
	SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Init(&SPI2Handle);

}

void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;
	memset(&spiIntPin, 0, sizeof(spiIntPin));

	// this is the led GPIO configuration
	spiIntPin.p_GPIOx = GPIOC;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FE;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	//  First Enable GPIO peripheral clock
	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&spiIntPin);

	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15);

}

int main(void)
{
	uint8_t dummy_byte = 0xff;

	Slave_GPIO_InterruptPinInit();

	SPI2_GPIOInits();

	SPI2_Inits();

	//makes NSS pulled low (active low) for hardware slave management when SSOE = 1 && SPE =1
	SPI_SSOEConfig(SPI2, ENABLE);

	// Configure SPI Interrupt
	SPI_IRQInterruptConfig(IRQ_NO_SPI2, ENABLE);

	while(1)
	{
		rcvStop = 0;

		// wait till data is available from slave to master
		while(!dataAvailable);

		// disable the interrupt so that we are not being interrupt during SPI comms
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		while(!rcvStop)
		{
			// fetch the data from the SPI peripheral byte by byte in interrupt mode
			// send the data first to start the clock, and only after 1 byte has transmitted read it
			while (SPI_SendDataIT(&SPI2Handle, &dummy_byte, 1) == SPI_BUSY_IN_TX);
			while (SPI_ReceiveDataIT(&SPI2Handle, &readByte, 1) == SPI_BUSY_IN_RX);
		}

		// wait until TX buffer is empty
		while(SPI_GetFlagStatus(SPI2, SPI_TXE_FLAG) == FLAG_RESET);

		// wait until BSY flag is 0, SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) != FLAG_RESET);

		//Disable the SPI2 peripheral
		SPI_PeripheralControl(SPI2,DISABLE);

		printf("Rcvd data = %s\n",RcvBuff);

		dataAvailable = 0; //enable the flag when data available

		// reenable the interrupt for next set of data
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);

	}

	return 0;
}


// Runs when a data byte is received from the SPI peripheral
void SPI_IRQHandler(void)
{
	SPI_IRQHandling(&SPI2Handle);
}

// Slave data available interrupt handler
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_8);
	dataAvailable = 1;
}

void SPI_ApplicationEventCallback(SPI_Handle_t *p_SPIHandle, uint8_t AppEV)
{
	static uint32_t i = 0;

	// In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1)
	if (AppEV == SPI_EVENT_RX_CMPLT)
	{
		RcvBuff[i++] = readByte;
		if (readByte == '\0' || i == MAX_LEN)
		{
			rcvStop = 1;
			RcvBuff[i-1] = '\0';
			i = 0;
		}
	}
}


























