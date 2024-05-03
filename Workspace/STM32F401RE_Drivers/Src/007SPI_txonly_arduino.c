/*
 * 007SPI_txonly_arduino.c
 *
 *  Created on: Mar 31, 2024
 *      Author: ayushganguly
 */

#include <string.h>

#include "STM32F401RE.h"
#include "stm32f401RE_gpio_driver.h"
#include "stm32f401RE_spi_driver.h"
#include "stm32f401RE_gpio_driver.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2 MOSI
 * PB10 --> SCLK
 * PB09 --> NSS
 * ALT function mode: 05
 */


void delay(void)
{
	// software delay
	for(uint32_t i = 0; i < 500000; i++);
}



void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.p_GPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = ALT_FUN_MODE_5;
	SPIPins.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_PP; // for I2C we use OD as per specifications, but for SPI we use PP
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;

	//  First Enable GPIO peripheral clock
	GPIO_PeriClockControl(GPIOB, ENABLE);

	// SCLK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
	GPIO_Init(&SPIPins);

//	// MISO
//	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
//	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&SPIPins);

	// NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	GPIO_Init(&SPIPins);

}

void GPIO_ButtonInit(void)
{

	GPIO_Handle_t GPIOBtn;

	// BTN GPIO Config
	GPIOBtn.p_GPIOx = GPIOC;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD; // dont need to enable because schematic has external pull up resistor

	GPIO_PeriClockControl(GPIOC, ENABLE);
	GPIO_Init(&GPIOBtn);
}


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.p_SPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SCLKSpeed = SPI_SCLK_SPEED_DIV8; // generates sclk of 2MHz
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // hardware slave management for NSS Pin

	// first enable peripheral clock then initialize SPI
	SPI_PeriClockControl(SPI2, ENABLE);

	SPI_Init(&SPI2Handle);


}

// We transmit the hello world through MOSI line, no need to configure the MISO and NSS lines

int main(void)
{

	char user_data[] = "Hello world";
	SPI2_GPIOInits(); // this function is used to initialize the GPIO pins to behave as SPI2 pins

	GPIO_ButtonInit();

	SPI2_Inits();

	//makes NSS pulled low (active low) for hardware slave management when SSOE = 1
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		// do not transmit message until button press
		while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13));

		// to avoid button debouncing related issues 200ms of delay
		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		// Since slave will not know how many data bytes we are sending,
		// we should send the length of the data as well
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI3, &dataLen, 1);

		SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

		// wait until TX buffer is empty
		while(SPI_GetFlagStatus(SPI2, SPI_TXE_FLAG) == FLAG_RESET);

		// wait until BSY flag is 0, SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) != FLAG_RESET);

		// Disable SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);
	}

	return 0;
}


