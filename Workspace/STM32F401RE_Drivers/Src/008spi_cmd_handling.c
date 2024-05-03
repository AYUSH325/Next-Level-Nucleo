/*
 * 008spi_cmd_handling.c
 *
 *  Created on: Apr 6, 2024
 *      Author: ayushganguly
 */

#include <string.h>
#include <stdio.h>

#include "STM32F401RE.h"
#include "stm32f401RE_gpio_driver.h"
#include "stm32f401RE_spi_driver.h"

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2 MOSI
 * PB13 --> SCLK
 * PB12 --> NSS
 * ALT function mode: 05
 */

// For semihosting printf purposes
extern void initialise_monitor_handles();

#define COMMAND_LED_CTRL		0x50
#define COMMAND_SENSOR_READ		0x51
#define COMMAND_LED_READ		0x52
#define COMMAND_PRINT			0x53
#define COMMAND_ID_READ			0x54

#define LED_ON					1
#define LED_OFF					0

#define PRESSED					0

// arduino analog pins
#define ANALOG_PIN0				0
#define ANALOG_PIN1				1
#define ANALOG_PIN2				2
#define ANALOG_PIN3				3
#define ANALOG_PIN4				4

// arduino LED (connected LED to pin 9 of arduino)

#define LED_PIN					9


void delay(void)
{
	// software delay
	for(uint32_t i = 0; i < 500000; i++);
}



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


void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;
	memset(&SPI2Handle, 0, sizeof(SPI2Handle));

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


uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if (ackbyte == 0xF5)
	{
		// ack
		return 1;
	}
	return 0;
}

// We transmit the hello world through MOSI line, no need to configure the MISO and NSS lines

int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

	initialise_monitor_handles();

	printf("application is running\n");

	SPI2_Inits();

	SPI2_GPIOInits(); // this function is used to initialize the GPIO pins to use the SPI2 peripheral

	GPIO_ButtonInit();

	printf("SPI init done\n");


	//makes NSS pulled low (active low) for hardware slave management when SSOE = 1
	SPI_SSOEConfig(SPI2, ENABLE);

	while(1)
	{
		// do not transmit message until button press i.e enable SPI peripheral only when
		// btn pressed
		// active low
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != PRESSED);

		printf("Button is pressed\n");

		// to avoid button debouncing related issues 200ms of delay
		delay();

		// enable the SPI2 peripheral
		SPI_PeripheralControl(SPI2, ENABLE);

		//1. CMD_LED_CTRL <pin no(1)> <value(1)>
		uint8_t commandcode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &commandcode, 1);

		// the transmission of 1 byte resulted 1 garbage byte collection in RX buffer of the master and RXNE flag is set
		// so do the dummy read to clear the RXNE flag
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		// send some dummy bits (1 byte - since 8 bits data transfer) to fetch the response from the slave
		// as when you write to the slave's shift register from master, the current 8 bits in the slave's shift register gets transferred to the master
		// shift register through MOSI 1 bit at a time
		SPI_SendData(SPI2, &dummy_write, 1);

		// now we can read the ack/nack from master's shift register since it got transeferred to master when writing the dummy
		// data to slave
		uint8_t ackbyte;
		SPI_ReceiveData(SPI2, &ackbyte, 1);
		uint8_t args[2];

		if (SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			// Send data
			SPI_SendData(SPI2, args, 2);

			printf("Command LED ctrl executed\n");

		}


		// 2. CMD_SENSOR_READ  <analog pin number(1)>

		// do not transmit message until button press i.e enable SPI peripheral only when
		// btn pressed
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != PRESSED);

		// to avoid button debouncing related issues 200ms of delay
		delay();


		commandcode = COMMAND_SENSOR_READ;

		SPI_SendData(SPI2, &commandcode, 1);

		SPI_ReceiveData(SPI2, &dummy_read, 1);

		SPI_SendData(SPI2, &dummy_write, 1);

		SPI_ReceiveData(SPI2, &ackbyte, 1);
		uint8_t args2;

		if (SPI_VerifyResponse(ackbyte))
		{
			// send arguments
			args2 = ANALOG_PIN0;

			// Send data
			SPI_SendData(SPI2, &args2, 1);

			// read the value of sensor connected to analog pin of arduino

			// since we send data to slave, we also will receive 1B in master
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			// insert some delay for slave to be ready with data from ADC conversion
			// 0 - OV
			// 255 - 5V
			delay();

			// To receive the sensor value result in the anlog pin, send 1 Byte of data to slave
			SPI_SendData(SPI2, &dummy_write, 1);

			// receive the analog sensor result
			uint8_t analog_read;
			SPI_ReceiveData(SPI2, &analog_read, 1);
			printf("Command Sensor Read %d\n", analog_read);
		}


		// wait until TX buffer is empty
		while(SPI_GetFlagStatus(SPI2, SPI_TXE_FLAG) == FLAG_RESET);

		// wait until BSY flag is 0, SPI is not busy
		while (SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) != FLAG_RESET);

		// Disable SPI2 Peripheral
		SPI_PeripheralControl(SPI2, DISABLE);

		printf("SPI communication closed\n");
	}

	return 0;
}




