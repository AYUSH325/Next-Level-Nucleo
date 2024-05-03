/*
 * 011I2C_MasterReceiveData.c
 *
 *  Created on: Apr 18, 2024
 *      Author: ayushganguly
 */

#include "STM32F401RE.h"
#include <string.h>
#include <stdio.h>

extern void initialise_monitor_handles();

#define SLAVE_ADDR		0x68
#define PRESSED			0


// Arduino I2C (slave)
// SDA - A4
// SCL - A5

void delay(void)
{
	// software delay
	for(uint32_t i = 0; i < 500000; i++);
}

// STM Master
// SCL - PB6
// SDA - PB9

I2C_Handle_t I2C1Handle;

// rcv buffer
uint8_t rcv_buf[32]; //32 bytes
uint8_t command_code;
uint8_t length;


void I2C_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;
	memset(&I2CPins, 0, sizeof(I2CPins));

	I2CPins.p_GPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = ALT_FUN_MODE_4;
	I2CPins.GPIO_PinConfig.GPIO_OPType = GPIO_OP_TYPE_OD; // for I2C we use OD as per specifications, but for SPI we use PP
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;

	//First Enable GPIO peripheral clock
	GPIO_PeriClockControl(GPIOB, ENABLE);

	//SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber =  GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	//SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber =  GPIO_PIN_NO_9;
	GPIO_Init(&I2CPins);

}

void I2C1_Inits(void)
{
	memset(&I2C1Handle, 0, sizeof(I2C1Handle));

	I2C1Handle.p_I2C = I2C1;
	I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
	// do not use reserved addresses from I2C docs -
	//also this address we assign the stm board in case we want it to act as slave
	I2C1Handle.I2C_Config.I2C_DeviceAddress = 0x61;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2; // not required since using standard mode


	// first enable peripheral clock then initialize I2C
	I2C_PeriClockControl(I2C1, ENABLE);
	I2C_Init(&I2C1Handle);

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
	// semihosting
	initialise_monitor_handles();
	printf("Application is running\n");

	// i2c pin inits
	I2C_GPIOInits();

	// i2c peripheral config
	I2C1_Inits();

	//Button GPIO
	GPIO_ButtonInit();

	// i2c peripheral enable
	I2C_PeripheralControl(I2C1, ENABLE);

	// From reference manual we can enable ack bit in CR register only after PE = 1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while (1)
	{
		// wait for button press
		while(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) != PRESSED);

		// to avoid button debouncing related issues 200ms of delay
		delay();

		// First write the command code to slave to ask for length info
		command_code = 0x51;
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		// Get the length info and store it in the length buffer
		I2C_MasterReceiveData(&I2C1Handle, &length, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		// send the second command to indicate slave to transmit data
		command_code = 0x52;
		I2C_MasterSendData(&I2C1Handle, &command_code, 1, SLAVE_ADDR, I2C_ENABLE_SR);

		// receive the data
		I2C_MasterReceiveData(&I2C1Handle, rcv_buf, length, SLAVE_ADDR, I2C_DISABLE_SR);

		rcv_buf[length+1] = '\0';
		printf("The data is %s", rcv_buf); //%s needs null char

	}

	return 0;
}
