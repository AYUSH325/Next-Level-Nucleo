/*
 * 011I2C_MasterReceiveData.c
 *
 *  Created on: Apr 18, 2024
 *      Author: ayushganguly
 */

#include "STM32F401RE.h"
#include <string.h>
#include <stdio.h>


#define SLAVE_ADDR		0x68
#define MY_ADDR			SLAVE_ADDR
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
uint8_t tx_buf[32] = "STM32 Slave mode testing.."; //32 bytes - wire library of arduino limitation
uint8_t commandCode;
uint8_t length;
uint8_t cnt = 0;


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
	// do not use reserved addresses from I2C docs.
	//also this address we assign the stm board in case we want it to act as slave
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
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

	// i2c pin inits
	I2C_GPIOInits();

	// i2c peripheral config
	I2C1_Inits();

	//Button GPIO
	GPIO_ButtonInit();

	// i2c IRQ configurations - slave appliction is interrupt moce
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
	I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

	// need to enable the interrupt control bits so that interrupt can be triggered on processor
	I2C_SLaveEnableDisableCallbackEvents(I2C1, ENABLE);

	// I2C peripheral enable
	I2C_PeripheralControl(I2C1, ENABLE);

	// From reference manual we can enable ack bit in CR register only after PE = 1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1);

	return 0;
}


// interrupts from startup file which were weak defined are overriden
void I2C1_EV_IRQHandler(void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *p_I2CHandle, uint8_t AppEV)
{
	if ( AppEV == I2C_EV_DATA_REQ)
	{
		// Master wants some data, slave has to send it
		if (commandCode == 0x51)
		{
			// send the length information
			I2C_SlaveSendData(p_I2CHandle->p_I2C, strlen((char*)tx_buf));
		}
		else if (commandCode == 0x52)
		{
			I2C_SlaveSendData(p_I2CHandle->p_I2C, tx_buf[cnt]);
			cnt++;
		}
	}
	else if (AppEV == I2C_EV_DATA_RECV)
	{
		// Master is sending data, slave has to read it

		// Step 1: slave receives command code 0x51 for length & 0x52 for request for data
		commandCode = I2C_SlaveReceiveData(p_I2CHandle->p_I2C);

	}
	else if (AppEV == I2C_ER_AF)
	{
		// Master does not want anymore data so sending NACK - happens only when slace txing
		commandCode = 0xFF; //invalidate
		cnt = 0;
	}
	else if (AppEV == I2C_EV_STOP)
	{
		// Master sends closing condition when slave is receiving data
		// So master has ended I2C communication with slave
		cnt = 0;
	}
}













