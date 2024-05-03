/*
 * stm32F401RE_i2c_driver.c
 *
 *  Created on: Apr 13, 2024
 *      Author: ayushganguly
 */

// remember to document today at 9pm
#include "STM32F401RE.h"
#include <stddef.h>


static void I2C_GenerateStartCondition(I2C_Regdef_t *p_I2C);
static void I2C_ExecuteAddressPhase(I2C_Regdef_t *p_I2C, uint8_t slaveAddr,
		uint8_t r_w);
static void I2C_ClearADDrFlag(I2C_Handle_t *p_I2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *p_I2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *p_I2CHandle);

static void I2C_GenerateStartCondition(I2C_Regdef_t *p_I2C)
{
	p_I2C->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ExecuteAddressPhase(I2C_Regdef_t *p_I2C, uint8_t slaveAddr,
		uint8_t r_w) {
	slaveAddr = slaveAddr << 1;
	// add the read/write bit
	slaveAddr |= r_w;
	// dont't need to check TXE bit because it is not set during address phase
	p_I2C->DR |= slaveAddr & 0xFF;
}

static void I2C_ClearADDrFlag(I2C_Handle_t *p_I2CHandle)
{
	// check for device mode
	uint32_t dummy_read;
	if (p_I2CHandle->p_I2C->SR2 & (1 << I2C_SR2_MSL))
	{
		// master mode
		if (p_I2CHandle->txRxState == I2C_BUSY_IN_RX)
		{
			// I2C is being used for RX
			if (p_I2CHandle->rxSize == 1)
			{
				// disable Acking - case 1
				I2C_ManageAcking(p_I2CHandle->p_I2C, I2C_ACK_DISABLE);
			}

			// clear the ADDR flag (read SR1, read SR2)
			dummy_read = p_I2CHandle->p_I2C->SR1;
			dummy_read = p_I2CHandle->p_I2C->SR2;
			(void)dummy_read;

		}
		else
		{
			// clear the ADDR flag (read SR1, read SR2) - used for blocking API cases
			dummy_read = p_I2CHandle->p_I2C->SR1;
			dummy_read = p_I2CHandle->p_I2C->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		// slave mode
		dummy_read = p_I2CHandle->p_I2C->SR1;
		dummy_read = p_I2CHandle->p_I2C->SR2;
		(void)dummy_read;
	}
}

void I2C_GenerateStopCondition(I2C_Regdef_t *p_I2C)
{
	p_I2C->CR1 |= (1 << I2C_CR1_STOP);
}

/*
 * I2C Peripheral clock enable
 * @param1 p_I2Cx: takes in the base address of the I2C port peripheral to figure out which port's bit to be set to one in RCC clk bus enable reg
 * @param2: Enable/ Disable macros
 */
void I2C_PeriClockControl(I2C_Regdef_t *p_I2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE) {
		if (p_I2Cx == I2C1) {
			I2C1_PLCK_EN();
		} else if (p_I2Cx == I2C2) {
			I2C2_PLCK_EN();
		} else if (p_I2Cx == I2C3) {
			I2C3_PLCK_EN();
		}
	} else {
		if (p_I2Cx == I2C1) {
			I2C1_PLCK_DI();
		} else if (p_I2Cx == I2C2) {
			I2C2_PLCK_DI();
		} else if (p_I2Cx == I2C3) {
			I2C3_PLCK_DI();
		}
	}
}

void I2C_PeripheralControl(I2C_Regdef_t *p_I2Cx, uint8_t EnorDi) {
	if (EnorDi == ENABLE) {
		p_I2Cx->CR1 |= (1 << I2C_CR1_PE);
	} else {
		p_I2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/*
 * Init
 * @param: the I2C handle structure
 */

void I2C_Init(I2C_Handle_t *p_I2CHandle) {
	uint32_t tempReg = 0;

	// ack control bit
	tempReg |= p_I2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;
	p_I2CHandle->p_I2C->CR1 |= (tempReg & 0x00000400);
	tempReg = 0;

	// configure the FREQ field of CR2 as I2C uses this to decide freq of APB1
	tempReg |= RCC_GetPCLKValue(APB1) / 1000000U;
	p_I2CHandle->p_I2C->CR2 |= (tempReg & 0x003F);
	tempReg = 0;

	//program the device own address - applicable when peripheral is slave to assign it an address
	tempReg |= p_I2CHandle->I2C_Config.I2C_DeviceAddress << 1; //bit14 is dont care
	tempReg |= 1 << 14; // have to keep reserved bit 14 to 1
	p_I2CHandle->p_I2C->OAR1 |= tempReg;

	// CCR calculations for SClK speeds
	uint16_t ccr_value = 0;
	tempReg = 0;
	if (p_I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode -> no need to set the mode bit of CCR reg as it is default SM
		ccr_value = RCC_GetPCLKValue(APB1)
				/ (2 * p_I2CHandle->I2C_Config.I2C_SCLSpeed);
	} else {
		// Fast mode

		// 1) set mode bit
		p_I2CHandle->p_I2C->CCR |= 1 << I2C_CR_FS;

		// 2) Set the duty cycle bit for FM
		p_I2CHandle->p_I2C->CCR |= p_I2CHandle->I2C_Config.I2C_FMDutyCycle
				<< I2C_CCR_DUTY;

		// 3) Calculate CCR based on duty cycle
		if (p_I2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2) {
			// Tlow = 2*Thigh
			ccr_value = RCC_GetPCLKValue(APB1)
					/ (3 * p_I2CHandle->I2C_Config.I2C_SCLSpeed);
		} else {
			// Tlow = 1.8Thigh
			ccr_value = RCC_GetPCLKValue(APB1)
					/ (25 * p_I2CHandle->I2C_Config.I2C_SCLSpeed);
		}
	}
	tempReg |= ccr_value & 0xFFF; //mask all the rest of bits in CCR value to 0 except last 12 bits;
	p_I2CHandle->p_I2C->CCR |= tempReg;

	uint8_t trise;
	// Configure TRISE -> (fPclk1/frise(max)) + 1
	if (p_I2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM) {
		// Standard mode
		trise = (RCC_GetPCLKValue(APB1) / I2C_MAX_FRISE_SM) + 1;
	} else {
		// Fast mode
		trise = (RCC_GetPCLKValue(APB1) / I2C_MAX_FRISE_FM) + 1;
	}
	p_I2CHandle->p_I2C->TRISE |= trise & 0x3F;

}

void I2C_DeInit(I2C_Regdef_t *p_I2Cx);

uint8_t I2C_GetFlagStatus(I2C_Regdef_t *p_I2Cx, uint32_t FlagName) {
	if (p_I2Cx->SR1 & FlagName) {
		return FLAG_SET;
	}
	return FLAG_RESET;
}

void I2C_ManageAcking(I2C_Regdef_t	*p_I2Cx, uint8_t EnorDi)
{
	if (EnorDi == I2C_ACK_ENABLE)
	{
		p_I2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
	else
	{
		p_I2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}
}

/*
 * I2C blocking Master send API
 */

void I2C_MasterSendData(I2C_Handle_t *p_I2CHandle, uint8_t *p_TxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t Sr)
{
	//1. Generate the Start condition
	I2C_GenerateStartCondition(p_I2CHandle->p_I2C);

	//2. confirm that start generation is completed by checking the SB flag in SR1
	//	NOTE: Until SB is cleared SCL will be pulled to LOW (EV5), so clear SB or EV5

	while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to w (0) (total 8 bits)
	I2C_ExecuteAddressPhase(p_I2CHandle->p_I2C, slaveAddr, WRITE); // this will clear EV5

	//4. Confirm the address phase is completed by checking the ADDR flag in the SR1 (EV6)
	while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_ADDR_FLAG)); // stuck here

	//5. Clear EV6/ ADDR flag according to its software sequence
	// Note: Until ADDR is cleared SCL will be stretched - will get timeout err(pulled to LOW)
	I2C_ClearADDrFlag(p_I2CHandle);

	//6. send the data until len becomes 0
	while (len > 0) {
		while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_TXE_FLAG));
		p_I2CHandle->p_I2C->DR = *p_TxBuffer; // clears EV8
		p_TxBuffer++;
		len--;
	}

	//7. When len becomes zero wait for TXE=1 and BTF=1 before generating the STOP condition
	// 	Note: TXE=1 , BTF=1 means both the Shift register & data register are empty and next transmission should begin
	// 	when BTF=1 SCL will be stretched (pulled to LOW)

	while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_TXE_FLAG));

	while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_BTF_FLAG));

	//8. Generate STOP condition and master need not to wait for the completion of stop condtion
	// Note: Generating STOP, automatically clears the BTF

	if (Sr == I2C_DISABLE_SR)
	{
		I2C_GenerateStopCondition(p_I2CHandle->p_I2C);
	}


}

/*
 * I2C blocking Master receive API
 */

void I2C_MasterReceiveData(I2C_Handle_t *p_I2CHandle, uint8_t *p_RxBuffer,
		uint32_t len, uint8_t slaveAddr, uint8_t Sr) {

	//1. Generate the START condition
	I2C_GenerateStartCondition(p_I2CHandle->p_I2C);

	//2. confirm that start generation is completed by checking the SB flag in the SR1
	//   Note: Until SB is cleared SCL will be stretched (pulled to LOW)
	while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_SB_FLAG));

	//3. Send the address of the slave with r/nw bit set to R(1) (total 8 bits )
	I2C_ExecuteAddressPhase(p_I2CHandle->p_I2C, slaveAddr, READ); // this will clear EV5

	//4. wait until address phase is completed by checking the ADDR flag in the SR1
	while (!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_ADDR_FLAG));

	//procedure to read only 1 byte from slave
	if (len == 1)
	{

		//First Disable Acking
		I2C_ManageAcking(p_I2CHandle->p_I2C, I2C_ACK_DISABLE);

		//clear the ADDR flag
		I2C_ClearADDrFlag(p_I2CHandle);

		//wait until RXNE becomes 1
		while(!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_SR1_RXNE));

		//generate STOP condition
		if (Sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(p_I2CHandle->p_I2C);
		}

		//read data in to buffer
		*p_RxBuffer = p_I2CHandle->p_I2C->DR;

	}

	//procedure to read data from slave when len > 1
	if (len > 1)
	{
		//clear the ADDR flag
		I2C_ClearADDrFlag(p_I2CHandle);

		//read the data until len becomes zero
		for (uint32_t i = len; i > 0; i--) {
			// wait until RXNE becomes 1
			while(!I2C_GetFlagStatus(p_I2CHandle->p_I2C, I2C_SR1_RXNE));

			if (i == 2) //if last 2 bytes are remaining
			{
				//Disable Acking
				I2C_ManageAcking(p_I2CHandle->p_I2C, I2C_ACK_DISABLE);

				//generate STOP condition
				if (Sr == I2C_DISABLE_SR)
				{
					I2C_GenerateStopCondition(p_I2CHandle->p_I2C);
				}

			}

			//read the data from data register in to buffer
			*p_RxBuffer = p_I2CHandle->p_I2C->DR;

			//increment the buffer address
			p_RxBuffer++;

		}

	}
	//re-enable ACKing
	if (p_I2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(p_I2CHandle->p_I2C, I2C_ACK_ENABLE);
	}

}

/*
 * Interrupt based Master Send API - Fill in the I2CHandle struct with the application info and enable the control bits
 * so processor is able to get interrupted by the event. Also enable start condition
 * @param1: I2C handle struct
 * @param2: Buffer containing data to be transmitted
 * @param3: length of transmitted data
 * @param4: repeated start condition required or not
 * @return: state the I2C is in
 */
uint8_t I2C_MasterSendData_IT(I2C_Handle_t *p_I2CHandle, uint8_t *p_TxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr)
{

	uint8_t busystate = p_I2CHandle->txRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		p_I2CHandle->p_TxBuffer = p_TxBuffer;
		p_I2CHandle->txLen = len;
		p_I2CHandle->txRxState = I2C_BUSY_IN_TX;
		p_I2CHandle->slaveAddress = slaveAddr;
		p_I2CHandle->sr = sr;

		//Generate START Condition
		I2C_GenerateStartCondition(p_I2CHandle->p_I2C); // will trigger start bit interrupt when control bit enabled


		//enable ITBUFEN Control Bit
		p_I2CHandle->p_I2C->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//enable ITEVFEN Control Bit
		p_I2CHandle->p_I2C->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		p_I2CHandle->p_I2C->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*
 * Interrupt based Master Receive API - Fill in the I2CHandle struct with the application info and enable the control bits
 * so processor is able to get interrupted by the event. Also enable start condition
 * @param1: I2C handle struct
 * @param2: Buffer which will contain data once received
 * @param3: length of received data
 * @param4: repeated start condition required or not
 * @return: state the I2C is in
 */

uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *p_I2CHandle, uint8_t *p_RxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr)
{
	uint8_t busystate = p_I2CHandle->txRxState;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		p_I2CHandle->p_RxBuffer = p_RxBuffer;
		p_I2CHandle->rxLen = len;
		p_I2CHandle->txRxState = I2C_BUSY_IN_RX;
		p_I2CHandle->rxSize = len; //rxSize is used in the ISR manage the data reception
		p_I2CHandle->slaveAddress = slaveAddr;
		p_I2CHandle->sr = sr;

		//Generate START Condition
		I2C_GenerateStartCondition(p_I2CHandle->p_I2C);

		//enable ITBUFEN Control Bit
		p_I2CHandle->p_I2C->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//enable ITEVFEN Control Bit
		p_I2CHandle->p_I2C->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		p_I2CHandle->p_I2C->CR2 |= (1 << I2C_CR2_ITERREN);
	}

	return busystate;
}

/*
 * Configure the NVIC registers of the ARM Cortex processor - set the IRQ number
 * @param1: IRQNumber
 * @param2: Enable or Disable
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi) {
	// Enable Interrupts on IRQ numbers
	if (EnorDi == ENABLE) {
		if (IRQNumber <= 31) {
			// program ISER0 register
			// dereference for storing value
			*NVIC_ISER0 |= (1 << IRQNumber);

		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNumber - 32));

		} else if (IRQNumber > 65 && IRQNumber < 96) {
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNumber - 64));

		}

	}
	// Disable Interrupts on IRQ numbers
	else {
		if (IRQNumber <= 31) {
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNumber);
		} else if (IRQNumber > 31 && IRQNumber < 64) {
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNumber - 32));
		} else if (IRQNumber >= 64 && IRQNumber < 96) {
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber - 64));
		}
	}
}

/*
 * Configure the IRQ Priority number of the interrupt
 * @param1: IRQNumber
 * @param2: Priority of IRQ - possible values from @IRQ_PRIORITY_VALUES
 */

void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority) {
	//1. findout the ipr reg
	uint8_t IRQPriorityReg = IRQNumber / 4;
	uint8_t IRQPriorityRegSection = IRQNumber % 4;

	// each section is 8 bits long in a register and only upper 4 bits in each section are implemented
	uint8_t shift_amount = (8 * IRQPriorityRegSection)
			+ (8 - NO_PRIORITY_BITS_IMPLEMENTED);

	// clear
	*(NVIC_IPR_BASE_ADDR + (IRQPriorityReg)) &= ~(0xF << (shift_amount));

	// adding two 32 bits which is 4 bytes, each addition is an addition of a byte
	*(NVIC_IPR_BASE_ADDR + (IRQPriorityReg)) |= (IRQPriority << (shift_amount));
}

/*
 * When master requests for data i.e a read operation is ongoing
 * @param1: the address of the I2C peripheral
 * @param2: the data which we need to send to master
 */
void I2C_SlaveSendData(I2C_Regdef_t *p_I2Cx, uint8_t data)
{
	p_I2Cx->DR = data;
}

/*
 * when master sends data i.e a write operation is ongoing
 * @param1: the address to the I2C peripheral
 * @return: a byte of the data which the master has sent
 */
uint8_t I2C_SlaveReceiveData(I2C_Regdef_t *p_I2Cx)
{
	uint8_t data;
	data = p_I2Cx->DR;
	return data;
}

/*
 * Enable or Disable the interrupt control bits in CR2
 * @param1: I2C peripheral address
 * @param2: Enable or Disable Macro
 */
void I2C_SLaveEnableDisableCallbackEvents(I2C_Regdef_t	*p_I2Cx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		//enable ITBUFEN Control Bit
		p_I2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//enable ITEVFEN Control Bit
		p_I2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//enable ITERREN Control Bit
		p_I2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	else
	{
		// Disable all interrupt control bits
		p_I2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
		p_I2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
		p_I2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
	}

}

/*
 * I2C Interrupts can occur due to a number of events from status register, handle them appropriately
 * Decode the event interrupts here
 */
void I2C_EV_IRQHandling(I2C_Handle_t *p_I2CHandle)
{
	uint32_t temp1, temp2, temp3;
	// Interrupt handling for both master and slave mode of a device
	temp1 = p_I2CHandle->p_I2C->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = p_I2CHandle->p_I2C->CR2 & (1 << I2C_CR2_ITBUFEN);

	temp3 = p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_SB);

	//1. Handle for interrupt generated by SB event
	// Note: start flag is applicable for only master mode. In slave mode the bit is always 0
	if (temp1 && temp3)
	{
		// SB flag is set - SB event has occurred
		// Have to execute address phase next
		if (p_I2CHandle->txRxState == I2C_BUSY_IN_TX)
		{
			I2C_ExecuteAddressPhase(p_I2CHandle->p_I2C, p_I2CHandle->slaveAddress, WRITE);
		}
		else if(p_I2CHandle->txRxState == I2C_BUSY_IN_RX)
		{
			I2C_ExecuteAddressPhase(p_I2CHandle->p_I2C, p_I2CHandle->slaveAddress, READ);
		}

	}

	//2. Handle for interrupt generated by ADDR event
	temp3 = p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_ADDR);
	if (temp1 && temp3)
	{
		// ADDR flag is set - ADDR event has occurred
		// clear the ADDR flag
		I2C_ClearADDrFlag(p_I2CHandle);
	}

	//3. Handle for interrupt generated by BTF event
	temp3 = p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_BTF);
	if (temp1 && temp3)
	{
		// BTF flag is set - BTF (Byte transfer finished) event has occurred
		if (p_I2CHandle->txRxState == I2C_BUSY_IN_TX)
		{
			// both data register & shift register are empty, so the last data byte has been transmitted
			// make sure that TXE is also set
			if (p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_TXE))
			{
				//make sure first that no more bytes left to transfer
				if (p_I2CHandle->txLen == 0)
				{
					// BTF & TXE are set so close transmission

					// Generate the STOP condition
					if (p_I2CHandle->sr == I2C_DISABLE_SR)
					{
						I2C_GenerateStopCondition(p_I2CHandle->p_I2C);
					}

					// reset all the member elements of the handle structure
					I2C_CloseSendData(p_I2CHandle);

					// notify the application of transmission complete
					I2C_ApplicationEventCallback(p_I2CHandle, I2C_EV_TX_CMPLT);
				}
			}
		}
		else if(p_I2CHandle->txRxState == I2C_BUSY_IN_RX)
		{
			// both data register & shift register are full
		}
	}

	//4. Handle for interrupt generated by STOPF event
	// Note: Stop detection flag is only applicable for slave mode. For master this flag will not be set
	temp3 = p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_STOPF);
	if (temp1 && temp3)
	{
		// STOPF flag is set - STOPF event has occurred
		// clear the STOPF flag -> read SR1 & write to CR1 (we have already read the SR1 in temp3)

		p_I2CHandle->p_I2C->CR1 |= 0x0000;

		// Notify the application that STOP is detected
		I2C_ApplicationEventCallback(p_I2CHandle, I2C_EV_STOP);
	}

	//5. Handle for interrupt generated by TXE event
	temp3 = p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_TXE);
	if (temp1 && temp2 && temp3)
	{
		// TXE flag is set - TXE event has occurred

		// Handle the event in the following way only if device is in master mode
		if (p_I2CHandle->p_I2C->SR2 & (1 << I2C_SR2_MSL))
		{
			// the data register is empty so software has to put a byte in DR for transmission
			I2C_MasterHandleTXEInterrupt(p_I2CHandle);
		}
		else
		{
			// Slave mode - Request for data (READ OP)
			if (p_I2CHandle->p_I2C->SR2 & (1 << I2C_SR2_TRA))
			{
				// make sure slave is in transmitter mode

				I2C_ApplicationEventCallback(p_I2CHandle, I2C_EV_DATA_REQ);
			}

		}
	}

	//6. Handle for interrupt generated by RXNE event
	temp3 = p_I2CHandle->p_I2C->SR1 & (1 << I2C_SR1_RXNE);
	if (temp1 && temp2 && temp3)
	{
		if (p_I2CHandle->p_I2C->SR2 & (1 << I2C_SR2_MSL))
		{
			// Master Mode

			// RXNE flag is set - RXNE event has occurred
			if (p_I2CHandle->txRxState == I2C_BUSY_IN_RX)
			{
				I2C_MasterHandleRXNEInterrupt(p_I2CHandle);
			}
		}
		else
		{
			// slave mode in receive data  (WRITE OP)
			if (!(p_I2CHandle->p_I2C->SR2 & (1 << I2C_SR2_TRA)))
			{
				// make sure slave is in receiver mode

				I2C_ApplicationEventCallback(p_I2CHandle, I2C_EV_DATA_RECV);
			}
		}
	}
}

static void I2C_MasterHandleTXEInterrupt(I2C_Handle_t *p_I2CHandle)
{
	if (p_I2CHandle->txRxState == I2C_BUSY_IN_TX)
	{
		if (p_I2CHandle->txLen > 0)
		{
			//1. Load the data to DR
			p_I2CHandle->p_I2C->DR = *(p_I2CHandle->p_TxBuffer);

			//2. decrement the txLen
			p_I2CHandle->txLen--;

			//3. Increment the buffer address
			p_I2CHandle->p_TxBuffer++;
		}
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2C_Handle_t *p_I2CHandle)
{
	// do data reception
	if (p_I2CHandle->rxSize == 1)
	{
		// sending NACK and clearing of addr event is done before
		*(p_I2CHandle->p_RxBuffer) = p_I2CHandle->p_I2C->DR;
		p_I2CHandle->rxLen--;
	}

	if (p_I2CHandle->rxSize > 1)
	{
		if (p_I2CHandle->rxLen == 2)
		{
			// disable the ACK
			I2C_ManageAcking(p_I2CHandle->p_I2C, I2C_ACK_DISABLE);
		}
		*(p_I2CHandle->p_RxBuffer) = p_I2CHandle->p_I2C->DR;
		p_I2CHandle->rxLen--;
		p_I2CHandle->p_RxBuffer++;
	}

	if (p_I2CHandle->rxLen == 0)
	{
		// close the I2C reception & notify application

		// Generate Stop Condition
		if (p_I2CHandle->sr == I2C_DISABLE_SR)
		{
			I2C_GenerateStopCondition(p_I2CHandle->p_I2C);
		}

		// Close the I2C RX -> re-enable acking here
		I2C_CloseReceiveData(p_I2CHandle);

		// Notify the application
		I2C_ApplicationEventCallback(p_I2CHandle, I2C_EV_RX_CMPLT);

	}
}

void I2C_CloseSendData(I2C_Handle_t *p_I2CHandle)
{
	// Disable all interrupts by resetting control bits
	p_I2CHandle->p_I2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	p_I2CHandle->p_I2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	p_I2CHandle->p_TxBuffer = NULL;
	p_I2CHandle->txRxState = I2C_READY;
	p_I2CHandle->txLen = 0;
}

void I2C_CloseReceiveData(I2C_Handle_t *p_I2CHandle)
{
	// Disable all interrupts by resetting control bits
	p_I2CHandle->p_I2C->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	p_I2CHandle->p_I2C->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

	p_I2CHandle->p_RxBuffer = NULL;
	p_I2CHandle->txRxState = I2C_READY;
	p_I2CHandle->rxLen = 0;
	p_I2CHandle->rxSize = 0;

	//re-enable ACKing
	if (p_I2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		I2C_ManageAcking(p_I2CHandle->p_I2C, I2C_ACK_ENABLE);
	}

}


void I2C_ER_IRQHandling(I2C_Handle_t *p_I2CHandle)
{
	uint32_t temp1,temp2;

	//Know the status of  ITERREN control bit in the CR2
	temp2 = (p_I2CHandle->p_I2C->CR2) & ( 1 << I2C_CR2_ITERREN);


	/***********************Check for Bus error************************************/
	temp1 = (p_I2CHandle->p_I2C->SR1) & ( 1<< I2C_SR1_BERR);
	if(temp1  && temp2 )
	{
		//This is Bus error

		//clear the bus error flag
		p_I2CHandle->p_I2C->SR1 &= ~(1 << I2C_SR1_BERR);


		//Implement the notify the application about the error
	   I2C_ApplicationEventCallback(p_I2CHandle, I2C_ER_BERR);
	}

	/***********************Check for arbitration lost error************************************/
	temp1 = (p_I2CHandle->p_I2C->SR1) & ( 1 << I2C_SR1_ARLO );
	if(temp1  && temp2)
	{
		//This is arbitration lost error

		//the arbitration lost error flag
		p_I2CHandle->p_I2C->SR1 &= ~(1 << I2C_SR1_ARLO);

		//notify the application about the error
		I2C_ApplicationEventCallback(p_I2CHandle, I2C_ER_ARLO);
	}

	/***********************Check for ACK failure error************************************/
	temp1 = (p_I2CHandle->p_I2C->SR1) & ( 1 << I2C_SR1_AF);
	if(temp1  && temp2)
	{
		//This is ACK failure error

		//the ACK failure error flag
		p_I2CHandle->p_I2C->SR1 &= ~(1 << I2C_SR1_AF);

		//notify the application about the error
		I2C_ApplicationEventCallback(p_I2CHandle, I2C_ER_AF);
	}

	/***********************Check for Overrun/underrun error************************************/
	temp1 = (p_I2CHandle->p_I2C->SR1) & ( 1 << I2C_SR1_OVR);
	if(temp1  && temp2)
	{
		//This is overrun/underrun

		//clear the Overrun/underrun error flag
		p_I2CHandle->p_I2C->SR1 &= ~(1 << I2C_SR1_OVR);

		//notify the application about the error
		I2C_ApplicationEventCallback(p_I2CHandle, I2C_ER_OVR);
	}

	/***********************Check for Time out error************************************/
	temp1 = (p_I2CHandle->p_I2C->SR1) & ( 1 << I2C_SR1_TIMEOUT);
	if(temp1  && temp2)
	{
		//This is time out error

		//clear the Time out error flag
		p_I2CHandle->p_I2C->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

		//notify the application about the error
		I2C_ApplicationEventCallback(p_I2CHandle, I2C_ER_TIMEOUT);
	}
}

