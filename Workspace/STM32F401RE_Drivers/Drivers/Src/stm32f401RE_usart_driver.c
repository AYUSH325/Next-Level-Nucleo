/*
 * stm32f401RE_usart_driver.c
 *
 *  Created on: Apr 21, 2024
 *      Author: ayushganguly
 */
#include <stddef.h>

#include "STM32F401RE.h"

/*
 * Peripheral clock setup
 */

void USART_PeriClockControl(USART_Regdef_t *p_USARTx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if(p_USARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if (p_USARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if (p_USARTx == USART6)
		{
			USART6_PCLK_EN();
		}
	}
	else
	{
		if(p_USARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if (p_USARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if (p_USARTx == USART6)
		{
			USART6_PCLK_DI();
		}
	}
}


/*
 * Init and Deinit
 */

/*
 * Initialize the USART peripheral with the specific base address and
 * set its registers to the values depending on the application
 * @param: pointer to the USART Handle
 */

void USART_Init(USART_Handle_t *p_USARTHandle)
{
	uint32_t tempreg=0;

	/******************************** Configuration of CR1******************************************/

	//Enable USART TX and RX engines according to the USART_Mode configuration item
	if ( p_USARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//enable the Receiver bit field
		tempreg |= (1 << USART_CR1_RE);
	} else if (p_USARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//enable the Transmitter bit field
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (p_USARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//enable the both Transmitter and Receiver bit fields
		tempreg |=  ( 1 << USART_CR1_RE);
		tempreg	|=  ( 1 << USART_CR1_TE);
	}

	//configure the Word length configuration item
	tempreg |= p_USARTHandle->USART_Config.USART_WordLength << USART_CR1_M;


	//Configuration of parity control bit fields
	if ( p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		// will be configured to do ED & EC with even parity by default

	}else if (p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//enable the parity control
		tempreg |= ( 1 << USART_CR1_PCE);

		//enable ODD parity
		tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	p_USARTHandle->p_USARTx->CR1 |= tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//configure the number of stop bits inserted during USART frame transmission
	tempreg |= p_USARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	p_USARTHandle->p_USARTx->CR2 |= tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( p_USARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//enable CTS flow control
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (p_USARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//enable RTS flow control
		tempreg |= ( 1 << USART_CR3_RTSE);

	}else if (p_USARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//enable both CTS and RTS Flow control
		tempreg |= ( 1 << USART_CR3_CTSE);
		tempreg |= ( 1 << USART_CR3_RTSE);
	}


	p_USARTHandle->p_USARTx->CR3 |= tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//configure the baud rate
	USART_SetBaudRate(p_USARTHandle->p_USARTx, p_USARTHandle->USART_Config.USART_Baud);


}


/*
 * Configures the user defined Baud Rate by setting the USARTDIV in the BRR register
 * @param1: Base address of the USART peripheral
 * @param2: user defined baud rate from @USART_Baud
 */
void USART_SetBaudRate(USART_Regdef_t *p_USARTx, uint32_t baudRate)
{
	//hold the mantissa & fraction values
	uint32_t mantissa;
	uint32_t fraction;

	// will set the mantissa * fration values to temp
	uint32_t temp = 0;

	// peripheral clock frequency & USARTDIV
	uint32_t F_PCLK, value;

	// Get peripheral clock frequency
	if (p_USARTx == USART1 || p_USARTx == USART6)
	{
		// APB2
		F_PCLK = RCC_GetPCLKValue(APB2);
	}
	else
	{
		// APB1
		F_PCLK = RCC_GetPCLKValue(APB1);
	}

	// check what is the oversampling rate& calculate USARTDIV
	if (p_USARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// oversampling by 8
		value = (F_PCLK * 100) / ( 8 * baudRate);
	}
	else
	{
		// oversampling by 16
		value = (F_PCLK * 100) / ( 16 * baudRate);
	}

	// calculate mantissa
	mantissa = value / 100;
	temp |= mantissa << MANTISSA_SHIFT;

	// calculate fraction part
	fraction = value - (mantissa*100);

	// factor in the oversampling rate
	if (p_USARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// oversampling by 8
		fraction = fraction * 8;
	}
	else
	{
		// oversampling by 16
		fraction = fraction * 16;
	}

	// round off
	fraction += 50;

	// have to divide by 100 since multiplied by 100 to start with
	fraction = fraction / 100;

	// set the fraction bits depending on the oversampling used
	if (p_USARTx->CR1 & (1 << USART_CR1_OVER8))
	{
		// oversampling by 8 - can only use 3 bits for fraction
		temp |= fraction & (uint8_t)0x07;
	}
	else
	{
		// oversampling by 16 - can use the last 4 bits
		temp |= fraction & (uint8_t)0x0F;
	}

	// set the BRR register
	p_USARTx->BRR |= temp;
}




/*
 * Reset all register contents of the USART peripheral
 * @param1: p_USARTx: Base address of the USART port to know which bit of the
 * 					RCC reset bus registers to reset

 */
void USART_DeInit(USART_Regdef_t *p_USARTx)
{

	if (p_USARTx == USART1)
	{
		USART1_PCLK_RESET();
	}else if (p_USARTx == USART2)
	{
		USART2_PCLK_RESET();
	}else if (p_USARTx == USART6)
	{
		USART6_PCLK_RESET();
	}
}

/*
 * Get flag status of register
 * @param1: USART peripheral base address
 * @param2: the flag
 * @return: the status of that register
 */
uint8_t USART_GetFlagStatus(USART_Regdef_t	*p_USARTx, uint32_t FlagName)
{
	if (p_USARTx->SR & FlagName)
	{
		return SET;
	}
	return RESET;
}


/*
 * Clear the status flag in the SR register
 */
void USART_ClearFlag(USART_Regdef_t *p_USARTx, uint32_t statusFlagName)
{
	p_USARTx->SR &= ~(1 << statusFlagName);
}

/*
 * Data Send and Receive
 * This can be polling based (blocking) or interrupt based (non-blocking) or DMA
 */
void USART_SendData(USART_Handle_t *p_USARTHandle, uint8_t *p_TxBuffer, uint32_t len)
{
	while (len > 0)
	{
		while(USART_GetFlagStatus(p_USARTHandle->p_USARTx, USART_SR_TX));

		if (p_USARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// 9 bits of transmission
			p_USARTHandle->p_USARTx->DR = (*((uint16_t*) p_TxBuffer) & (uint16_t) 0x01FF);

			if (p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// if parity is disabled then 9 data bits so increase buffer address by 2
				p_TxBuffer += 2;
				len --;
			}
			else
			{
				// 9th bit is parity, so 8 data bits
				p_TxBuffer++;
				len--;
			}
		}
		else
		{
			// 8 bits of transmission
			// write to DR
			p_USARTHandle->p_USARTx->DR = *p_TxBuffer & 0xFF;

			// increment address & LEN
			p_TxBuffer++;
			len--;
		}

	}

	// after last byte transferred wait until TC becomes 1
	while(!USART_GetFlagStatus(p_USARTHandle->p_USARTx, USART_SR_TC));

}

void USART_ReceiveData(USART_Handle_t *p_USARTHandle, uint8_t *p_RxBuffer, uint32_t len)
{

	while (len > 0)
	{
		// wait for RXNE flag to be set so that data is in RDR register
		while(!USART_GetFlagStatus(p_USARTHandle->p_USARTx, USART_SR_RXNE));

		if (p_USARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			// 9 bits of reception


			if (p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// if parity is disabled then 9 data bits so increase buffer address by 2
				*((uint16_t*) p_RxBuffer) = p_USARTHandle->p_USARTx->DR & (uint16_t)0x01FF;
				p_RxBuffer += 2;
				len --;
			}
			else
			{
				// 9th bit is parity, so just read 8 data bits
				*(p_RxBuffer) = p_USARTHandle->p_USARTx->DR & (uint8_t)0xFF;
				p_RxBuffer++;
				len--;
			}
		}
		else
		{
			// 8 bits of reception
			// Read to DR

			if (p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				// if parity is disabled then 9 data bits so increase buffer address by 2
				*p_RxBuffer = p_USARTHandle->p_USARTx->DR  & (uint8_t)0xFF;
			}
			else
			{
				// 9th bit is parity, so 8 data bits
				*p_RxBuffer = p_USARTHandle->p_USARTx->DR  & (uint8_t)0x7F;
			}

			// increment address & LEN
			p_RxBuffer++;
			len--;
		}

	}
}

uint8_t USART_SendDataIT(USART_Handle_t *p_USARTHandle, uint8_t *p_TxBuffer, uint32_t len)
{

	uint8_t txState = p_USARTHandle->USART_txState;

	if (txState != USART_BUSY_IN_TX)
	{
		p_USARTHandle->USART_txLen = len;
		p_USARTHandle->USART_txState = USART_BUSY_IN_TX;
		p_USARTHandle->p_USART_txBuffer = p_TxBuffer;

		// enable control bit to receive TXE interrupt
		p_USARTHandle->p_USARTx->CR1 |= (1 << USART_CR1_TXEIE);

		// enable control bit to receive TC interrupt
		p_USARTHandle->p_USARTx->CR1 |= (1 << USART_CR1_TCIE);

		uint8_t isUseCTS = p_USARTHandle->USART_Config.USART_HWFlowControl;
		if (isUseCTS == USART_HW_FLOW_CTRL_CTS || isUseCTS == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			p_USARTHandle->p_USARTx->CR3 |= (1 << USART_CR3_CTSE);
			p_USARTHandle->p_USARTx->CR3 |= (1 << USART_CR3_CTSIE);
		}
	}

	return txState;

}

uint8_t USART_ReceiveDataIT(USART_Handle_t	*p_USARTHandle, uint8_t *p_RxBuffer, uint32_t len)
{

	uint8_t rxState = p_USARTHandle->USART_rxState;

	if (rxState != USART_BUSY_IN_RX)
	{
		p_USARTHandle->USART_rxLen = len;
		p_USARTHandle->USART_rxState = USART_BUSY_IN_RX;
		p_USARTHandle->p_USART_rxBuffer = p_RxBuffer;

		// enable control bit to receive TXE interrupt
		p_USARTHandle->p_USARTx->CR1 |= (1 << USART_CR1_RXNEIE);
	}

	return rxState;
}


/*
 * Other USART Peripheral Control APIS
 */

void USART_PeripheralControl(USART_Regdef_t	*p_USARTx, uint8_t EnorDi)
{
	if (EnorDi ==  ENABLE)
	{
		p_USARTx->CR1 |= (1 << USART_CR1_UE);
	}
	else
	{
		p_USARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/*
 * IRQ Configuration and ISR handling
 */

/*
 * Configure the NVIC registers of the ARM Cortex processor - set the IRQ number
 * @param1: IRQNumber
 * @param2: Enable or Disable
 */

void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
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
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. find out the IPR register
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



void USART_IRQHandling(USART_Handle_t *p_USARTHandle)
{
	uint32_t temp1 , temp2, temp3;

	/*************************Check for TC flag ********************************************/

	//Implement the code to check the state of TC bit in the SR
	temp1 = p_USARTHandle->p_USARTx->SR & ( 1 << USART_SR_TC);

	 //Implement the code to check the state of TCEIE bit
	temp2 = p_USARTHandle->p_USARTx->CR1 & ( 1 << USART_CR1_TCIE);

	if(temp1 && temp2 )
	{
		//this interrupt is because of TC

		//close transmission and call application callback if TxLen is zero
		if ( p_USARTHandle->USART_txState == USART_BUSY_IN_TX)
		{
			//Check the TxLen . If it is zero then close the data transmission
			if(! p_USARTHandle->USART_txLen )
			{
				//Implement the code to clear the TC flag
				p_USARTHandle->p_USARTx->SR &= ~(1 << USART_SR_TC);

				//Implement the code to clear the TCIE control bit
				p_USARTHandle->p_USARTx->CR1 &= ~(1 << USART_CR1_TCIE);

				//Reset the application state
				p_USARTHandle->USART_txState = USART_READY;

				//Reset Buffer address to NULL
				p_USARTHandle->p_USART_txBuffer = NULL;

				//Reset the length to zero
				p_USARTHandle->USART_txLen = 0;

				// Disable the CTS related interrupts in CR3
				uint8_t isUseCTS = p_USARTHandle->USART_Config.USART_HWFlowControl;
				if (isUseCTS == USART_HW_FLOW_CTRL_CTS || isUseCTS == USART_HW_FLOW_CTRL_CTS_RTS)
				{
					p_USARTHandle->p_USARTx->CR3 &= ~(1 << USART_CR3_CTSE);
					p_USARTHandle->p_USARTx->CR3 &= ~(1 << USART_CR3_CTSIE);
				}

				//Call the applicaton call back with event USART_EVENT_TX_CMPLT
				USART_ApplicationEventCallback(p_USARTHandle, USART_EVENT_TX_CMPLT);
			}
		}
	}

	/*************************Check for TXE flag ********************************************/

	//Implement the code to check the state of TXE bit in the SR
	temp1 = p_USARTHandle->p_USARTx->SR & ( 1 << USART_SR_TX);

	//Implement the code to check the state of TXEIE bit in CR1
	temp2 = p_USARTHandle->p_USARTx->CR1 & (1 << USART_CR1_TXEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of TXE

		if(p_USARTHandle->USART_txState == USART_BUSY_IN_TX)
		{
			//Keep sending data until Txlen reaches to zero
			if(p_USARTHandle->USART_txLen > 0)
			{
				//Check the USART_WordLength item for 9BIT or 8BIT in a frame
				if(p_USARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//if 9BIT , load the DR with 2bytes masking the bits other than first 9 bits
					p_USARTHandle->p_USARTx->DR = (*((uint16_t*) p_USARTHandle->p_USART_txBuffer) & (uint16_t) 0x01FF);

					//check for USART_ParityControl
					if(p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used in this transfer , so, 9bits of user data will be sent
						//Implement the code to increment pTxBuffer twice
						p_USARTHandle->p_USART_txBuffer += 2;

						//Implement the code to decrement the length
						p_USARTHandle->USART_txLen--;
					}
					else
					{
						//Parity bit is used in this transfer . so , 8bits of user data will be sent
						//The 9th bit will be replaced by parity bit by the hardware
						p_USARTHandle->p_USART_txBuffer++;

						//Implement the code to decrement the length
						p_USARTHandle->USART_txLen--;
					}
				}
				else
				{
					//This is 8bit data transfer
					p_USARTHandle->p_USARTx->DR = (*(p_USARTHandle->p_USART_txBuffer)  & (uint8_t)0xFF);

					//Implement the code to increment the buffer address
					p_USARTHandle->p_USART_txBuffer++;

					//Implement the code to decrement the length
					p_USARTHandle->USART_txLen--;
				}

			}
			if (p_USARTHandle->USART_txLen == 0 )
			{
				//TxLen is zero
				//Implement the code to clear the TXEIE bit (disable interrupt for TXE flag )
				p_USARTHandle->p_USARTx->CR1 &= ~(1 << USART_CR1_TXEIE);
			}
		}
	}

	/*************************Check for RXNE flag ********************************************/

	temp1 = p_USARTHandle->p_USARTx->SR & ( 1 << USART_SR_RXNE);
	temp2 = p_USARTHandle->p_USARTx->CR1 & ( 1 << USART_CR1_RXNEIE);


	if(temp1 && temp2 )
	{
		//this interrupt is because of rxne

		if(p_USARTHandle->USART_rxState == USART_BUSY_IN_RX)
		{
			//RXNE is set so receive data
			if(p_USARTHandle->USART_rxState > 0)
			{
				//Check the USART_WordLength to decide whether we are going to receive 9bit of data in a frame or 8 bit
				if(p_USARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
				{
					//We are going to receive 9bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used. so, all 9bits will be of user data

						//read only first 9 bits so mask the DR with 0x01FF
						*((uint16_t*) p_USARTHandle->p_USART_rxBuffer) = (p_USARTHandle->p_USARTx->DR  & (uint16_t)0x01FF);

						//Now increment the pRxBuffer two times
						p_USARTHandle->p_USART_rxBuffer++;
						p_USARTHandle->p_USART_rxBuffer++;

						//Implement the code to decrement the length
						p_USARTHandle->USART_rxLen--;
					}
					else
					{
						//Parity is used. so, 8bits will be of user data and 1 bit is parity
						 *(p_USARTHandle->p_USART_rxBuffer) = (p_USARTHandle->p_USARTx->DR  & (uint8_t)0xFF);

						 //Now increment the pRxBuffer
						 p_USARTHandle->p_USART_rxBuffer++;

						 //Implement the code to decrement the length
						 p_USARTHandle->USART_rxLen--;
					}
				}
				else
				{
					//We are going to receive 8bit data in a frame

					//Now, check are we using USART_ParityControl control or not
					if(p_USARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
					{
						//No parity is used , so all 8bits will be of user data

						//read 8 bits from DR
						 *(p_USARTHandle->p_USART_rxBuffer) = (uint8_t) (p_USARTHandle->p_USARTx->DR  & (uint8_t)0xFF);
					}

					else
					{
						//Parity is used, so , 7 bits will be of user data and 1 bit is parity

						//read only 7 bits , hence mask the DR with 0X7F
						 *(p_USARTHandle->p_USART_rxBuffer) = (uint8_t) (p_USARTHandle->p_USARTx->DR  & (uint8_t)0x7F);

					}

					//Now , increment the pRxBuffer
					p_USARTHandle->p_USART_rxBuffer++;

					//Implement the code to decrement the length
					p_USARTHandle->USART_rxLen--;
				}


			}

			if(!p_USARTHandle->USART_rxLen)
			{
				//disable the rxne
				p_USARTHandle->p_USARTx->CR1 &= ~( 1 << USART_CR1_RXNEIE );
				p_USARTHandle->USART_rxState = USART_READY;
				USART_ApplicationEventCallback(p_USARTHandle, USART_EVENT_RX_CMPLT);
			}
		}
	}


	/*************************Check for CTS flag ********************************************/

	//Implement the code to check the status of CTS bit in the SR
	temp1 = p_USARTHandle->p_USARTx->SR & (1 << USART_SR_CTS);

	//Implement the code to check the state of CTSE bit in CR3
	temp2 = p_USARTHandle->p_USARTx->CR3 & (1 << USART_CR3_CTSE);

	// Check the state of CTSIE bit in CR3
	temp3 = p_USARTHandle->p_USARTx->CR3 & (1 << USART_CR3_CTSIE);


	if(temp1  && temp2 && temp3 )
	{
		//Implement the code to clear the CTS flag in SR
		p_USARTHandle->p_USARTx->SR &= ~(1 << USART_SR_CTS);

		//this interrupt is because of cts
		USART_ApplicationEventCallback(p_USARTHandle, USART_EVENT_CTS);
	}

	/*************************Check for IDLE detection flag ********************************************/

	//Implement the code to check the status of IDLE flag bit in the SR
	temp1 = p_USARTHandle->p_USARTx->SR & (1 << USART_SR_IDLE);

	//Implement the code to check the state of IDLEIE bit in CR1
	temp2 = p_USARTHandle->p_USARTx->CR1 & (1 << USART_CR1_IDLEIE);


	if(temp1 && temp2)
	{
		//Implement the code to clear the IDLE flag
		uint32_t read = p_USARTHandle->p_USARTx->DR;
		(void)read;

		//this interrupt is because of idle
		USART_ApplicationEventCallback(p_USARTHandle, USART_EVENT_IDLE);
	}

	/*************************Check for Overrun detection flag ********************************************/

	//Implement the code to check the status of ORE flag  in the SR
	temp1 = p_USARTHandle->p_USARTx->SR & USART_SR_ORE;

	//Implement the code to check the status of RXNEIE  bit in the CR1
	temp2 = p_USARTHandle->p_USARTx->CR1 & USART_CR1_RXNEIE;


	if(temp1  && temp2 )
	{
		//Need not to clear the ORE flag here, instead an api for the application to clear the ORE flag .

		//this interrupt is because of Overrun error
		USART_ApplicationEventCallback(p_USARTHandle, USART_EVENT_ORE);
	}



	/*************************Check for Error Flag ********************************************/

	//Noise Flag, Overrun error and Framing Error in multibuffer communication
	//The blow code will get executed only if multibuffer mode is used.

	temp2 =  p_USARTHandle->p_USARTx->CR3 & ( 1 << USART_CR3_EIE) ;

	if(temp2 )
	{
		temp1 = p_USARTHandle->p_USARTx->SR;
		if(temp1 & ( 1 << USART_SR_FE))
		{
			/*
				This bit is set by hardware when a de-synchronization, excessive noise or a break character
				is detected. It is cleared by a software sequence (an read to the USART_SR register
				followed by a read to the USART_DR register).
			*/
			uint32_t read = p_USARTHandle->p_USARTx->DR;
			(void)read;
			USART_ApplicationEventCallback(p_USARTHandle, USART_ERREVENT_FE);
		}

		if(temp1 & ( 1 << USART_SR_NF) )
		{
			/*
				This bit is set by hardware when noise is detected on a received frame. It is cleared by a
				software sequence (an read to the USART_SR register followed by a read to the
				USART_DR register).
			*/
			uint32_t read = p_USARTHandle->p_USARTx->DR;
			(void)read;
			USART_ApplicationEventCallback(p_USARTHandle, USART_ERREVENT_NE);
		}

		if(temp1 & ( 1 << USART_SR_ORE) )
		{
			USART_ApplicationEventCallback(p_USARTHandle, USART_ERREVENT_ORE);
		}
	}

}


