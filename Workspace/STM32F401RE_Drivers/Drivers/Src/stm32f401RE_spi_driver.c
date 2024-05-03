/*
 * stm32f401RE_spi_driver.c
 *
 *  Created on: Mar 24, 2024
 *      Author: ayushganguly
 */

#include "stm32f401RE_spi_driver.h"

static void SPI_txe_interrupt_handle(SPI_Handle_t *p_SPIHandle);
static void SPI_rxe_interrupt_handle(SPI_Handle_t *p_SPIHandle);
static void SPI_ovr_err_interrupt_handle(SPI_Handle_t *p_SPIHandle);

/*
 * SPI Peripheral clock enable
 * @param1 p_SPIx: takes in the base address of the SPI port peripheral to figure out which port's bit to be set to one in RCC clk bus enable reg
 * @param2: Enable/ Disable macros
 */
void SPI_PeriClockControl(SPI_Regdef_t *p_SPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
		{
			if(p_SPIx == SPI1)
			{
				SPI1_PCLK_EN();
			}
			else if (p_SPIx == SPI2)
			{
				SPI2_PCLK_EN();
			}
			else if (p_SPIx == SPI3)
			{
				SPI3_PCLK_EN();
			}
			else if (p_SPIx == SPI4)
			{
				SPI4_PCLK_EN();
			}
		}
		else
		{
			if(p_SPIx == SPI1)
			{
				SPI1_PCLK_DI();
			}
			else if (p_SPIx == SPI2)
			{
				SPI2_PCLK_DI();
			}
			else if (p_SPIx == SPI3)
			{
				SPI3_PCLK_DI();
			}
			else if (p_SPIx == SPI4)
			{
				SPI4_PCLK_DI();
			}
		}
}

/*
 * Init and Deinit
 */

/*
 * Initialize the SPI peripheral with the specific base address and
 * set its registers to the values depending on the application
 * @param: pointer to the SPI Handle
 */
void SPI_Init(SPI_Handle_t *p_SPIHandle)
{

	// 1) configure SPI_CR1 register
	uint32_t temp = 0;

	// i) Configure the device mode
	temp = temp | p_SPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;
	// clear
	p_SPIHandle->p_SPIx->CR1 &= ~(0x4);

	// ii) Configure the bus config
	// clear the BIDI bit

	if (p_SPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// set BIDI mode should be cleared
		p_SPIHandle->p_SPIx->CR1 &= ~(0x8000);
	}
	else if (p_SPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BIDI mode should be set
		p_SPIHandle->p_SPIx->CR1 |= 1 << SPI_CR1_BIDI_MODE;
	}
	else if (p_SPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// BIDI mode should be cleared for FG but RX_Only bit should be enabled
		p_SPIHandle->p_SPIx->CR1 &= ~(0x8000);
		p_SPIHandle->p_SPIx->CR1 |= 1 << SPI_CR1_RX_ONLY;
	}

	// iii) Configure the SPI serial clock speed (baud rate)
	// clear the bit fields BR[2:0}
	p_SPIHandle->p_SPIx->CR1 &= ~(0x0038);

	temp |= p_SPIHandle->SPIConfig.SPI_SCLKSpeed << SPI_CR1_BR;

	// iv) Configure the DFF
	//reset
	p_SPIHandle->p_SPIx->CR1 &= ~(0x0800);

	temp |= p_SPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	// v) Configure the CPOL
	//reset
	p_SPIHandle->p_SPIx->CR1 &= ~(0x0002);

	temp |= p_SPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	// vi) Configure the CPHA
	//reset
	p_SPIHandle->p_SPIx->CR1 &= ~(0x0001);

	temp |= p_SPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	// vii) Configure the SSM bit
	//reset
	p_SPIHandle->p_SPIx->CR1 &= ~(0x0200);

	temp |= p_SPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;


	// set all the configs to the CR1 register
	p_SPIHandle->p_SPIx->CR1 |= temp;
}

/*
 * Enable/Disable the SSI bit in CR1 register, should be done when using SSM to pull the NSS pin to
 * VDD in non multi master environment
 * @param1 p_GPIOx: takes in the base address of the SPI peripheral
 * @param2: Enable/ Disable macros
 */

void SPI_SSIConfig(SPI_Regdef_t *p_SPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		p_SPIx->CR1 |= 1 << SPI_CR1_SSI;
	}
	else
	{
		p_SPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}

/*
 * Enable or Disable the SPE bit in CR1 reg to begin or end protocol communication
 * For disabling the peripheral follow rules in the user manual
 * @param1 p_GPIOx: takes in the base address of the SPI peripheral
 * @param2: Enable/ Disable macros
 */

void SPI_PeripheralControl(SPI_Regdef_t	*p_SPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		p_SPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		p_SPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*
 * Enable or Disable the SSOE bit in CR1 reg to enable NSS pin output when NSS pin is pulled low
 * Note: NSS pin is pulled low automatically by hardware Slave management (SSM = 0) when SPE = 1
 * For disabling the peripheral follow rules in the user manual
 * @param1 p_GPIOx: takes in the base address of the SPI peripheral
 * @param2: Enable/ Disable macros
 */
void SPI_SSOEConfig(SPI_Regdef_t *p_SPIx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		p_SPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}
	else
	{
		p_SPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}

/*
 * DeInitialize the SPI peripheral by reseting all values in its registers
 * @param: pointer to SPI base address
 */
void SPI_DeInit(SPI_Regdef_t *p_SPIx)
{
	// rcc peripheral set the reset bus bit and then reset
}


/*
 * Data Send and Receive
 * This can be polling based (blocking)
 * This is a blocking api as we have to wait to send all the data from TX buffer before putting more data in it
 *
 * @param1: pointer to the SPI peripheral base address
 * @param2: pointer to the TX buffer with the data to transmit to the external world
 * @param3: length of the data to be transmitted, always max limit is 32 bytes
 */
void SPI_SendData(SPI_Regdef_t	*p_SPIx, uint8_t *p_TxBuffer, uint32_t len)
{
	// Loop until len of data to be sent is 0, each len transaction is 1Byte
	// but we can send 1byte or 2byte depending on DFF bit of CR register

	while (len > 0)
	{
		// 1.) wait until TXE flag is 1 i.e tx buffer empty (Blocking)
		while(SPI_GetFlagStatus(p_SPIx, SPI_TXE_FLAG) == FLAG_RESET);

		// check the DFF bit is set
		if ((p_SPIx->CR1 & (1 << SPI_CR1_DFF)))
		{
			// 16bit data transmission
			// 1. Load the 16 bit data in the DR.
			p_SPIx->DR = *((uint16_t*)p_TxBuffer);
			(uint16_t *)p_TxBuffer++; // increment by 2 bytes
			len = len - 2;
		}
		else
		{
			// 8 bit data transmission
			p_SPIx->DR = *p_TxBuffer; // remember to dereference because this is a pointer
			p_TxBuffer++;	// increment to next byte
			len--;
		}

	}

}

/*
 * Get flag status of register
 * @param1: SPI peripheral base address
 * @param2: the flag
 * @return: the status of that register
 */
uint8_t SPI_GetFlagStatus(SPI_Regdef_t	*p_SPIx, uint32_t FlagName)
{

	if ((p_SPIx->SR & FlagName) == RESET)
	{
		return FLAG_RESET;
	}
	return FLAG_SET;
}

/*
 * @param1: pointer to the SPI peripheral base address
 * @param2: pointer to the RX buffer with the data that is received from the external world
 * @param3: length of the data to be received, always max limit is 32 bytes
 */

void SPI_ReceiveData(SPI_Regdef_t *p_SPIx, uint8_t *p_RxBuffer, uint32_t len)
{
	while (len > 0)
	{
		// 1.) Wait until RXNE flag is 1 i.e rx buffer should not be empty (blocking code)
		while(SPI_GetFlagStatus(p_SPIx, SPI_RXNE_FLAG) == (uint8_t) FLAG_RESET);

		if (p_SPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit data transmission
			// load the 2B data from DR to Rx bufer given by user
			*(uint16_t*) p_RxBuffer = p_SPIx->DR;
			(uint16_t*) p_RxBuffer++;
			len = len - 2;

		}
		else
		{
			// 8 bit data transmission
			// load the 1B data from DR to Rx bufer given by user
			*p_RxBuffer = p_SPIx->DR;
			p_RxBuffer++;
			len--;
		}
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
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
		else if	(IRQNumber >= 64 && IRQNumber < 96)
		{
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNumber - 64));
		}
	}
}

/*
 * Configure the IRQ Priority number of the interrupt
 * @param1: IRQNumber
 * @param2: Priority of IRQ - possible values from @SP_PRIORITY_VALUES
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
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
 * Data send non blocking interrupt way
 *
 * @param1: pointer to the SPI peripheral base address
 * @param2: pointer to the TX buffer with the data to transmit to the external world
 * @param3: length of the data to be transmitted, always max limit is 32 bytes
 *
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *p_SPIHandle, uint8_t *p_TxBuffer, uint32_t len)
{
	uint8_t state = p_SPIHandle->TxState;

	if (state != SPI_BUSY_IN_TX)
	{
		// 1. Save the TX buffer address and len in global variables
		p_SPIHandle->p_TxBuffer = p_TxBuffer;
		p_SPIHandle->TxLen = len;

		// 2. Mark the SPI state as busy in transmission so that no other
		// code can take over same SPI peripheral until transmission is over

		p_SPIHandle->TxState = SPI_BUSY_IN_TX;

		// 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		p_SPIHandle->p_SPIx->CR2 |= (1 << SPI_CR2_TXEIE);

		//Data transmission will be handled by the ISR code
	}
	return state;

}



/*
 * Data receive non blocking interrupt way
 *
 * @param1: pointer to the SPI peripheral base address
 * @param2: pointer to the TX buffer with the data to transmit to the external world
 * @param3: length of the data to be transmitted, always max limit is 32 bytes
 *
 */

uint8_t SPI_ReceiveDataIT(SPI_Handle_t	*p_SPIHandle, volatile uint8_t *p_RxBuffer, uint32_t len)
{
	uint8_t state = p_SPIHandle->RxState;

	if (state != SPI_BUSY_IN_RX)
	{
		// 1. Save the TX buffer address and len in global variables
		p_SPIHandle->p_RxBuffer = p_RxBuffer;
		p_SPIHandle->RxLen = len;

		// 2. Mark the SPI state as busy in transmission so that no other
		// code can take over same SPI peripheral until transmission is over

		p_SPIHandle->RxState = SPI_BUSY_IN_RX;

		// 3. Enable the RXNEIE control bit to get interrupt whenever TXE flag is set in SR
		p_SPIHandle->p_SPIx->CR2 |= (1 << SPI_CR2_RXNEIE);

		// Data reception will be handled by the ISR code
	}
	return state;
}

/*
 * Whenever we have an interrupt trigger, we have this function to process the interrupt
 * 6 possible reasons, understand which event caused the interrupt to trigger, check status
 * (Master mode fault/ CRC error & TI error not handled)
 * register -> the RXNE flag r TXE flag or error flag.
 * @param1: pointer to SPI handler
 */
void SPI_IRQHandling(SPI_Handle_t *p_SPIHandle)
{
	uint8_t currEventStatus, currControlStatus;

	// first check for TX status
	currEventStatus = p_SPIHandle->p_SPIx->SR & (1 << SPI_SR_TXE); // transmit buffer empty?
	currControlStatus = p_SPIHandle->p_SPIx->SR & (1 << SPI_CR2_TXEIE);

	if (currEventStatus && currControlStatus)
	{
		SPI_txe_interrupt_handle(p_SPIHandle);
	}

	// then check for RX status
	currEventStatus = p_SPIHandle->p_SPIx->SR & (1 << SPI_SR_RXNE); // receive buffer not empty?
	currControlStatus = p_SPIHandle->p_SPIx->SR & (1 << SPI_CR2_RXNEIE);


	if (currEventStatus && currControlStatus)
	{
		SPI_rxe_interrupt_handle(p_SPIHandle);
	}


	// Check for Overrun error
	currEventStatus = p_SPIHandle->p_SPIx->SR & (1 << SPI_SR_OVR); // over run bit set?
	currControlStatus = p_SPIHandle->p_SPIx->SR & (1 << SPI_CR2_ERRIE);

	if (currEventStatus && currControlStatus)
	{
		SPI_ovr_err_interrupt_handle(p_SPIHandle);
	}

}

// helper functions

/*
 * This is the interrupt handler for transmitting the data as and when an interrupt is triggered
 * @param1: pointer to SPI handler
 */
static void SPI_txe_interrupt_handle(SPI_Handle_t *p_SPIHandle)
{
	// check the DFF bit is set
	if ((p_SPIHandle->p_SPIx->CR1 & (1 << SPI_CR1_DFF)))
	{
		// 16bit data transmission
		// 1. Load the 16 bit data in the DR.
		p_SPIHandle->p_SPIx->DR = *((uint16_t*)p_SPIHandle->p_TxBuffer);
		(uint16_t *)p_SPIHandle->p_TxBuffer++; // increment by 2 bytes
		p_SPIHandle->TxLen = p_SPIHandle->TxLen - 2;
	}
	else
	{
		// 8 bit data transmission
		p_SPIHandle->p_SPIx->DR = *(p_SPIHandle->p_TxBuffer); // remember to dereference because this is a pointer
		p_SPIHandle->p_TxBuffer++;	// increment to next byte
		p_SPIHandle->TxLen--;
	}

	if (!p_SPIHandle->TxLen)
	{
		// TXLen is zero, so end the SPI transmission, reset the SPI Handle and inform the application of it
		// TX is over

		// this prevents the interrupt from getting triggered
		SPI_CloseTransmission(p_SPIHandle);

		SPI_ApplicationEventCallback(p_SPIHandle, SPI_EVENT_TX_CMPLT);
	}

}


/*
 * This is the interrupt handler for receiving the data as and when an interrupt is triggered
 * @param1: pointer to SPI handler
 */
static void SPI_rxe_interrupt_handle(SPI_Handle_t *p_SPIHandle)
{

	if (p_SPIHandle->p_SPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		// 16 bit data transmission
		// load the 2B data from DR to Rx buFfer given by user
		*((uint16_t*) p_SPIHandle->p_RxBuffer) = p_SPIHandle->p_SPIx->DR;
		(uint16_t*) p_SPIHandle->p_RxBuffer++;
		p_SPIHandle->RxLen = p_SPIHandle->RxLen - 2;
	}
	else
	{
		// 8 bit data transmission
		// load the 1B data from DR to Rx bufer given by user
		*(p_SPIHandle->p_RxBuffer) = p_SPIHandle->p_SPIx->DR;
		p_SPIHandle->p_RxBuffer++;
		p_SPIHandle->RxLen--;
	}

	if (!p_SPIHandle->RxLen)
	{
		// TXLen is zero, so end the SPI transmission, reset the SPI Handle and inform the application of it
		// TX is over

		// this prevents the interrupt from getting triggered
		SPI_CloseReception(p_SPIHandle);

		SPI_ApplicationEventCallback(p_SPIHandle, SPI_EVENT_RX_CMPLT);
	}
}



static void SPI_ovr_err_interrupt_handle(SPI_Handle_t *p_SPIHandle)
{
	uint8_t temp;

	// clear the ovr flag when SPI is not busy in transmission, if busy in transmission
	// clear the flag from application side

	if (p_SPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = p_SPIHandle->p_SPIx->DR;
		temp = p_SPIHandle->p_SPIx->SR;
	}

	(void) temp;
	// inform the application, also clear the OVR flag from application side if the peripheral
	// was busy in transmission
	SPI_ApplicationEventCallback(p_SPIHandle, SPI_EVENT_OVR_ERR);
}

// Application can call these APIs to close comms abruptly
void SPI_ClearOVRFlag(SPI_Regdef_t *p_SPIx)
{
	uint8_t temp;
	temp = p_SPIx->DR;
	temp = p_SPIx->SR;
	(void) temp;
}

/*
 * End current SPI Transmission
 * @param1: pointer to SPI handler
 */
void SPI_CloseTransmission(SPI_Handle_t *p_SPIHandle)
{
	p_SPIHandle->p_SPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	p_SPIHandle->p_TxBuffer = NULL;
	p_SPIHandle->TxLen = 0;
	p_SPIHandle->TxState = SPI_READY;
}

/*
 * End current SPI reception
 * @param1: pointer to SPI handler
 */

void SPI_CloseReception(SPI_Handle_t *p_SPIHandle)
{
	p_SPIHandle->p_SPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	p_SPIHandle->p_RxBuffer = NULL;
	p_SPIHandle->RxLen = 0;
	p_SPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *p_SPIHandle, uint8_t AppEV)
{
	// Weak implementation, override in application side
}
