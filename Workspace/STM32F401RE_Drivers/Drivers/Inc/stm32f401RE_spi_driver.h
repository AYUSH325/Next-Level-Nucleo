/*
 * stm32f401RE_spi_driver.h
 *
 *  Created on: Mar 24, 2024
 *      Author: ayushganguly
 */

#ifndef INC_STM32F401RE_SPI_DRIVER_H_
#define INC_STM32F401RE_SPI_DRIVER_H_

#include "STM32F401RE.h"

/*
 * Configuration structure for SPI -> we have CR
 */

typedef struct
{
	uint8_t SPI_DeviceMode; // @SPI_DeviceMode
	uint8_t	SPI_BusConfig; // @SPI_BusConfig
	uint8_t SPI_SCLKSpeed; // @SPI_SclkSpeed
	uint8_t SPI_DFF; //@SPI_DFF
	uint8_t SPI_CPOL; //@SPI_CPOL
	uint8_t SPI_CPHA; //@SPI_CPHA
	uint8_t SPI_SSM; //@SPI_SSM

} SPI_Config_t;

/*
 * Handle structure for SPI;
 */

typedef struct
{
	SPI_Regdef_t* p_SPIx; // holds the base address of the SPIx peripheral, (x= 1,2,3)
	SPI_Config_t SPIConfig;

	// **************For Interrupt based data communication************
	uint8_t *p_TxBuffer; // To store the application Tx buffer address
	uint8_t	*p_RxBuffer; // To store the application Rx buffer address
	uint32_t TxLen; // To store Tx len
	uint32_t RxLen; // To store Rx len
	uint8_t  TxState; // To store Tx State
	uint8_t  RxState; // To store Rx State

} SPI_Handle_t;

/*
 * @SPI_DeviceMode
 */

#define SPI_DEVICE_MODE_MASTER 			1
#define SPI_DEVICE_MODE_SLAVE			0


/*
 * @SPI_BusConfig - to configure the BR[2:0] bit fields to control SCLK speed
 * The values apply a prescalar value to the bus peripheral clock of the respective SPIx peripheral
 */

#define SPI_BUS_CONFIG_FD				1 // full duplex
#define SPI_BUS_CONFIG_HD				2 // half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY	3	// simplex rxonly - MISO connected only -> RX_ONLY bit should be set to 1 so as to force master to produce clk for SPI comm
//#define SPI_BUS_CONFIG_SIMPLEX_TXONLY	4 // simplex txonly - MOSI connected only, this is full duplex with MISO line removed

/*
 * @SPI_SclkSpeed
 */

#define SPI_SCLK_SPEED_DIV2				0
#define SPI_SCLK_SPEED_DIV4				1
#define SPI_SCLK_SPEED_DIV8				2
#define SPI_SCLK_SPEED_DIV16			3
#define SPI_SCLK_SPEED_DIV32			4
#define SPI_SCLK_SPEED_DIV64			5
#define SPI_SCLK_SPEED_DIV128			6
#define SPI_SCLK_SPEED_DIV256			7

/*
 * @SPI_DFF
 */

#define SPI_DFF_8BITS					0   // default
#define SPI_DFF_16BITS					1

/*
 * @SPI_CPOL
 */

#define SPI_CPOL_LOW					0
#define SPI_CPOL_HIGH					1


/*
 * @SPI_CPHA
 */

#define SPI_CPHA_LOW					0
#define SPI_CPHA_HIGH					1


/*
 * @SPI_SSM
 */

#define SPI_SSM_DI						0  // software SSM disabled by default so hardware is enabled
#define SPI_SSM_EN						1

/*
 * SPI related status flags definition
 */

#define SPI_TXE_FLAG 					(1 << SPI_SR_TXE)
#define SPI_RXNE_FLAG					(1 << SPI_SR_RXNE)
#define SPI_BUSY_FLAG					(1 << SPI_SR_BSY)

/*
 * SPI peripheral state macros
 */
#define SPI_READY						0  // SPI ready for data transmission/reception
#define SPI_BUSY_IN_RX					1  // SPI busy in reception
#define SPI_BUSY_IN_TX					2  // SPI busy in transmission


/*
 * Possible SPI Application event
 */

#define SPI_EVENT_TX_CMPLT				1
#define SPI_EVENT_RX_CMPLT				2
#define SPI_EVENT_OVR_ERR				3
#define SPI_EVENT_CRC_ERR				4


/************************************************************************************
  	  	  	  	  	  Driver API Prototype Requirements
 ***********************************************************************************/

/*
 * Peripheral clock setup
 */

void SPI_PeriClockControl(SPI_Regdef_t *p_SPIx, uint8_t EnorDi);


/*
 * Init and Deinit
 */

void SPI_Init(SPI_Handle_t *p_SPIHandle);
void SPI_DeInit(SPI_Regdef_t *p_SPIx);

/*
 * Gets the status of a bit in the SR register
 */
uint8_t SPI_GetFlagStatus(SPI_Regdef_t	*p_SPIx, uint32_t FlagName);


/*
 * Data Send and Receive
 * This can be polling based (blocking) or interrupt based (non-blocking) or DMA
 */
void SPI_SendData(SPI_Regdef_t *p_SPIx, uint8_t *p_TxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_Regdef_t	*p_SPIx, uint8_t *p_RxBuffer, uint32_t len);

uint8_t SPI_SendDataIT(SPI_Handle_t *p_SPIHandle, uint8_t *p_TxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t	*p_SPIHandle, volatile uint8_t *p_RxBuffer, uint32_t len);
/*
 * Other SPI Peripheral Control APIS
 */

void SPI_PeripheralControl(SPI_Regdef_t	*p_SPIx, uint8_t EnorDi);
void SPI_SSIConfig(SPI_Regdef_t	*p_SPIx, uint8_t EnorDi);
void SPI_SSOEConfig(SPI_Regdef_t *p_SPIx, uint8_t EnorDi);
void SPI_ClearOVRFlag(SPI_Regdef_t *p_SPIx);
void SPI_CloseTransmission(SPI_Handle_t *p_SPIHandle);
void SPI_CloseReception(SPI_Handle_t *p_SPIHandle);

/*
 * IRQ Configuration and ISR handling
 */

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *p_SPIHandle);

/*
 * Application Callback: Should be implemented in the application side
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *p_SPIHandle, uint8_t AppEV);


#endif /* INC_STM32F401RE_SPI_DRIVER_H_ */
