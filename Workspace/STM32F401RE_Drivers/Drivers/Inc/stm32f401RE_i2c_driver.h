/*
 * stm32f401RE_i2c_driver.h
 *
 *  Created on: Apr 13, 2024
 *      Author: ayushganguly
 */

// these prevent multiple definitions inclusions during preprocessor stage
#ifndef INC_STM32F401RE_I2C_DRIVER_H_
#define INC_STM32F401RE_I2C_DRIVER_H_

#include "STM32F401RE.h"

typedef struct
{
	uint32_t I2C_SCLSpeed; // @I2C_SCLSpeed
	uint8_t  I2C_DeviceAddress; // 7 bits configured by user 8th bit is read/W
	uint8_t	 I2C_ACKControl; //@I2C_AckControl
	uint8_t  I2C_FMDutyCycle; //@I2C_FMDutyCycle

} I2C_Config_t;

typedef struct
{
	I2C_Regdef_t *p_I2C;
	I2C_Config_t I2C_Config;
	uint8_t	*p_TxBuffer; // store the application Tx buffer address
	uint8_t *p_RxBuffer; // store the application Rx buffer address
	uint32_t txLen; // store tx len
	uint32_t rxLen; // store rx len
	uint8_t txRxState; // since I2C is half duplex we can use the lines one at a time, Rx or Tx - store communication state
	uint8_t slaveAddress; // store slave/device address
	uint32_t rxSize; // store rx size - this will be used to distinguish between the 2 cases of Master receive
	uint8_t sr; // store repeated start value
} I2C_Handle_t;

/*
 * R/W bit of the address byte during transmission
 */
#define WRITE					0
#define READ					1

/*
 * @I2C_SCLSpeed
 */

#define I2C_SCL_SPEED_SM 		100000 // standard mode
#define I2C_SCL_SPEED_FM4K 		400000 // fast mode
#define I2C_SCL_SPEED_FM2K 		200000 // fast mode


/*
 * @I2C_AckControl
 */

#define I2C_ACK_ENABLE			1
#define I2C_ACK_DISABLE			0

/*
 * @I2C_FMDutyCycle -> CCR Bit 14
 */
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*
 * I2C related status flags definitions
 */
#define I2C_TXE_FLAG 			(1 << I2C_SR1_TXE)
#define I2C_RXNE_FLAG			(1 << I2C_SR1_RXNE)
#define I2C_STOPF_FLAG			(1 << I2C_SR1_STOPF)
#define I2C_ADDR_FLAG			(1 << I2C_SR1_ADDR)
#define I2C_ARLO_FLAG			(1 << I2C_SR1_ARLO)
#define I2C_BERR_FLAG			(1 << I2C_SR1_BERR)
#define I2C_OVR_FLAG			(1 << I2C_SR1_OVR)
#define I2C_TIMEOUT_FLAG		(1 << I2C_SR1_TIMEOUT)
#define I2C_BTF_FLAG			(1 << I2C_SR1_BTF)
#define I2C_SB_FLAG				(1 << I2C_SR1_SB)
#define I2C_AF_FLAG				(1 << I2C_SR1_AF)

/*
 * Max Frise for different modes = 1/Trise
 */
#define I2C_MAX_FRISE_SM		1000000
#define I2C_MAX_FRISE_FM		3333333

/*
 * I2C TX/RX interrupt applications states
 */
#define I2C_READY				0
#define I2C_BUSY_IN_TX			1
#define I2C_BUSY_IN_RX			2

/*
 * I2C Application Event Macros
 */
#define I2C_EV_TX_CMPLT			0
#define I2C_EV_RX_CMPLT			1
#define I2C_EV_STOP				2
#define I2C_ER_BERR				3
#define I2C_ER_ARLO				4
#define I2C_ER_AF				5
#define I2C_ER_OVR				6
#define I2C_ER_TIMEOUT			7
#define I2C_EV_DATA_REQ			8
#define I2C_EV_DATA_RECV		9


/************************************************************************************
  	  	  	  	  	  Driver API Prototype Requirements
 ***********************************************************************************/

/*
 * Peripheral clock setup
 */

void I2C_PeriClockControl(I2C_Regdef_t *p_I2Cx, uint8_t EnorDi);


/*
 * Init and Deinit
 */

void I2C_Init(I2C_Handle_t *p_I2CHandle);
void I2C_DeInit(I2C_Regdef_t *p_I2Cx);

/*
 * Gets the status of a bit in the SR register
 */
uint8_t I2C_GetFlagStatus(I2C_Regdef_t	*p_I2Cx, uint32_t FlagName);


/*
 * Data Send and Receive
 * This can be polling based (blocking) or interrupt based (non-blocking) or DMA
 */

/*
 * Polling based APIs
 */

void I2C_MasterSendData(I2C_Handle_t *p_I2CHandle, uint8_t *p_TxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *p_I2CHandle, uint8_t *p_RxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t Sr);

/*
 * Interrupt based APIs
 */
uint8_t I2C_MasterSendData_IT(I2C_Handle_t *p_I2CHandle, uint8_t *p_TxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr);
uint8_t I2C_MasterReceiveData_IT(I2C_Handle_t *p_I2CHandle, uint8_t *p_RxBuffer, uint32_t len, uint8_t slaveAddr, uint8_t sr);
void I2C_EV_IRQHandling(I2C_Handle_t *p_I2CHandle);
void I2C_ER_IRQHandling(I2C_Handle_t *p_I2CHandle);

/*
 * Close Send & Receive Data
 */
void I2C_CloseSendData(I2C_Handle_t *p_I2CHandle);
void I2C_CloseReceiveData(I2C_Handle_t *p_I2CHandle);

/*
 * Send and Receive Data in slave mode
 */
void I2C_SlaveSendData(I2C_Regdef_t *p_I2Cx, uint8_t data); //for read operation, sends the data byte by byte
uint8_t I2C_SlaveReceiveData(I2C_Regdef_t *p_I2Cx); // when master sends data i.e for write operation



/*********************************************************************************************/

/*
 * Other I2C Peripheral Control APIS
 */

void I2C_PeripheralControl(I2C_Regdef_t	*p_I2Cx, uint8_t EnorDi);
void I2C_ManageAcking(I2C_Regdef_t	*p_I2Cx, uint8_t EnorDi);
void I2C_GenerateStopCondition(I2C_Regdef_t *p_I2C);

void I2C_SLaveEnableDisableCallbackEvents(I2C_Regdef_t	*p_I2Cx, uint8_t EnorDi);
/*
 * IRQ Configuration and ISR handling
 */

void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);

/*
 * Application Callback: Should be implemented in the application side
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *p_I2CHandle, uint8_t AppEV);




#endif /* INC_STM32F401RE_I2C_DRIVER_H_ */
