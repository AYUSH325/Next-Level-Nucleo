/*
 * STM32F401RE.h
 *
 *  Created on: Mar 9, 2024
 *      Author: ayushganguly
 */

#ifndef INC_STM32F401RE_H_
#define INC_STM32F401RE_H_

#include <stdint.h>
#include <stddef.h>

#define __vo volatile
#define __weak __attribute__((weak))

/***********************************************Processor Specific Details*********************************************************************/

/*
 * ARM Cortex M4 Processor NVIC ISERx register addresses
 */

#define NVIC_ISER0			((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1			((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2			((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3			((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex M4 Processor NVIC ICERx register addresses for clearing the interrupt on the IRQ
 */

#define NVIC_ICER0			((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1			((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2			((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3			((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex M4 Processor NVIC IPR register address
 */

#define NVIC_IPR_BASE_ADDR	((__vo uint32_t*) 0xE000E400)

#define NO_PRIORITY_BITS_IMPLEMENTED	4 // Specific to MCU

/*
 * base address of Flash and SRAM memories
 */

// by default these are signed int by compiler, so we need to typecast to unsigned
// non-volatile but looses data if kept out of power supply for too long
#define FLASH_BASEADDR		0x08000000U
// volatile
#define SRAM_BASEADDR		0x20000000U
// ROM = system memory - non volatile
#define ROM_BASEADDR		0x1FFF0000U

/*
 *  base address of bus address connected to peripherals
 *  AHB bus - for high speed data communication peripherals
 *  APB bus - for low speed data communication peripherals
 */
#define APB1_BASEADDR		0x40000000U  // also base address for TIM2 TIM2_CR1 reg
#define APB2_BASEADDR		0x40007400U
#define AHB1_BASEADDR		0x40020000U  // also base address for GPIO
#define AHB2_BASEADDR		0x50000000U  // also base address for USB OTG FS

/*
 * Base addresses of Peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR		0x40020000U
#define GPIOB_BASEADDR		0x40020400U
#define GPIOC_BASEADDR		0x40020800U
#define GPIOD_BASEADDR		0x40020C00U
#define GPIOE_BASEADDR		0x40021000U
#define GPIOH_BASEADDR		0x40021C00U

#define RCC_BASEADDR		0x40023800U

/*
 * Base addresses of Peripherals on APB1 bus
 */
#define I2C1_BASEADDR		0x40005400U
#define I2C2_BASEADDR		0x40005800U
#define I2C3_BASEADDR		0x40005C00U

#define SPI2_BASEADDR		0x40003800U
#define SPI3_BASEADDR		0x40003C00U

#define USART2_BASEADDR		0x40004400U

/*
 * Base addresses of Peripherals on APB2 bus
 */
#define EXTI_BASEADDR		0x40013C00U
#define USART1_BASEADDR		0x40011000U
#define USART6_BASEADDR		0x40011400U
#define SPI1_BASEADDR		0x40013000U
#define SPI4_BASEADDR		0x40013400U
#define SYSCFG_BASEADDR		0x40013800U



/********************************* peripheral register definition structures *****************************************/



/*
 * Struct for GPIO registers
 */
typedef struct
{
	__vo uint32_t MODER;	// GPIO port mode register - Address_offset 0x00
	__vo uint32_t OTYPER;	// GPIO port output type register - Address_offset 0x04
	__vo uint32_t OSPEEDR;	// GPIO port output speed register
	__vo uint32_t PUPDR;	// GPIO port pull-up/pull-down register
	__vo uint32_t IDR;		// GPIO port input data register
	__vo uint32_t ODR;		// GPIO port output data register
	__vo uint32_t BSRR;		// GPIO port bit set/reset register
	__vo uint32_t LCKR;		// GPIO port configuration lock register
	__vo uint32_t AFR[2];  	// GPIO alternate function low - AFR[0] GPIO alternate function high - AFR[1]
} GPIO_Regdef_t;

/*
 * Struct for SPI registers
 */

typedef struct
{
	__vo uint32_t 	CR1;
	__vo uint32_t 	CR2;
	__vo uint32_t 	SR;
	__vo uint32_t	DR;
	__vo uint32_t	CRCPR;
	__vo uint32_t	RXCRCR;
	__vo uint32_t	TXCRCR;
	__vo uint32_t	I2SCFGR;
	__vo uint32_t	I2SPR;
} SPI_Regdef_t;

/*
 * Struct for I2C registers
 */
typedef struct
{
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t OAR1;  // slave stores its address
	__vo uint32_t OAR2;
	__vo uint32_t DR;
	__vo uint32_t SR1;
	__vo uint32_t SR2;
	__vo uint32_t CCR;
	__vo uint32_t TRISE;
	__vo uint32_t FLTR;
} I2C_Regdef_t;

/*
 * Struct for USART registers
 */
typedef struct
{
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t BRR;
	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t CR3;
	__vo uint32_t GTPR;
} USART_Regdef_t;


/*
 * Struct for RCC registers
 */
typedef struct
{
	  __vo uint32_t CR;            /*!,     										Address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /*!<      										Address offset: 0x04 */
	  __vo uint32_t CFGR;          /*!<     										Address offset: 0x08 */
	  __vo uint32_t CIR;           /*!<     										Address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /*!<     										Address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /*!<     										Address offset: 0x14 */
	  __vo uint32_t AHB3RSTR;      /*!<     										Address offset: 0x18 */
	  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                       */
	  __vo uint32_t APB1RSTR;      /*!<     										Address offset: 0x20 */
	  __vo uint32_t APB2RSTR;      /*!<     										Address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                  */
	  __vo uint32_t AHB1ENR;       /*!<	    										Address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /*!<     										Address offset: 0x34 */
	  __vo uint32_t AHB3ENR;       /*!<     										Address offset: 0x38 */
	  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                       */
	  __vo uint32_t APB1ENR;       /*!<     										Address offset: 0x40 */
	  __vo uint32_t APB2ENR;       /*!<     										Address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                  */
	  __vo uint32_t AHB1LPENR;     /*!<     										Address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /*!<     										Address offset: 0x54 */
	  __vo uint32_t AHB3LPENR;     /*!<     										Address offset: 0x58 */
	  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                       */
	  __vo uint32_t APB1LPENR;     /*!<     										Address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /*!<     										Address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                  */
	  __vo uint32_t BDCR;          /*!<     										Address offset: 0x70 */
	  __vo uint32_t CSR;           /*!<     										Address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                  */
	  __vo uint32_t SSCGR;         /*!<      										Address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /*!<      										Address offset: 0x84 */
	  __vo uint32_t PLLSAICFGR;    /*!<     										Address offset: 0x88 */
	  __vo uint32_t DCKCFGR;       /*!<     										Address offset: 0x8C */
	  __vo uint32_t CKGATENR;      /*!<     										Address offset: 0x90 */
	  __vo uint32_t DCKCFGR2;

} RCC_Regdef_t;

/*
 * Struct for EXTI peripheral registers
 * This is for interrupts
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

} EXTI_Regdef_t;

/*
 * Struct for SYSCFG peripheral registers
 */

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	uint32_t RESERVED[2];
	__vo uint32_t CMPCR;

} SYSCFG_Regdef_t;


/*
 * Peripheral definitions (Peripheral base addresses typecasted to struct xxx_RegDef_t)
 */

#define GPIOA 				((GPIO_Regdef_t*)GPIOA_BASEADDR)
#define GPIOB				((GPIO_Regdef_t*)GPIOB_BASEADDR)
#define GPIOC				((GPIO_Regdef_t*)GPIOC_BASEADDR)
#define GPIOD				((GPIO_Regdef_t*)GPIOD_BASEADDR)
#define GPIOE				((GPIO_Regdef_t*)GPIOE_BASEADDR)
#define GPIOH				((GPIO_Regdef_t*)GPIOH_BASEADDR)

#define RCC					((RCC_Regdef_t*)RCC_BASEADDR)

#define EXTI 				((EXTI_Regdef_t*)EXTI_BASEADDR)

#define SYSCFG				((SYSCFG_Regdef_t*)SYSCFG_BASEADDR)

#define SPI1				((SPI_Regdef_t*)SPI1_BASEADDR)
#define SPI2				((SPI_Regdef_t*)SPI2_BASEADDR)
#define SPI3				((SPI_Regdef_t*)SPI3_BASEADDR)
#define SPI4 				((SPI_Regdef_t*)SPI4_BASEADDR)

#define I2C1				((I2C_Regdef_t*)I2C1_BASEADDR)
#define I2C2				((I2C_Regdef_t*)I2C2_BASEADDR)
#define I2C3				((I2C_Regdef_t*)I2C3_BASEADDR)

#define USART1				((USART_Regdef_t*)USART1_BASEADDR)
#define USART6				((USART_Regdef_t*)USART6_BASEADDR)
#define USART2				((USART_Regdef_t*)USART2_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR = RCC->AHB1ENR | 0x00000001)
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR = RCC->AHB1ENR | 0x00000002)
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR = RCC->AHB1ENR | 0x00000004)
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR = RCC->AHB1ENR | 0x00000008)
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR = RCC->AHB1ENR | 0x00000010)
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR = RCC->AHB1ENR | 0x00000080)

/*
 * Clock Enable Macros for I2Cx peripherals
 */

#define I2C1_PLCK_EN()		(RCC->APB1ENR = RCC->APB1ENR | 0x00200000)
#define I2C2_PLCK_EN()		(RCC->APB1ENR = RCC->APB1ENR | 0x00400000)
#define I2C3_PLCK_EN()		(RCC->APB1ENR = RCC->APB1ENR | 0x00800000)

/*
 * Clock Enable Macros for SPIx peripherals
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR = RCC->APB2ENR | 0x00001000)
#define SPI2_PCLK_EN()		(RCC->APB1ENR = RCC->APB1ENR | 0x00004000)
#define SPI3_PCLK_EN()		(RCC->APB1ENR = RCC->APB1ENR | 0x00008000)
#define SPI4_PCLK_EN()		(RCC->APB2ENR = RCC->APB2ENR | 0x00002000)

/*
 * Clock Enable Macros for USARTx peripherals
 */

#define USART2_PCLK_EN()	(RCC->APB1ENR = RCC->APB1ENR | 0x00020000)
#define USART1_PCLK_EN()	(RCC->APB2ENR = RCC->APB2ENR | 0x00000010)
#define USART6_PCLK_EN()	(RCC->APB2ENR = RCC->APB2ENR | 0x00000020)

/*
 * Clock Enable Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR = RCC->APB2ENR | 0x00004000)

/*
 * Clock Disable Macros for GPIOx peripherals
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR = RCC->AHB1ENR & 0xFFFFFFFE)
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR = RCC->AHB1ENR & 0xFFFFFFFD)
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR = RCC->AHB1ENR & 0xFFFFFFFB)
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR = RCC->AHB1ENR & 0xFFFFFFF7)
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR = RCC->AHB1ENR & 0xFFFFFFEF)
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR = RCC->AHB1ENR & 0xFFFFFF7F)

/*
 * Clock Reset Macros for I2Cx peripherals
 */

#define I2C1_PLCK_DI()		(RCC->APB1ENR = RCC->APB1ENR & 0xFFDFFFFF)
#define I2C2_PLCK_DI()		(RCC->APB1ENR = RCC->APB1ENR & 0xFFBFFFFF)
#define I2C3_PLCK_DI()		(RCC->APB1ENR = RCC->APB1ENR & 0xFF7FFFFF)

/*
 * Clock Reset Macros for SPIx peripherals
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR = RCC->APB2ENR & 0xFFFFEFFF)
#define SPI2_PCLK_DI()		(RCC->APB1ENR = RCC->APB1ENR & 0xFFFFBFFF)
#define SPI3_PCLK_DI()		(RCC->APB1ENR = RCC->APB1ENR & 0xFFFF7FFF)
#define SPI4_PCLK_DI()		(RCC->APB2ENR = RCC->APB2ENR & 0xFFFFDFFF)

/*
 * Clock Reset Macros for USARTx peripherals
 */

#define USART2_PCLK_DI()	(RCC->APB1ENR = RCC->APB1ENR & 0xFFFDFFFF)
#define USART1_PCLK_DI()	(RCC->APB2ENR = RCC->APB2ENR & 0xFFFFFFEF)
#define USART6_PCLK_DI()	(RCC->APB2ENR = RCC->APB2ENR & 0xFFFFFFDF)

/*
 * Clock Reset Macros for SYSCFG peripherals
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR = RCC->APB2ENR & 0xFFFFBFFF)


/*
 * Disable GPIOx Macros
 */
// need to first set it and then reset to 0 so that we can again use the peripheral in future, not permanently disable it
#define GPIOA_PCLK_RESET()		do { (RCC->AHB1RSTR = RCC->AHB1RSTR | 0x00000001); (RCC->AHB1RSTR = RCC->AHB1RSTR & 0xFFFFFFFE); } while(0)
#define GPIOB_PCLK_RESET()		do { (RCC->AHB1RSTR = RCC->AHB1RSTR | 0x00000002); (RCC->AHB1RSTR = RCC->AHB1RSTR & 0xFFFFFFFD); } while(0)
#define GPIOC_PCLK_RESET()		do { (RCC->AHB1RSTR = RCC->AHB1RSTR | 0x00000004); (RCC->AHB1RSTR = RCC->AHB1RSTR & 0xFFFFFFFB); } while(0)
#define GPIOD_PCLK_RESET()		do { (RCC->AHB1RSTR = RCC->AHB1RSTR | 0x00000008); (RCC->AHB1RSTR = RCC->AHB1RSTR & 0xFFFFFFF7); } while(0)
#define GPIOE_PCLK_RESET()		do { (RCC->AHB1RSTR = RCC->AHB1RSTR | 0x00000010); (RCC->AHB1RSTR = RCC->AHB1RSTR & 0xFFFFFFEF); } while(0)
#define GPIOH_PCLK_RESET()		do { (RCC->AHB1RSTR = RCC->AHB1RSTR | 0x00000080); (RCC->AHB1RSTR = RCC->AHB1RSTR & 0xFFFFFF7F); } while(0)


/*
 * Disable USARTx Macros
 */
#define USART1_PCLK_RESET() 	do { (RCC->APB2RSTR = RCC->APB2RSTR | 0x00000010);  (RCC->APB2RSTR = RCC->APB2RSTR & 0xFFFFFFEF); } while(0)
#define USART2_PCLK_RESET() 	do { (RCC->APB1RSTR = RCC->APB1RSTR | 0x00020000);  (RCC->APB1RSTR = RCC->APB1RSTR & 0xFFFDFFFF); } while(0)
#define USART6_PCLK_RESET() 	do { (RCC->APB2RSTR = RCC->APB2RSTR | 0x00000020);  (RCC->APB2RSTR = RCC->APB2RSTR & 0xFFFFFFDF); } while(0)

/*
 *  IRQ (Interrupt Request) Numbers for STM32F40Xx MCU - from vector table
 */

// EXTI lines are used by GPIO peripheral to raise interrupt in NVIC
#define IRQ_NO_EXTI0			6
#define IRQ_NO_EXTI1			7
#define IRQ_NO_EXTI2			8
#define IRQ_NO_EXTI3			9
#define IRQ_NO_EXTI4			10
#define IRQ_NO_EXTI9_5			23
#define IRQ_NO_EXTI10_15		40

// IRQ nos for SPI peripheral
#define IRQ_NO_SPI3				51
#define IRQ_NO_SPI2				36
#define IRQ_NO_SPI1				35
#define IRQ_NO_SPI4				84

// IRQ nos for I2C peripheral
#define IRQ_NO_I2C1_EV			31
#define IRQ_NO_I2C1_ER			32

#define IRQ_NO_I2C2_EV			33
#define IRQ_NO_I2C2_ER			34

#define IRQ_NO_I2C3_EV			72
#define IRQ_NO_I2C3_ER			73

/*
 * @IRQ_PRIORITY_VALUES
 * Macros for all possible priority levels (0-15 since 4 bits can be set in the register)
 */

#define NVIC_IRQ_PRIO0			0
#define NVIC_IRQ_PRIO1			1
#define NVIC_IRQ_PRIO2			2
#define NVIC_IRQ_PRIO3			3
#define NVIC_IRQ_PRIO4			4
#define NVIC_IRQ_PRIO5			5
#define NVIC_IRQ_PRIO6			6
#define NVIC_IRQ_PRIO7			7
#define NVIC_IRQ_PRIO8			8
#define NVIC_IRQ_PRIO9			9
#define NVIC_IRQ_PRIO10			10
#define NVIC_IRQ_PRIO11			11
#define NVIC_IRQ_PRIO12			12
#define NVIC_IRQ_PRIO13			13
#define NVIC_IRQ_PRIO14			14
#define NVIC_IRQ_PRIO15			15

// Generic Macros
#define ENABLE					1
#define DISABLE					0
#define SET						ENABLE
#define RESET					DISABLE
#define GPIO_PIN_SET 			SET
#define GPIO_PIN_RESET 			RESET
#define FLAG_RESET				RESET
#define FLAG_SET				SET

/*******************************************************************************************
* Bit position definitions of the SPI peripheral
********************************************************************************************/

/*
 * For CR1 Register
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RX_ONLY		10
#define SPI_CR1_DFF			11
#define SPI_CR1_BIDI_OE		14
#define SPI_CR1_BIDI_MODE	15

/*
 * For CR2 Register
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7

/*
 * For SPI Register
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8


/*******************************************************************************************
* Bit position definitions of the I2C peripheral
********************************************************************************************/

/*
 * For CR1 Register
 */
#define I2C_CR1_PE			0
#define I2C_CR1_NOSTRETCH	7
#define I2C_CR1_START		8
#define I2C_CR1_STOP		9
#define I2C_CR1_ACK			10
#define I2C_CR1_SWRST		15

/*
 * For CR2 Register
 */
#define I2C_CR2_FREQ		0
#define I2C_CR2_ITERREN		8
#define I2C_CR2_ITEVTEN		9
#define I2C_CR2_ITBUFEN		10

/*
 * For SR1 Register
 */
#define I2C_SR1_SB			0
#define I2C_SR1_ADDR		1
#define I2C_SR1_BTF			2
#define I2C_SR1_STOPF		4   //for slave mode
#define I2C_SR1_RXNE		6
#define I2C_SR1_TXE			7
#define I2C_SR1_BERR		8
#define I2C_SR1_ARLO		9
#define I2C_SR1_AF			10
#define I2C_SR1_OVR			11
#define I2C_SR1_TIMEOUT		14

/*
 * For SR2 Register
 */

#define I2C_SR2_MSL			0
#define I2C_SR2_BUSY		1
#define I2C_SR2_TRA			2
#define I2C_SR2_GENCALL		4
#define I2C_SR2_DUALF		7

/*
 * For CCR Register
 */
#define I2C_CCR_CCR			0
#define I2C_CCR_DUTY		14
#define I2C_CR_FS			15

/*
 * Enable or Disable I2C Repeated Start conditions
 */
#define I2C_DISABLE_SR		RESET
#define I2C_ENABLE_SR		SET


/*******************************************************************************************
* Bit position definitions of the USART peripheral
********************************************************************************************/

/*
 * For SR Register
 */
#define USART_SR_PE			0
#define USART_SR_FE			1
#define USART_SR_NF			2
#define USART_SR_ORE		3
#define USART_SR_IDLE		4
#define USART_SR_RXNE		5
#define USART_SR_TC			6
#define USART_SR_TX			7
#define USART_SR_LBD		8
#define USART_SR_CTS		9


/*
 * For BRR Register
 */
#define USART_BRR_DIV_FRAC	0
#define USART_BRR_MANTISSA	4

/*
 * For CR1 Register
 */
#define USART_CR1_SBK		0
#define USART_CR1_RWU		1
#define USART_CR1_RE		2
#define USART_CR1_TE		3
#define USART_CR1_IDLEIE	4
#define USART_CR1_RXNEIE	5
#define USART_CR1_TCIE		6
#define USART_CR1_TXEIE		7
#define USART_CR1_PEIE		8
#define USART_CR1_PS		9
#define USART_CR1_PCE		10
#define USART_CR1_WAKE		11
#define USART_CR1_M			12
#define USART_CR1_UE		13
#define USART_CR1_OVER8		15


/*
 * For CR2 Register
 */
#define USART_CR2_ADD		0
#define USART_CR2_LBDL		5
#define USART_CR2_LBDIE		6
#define USART_CR2_LBCL		8
#define USART_CR2_CPHA		9
#define USART_CR2_CPOL		10
#define USART_CR2_CLKEN		11
#define USART_CR2_STOP		12
#define USART_CR2_LINEN		14

/*
 * For CR3 Register
 */
#define USART_CR3_EIE		0
#define USART_CR3_RTSE		8
#define USART_CR3_CTSE		9
#define USART_CR3_CTSIE		10




#include "stm32f401RE_gpio_driver.h"
#include "stm32f401RE_spi_driver.h"
#include "stm32f401RE_i2c_driver.h"
#include "stm32f401RE_usart_driver.h"
#include "stm32f401RE_rcc_driver.h"

#endif /* INC_STM32F401RE_H_ */
