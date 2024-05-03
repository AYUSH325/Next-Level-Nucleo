/*
 * stm32f401RE_rcc_driver.c
 *
 *  Created on: Apr 23, 2024
 *      Author: ayushganguly
 */

#include "stm32f401RE_rcc_driver.h"

uint16_t ahbPrescalars[8] = { 2, 4, 8, 16, 64, 128, 256, 512 };
uint8_t apbPrescalars[4] = { 2, 4, 8, 16 };

/*
 * @param: APB1 or APB2 bus
 * returns the peripheral clock frequency of APB1 or APB2 by checking
 * which clock is used and dividing with the prescalars
 */

uint32_t RCC_GetPCLKValue(uint8_t bus) {
	uint32_t pclk, SystemClk;
	uint8_t clkSrc = (RCC->CFGR >> 2) & 0x3; //read only the SWS bits

	uint8_t temp, ahbPrescalar, apbxPrescalar;

	if (clkSrc == 0) {
		//HSI
		SystemClk = 16000000;
	} else if (clkSrc == 1) {
		// HSE
		SystemClk = 8000000; // 8MHz provided by MCO oscillator of ST debug board from the nucleo board datasheet
	} else if (clkSrc == 2) {
		// PLL
	}

	// read the value of AHB prescalar from HPRE in CFGR
	temp = (RCC->CFGR >> 4) & 0xF;

	if (temp < 8) {
		// systemclk not divided
		ahbPrescalar = 1;
	} else {
		ahbPrescalar = ahbPrescalars[temp - 8];
	}
	temp = 0;


	if (bus == APB1)
	{
		// read the value of APB1 prescalar from PPRE1 in CFGR
		temp = (RCC->CFGR >> 10) & 0x7;
	}
	else if (bus == APB2)
	{
		// read the value of APB2 prescalar from PPRE2 in CFGR
		temp = (RCC->CFGR >> 13) & 0x7;
	}
	if (temp < 4) {
		// systemclk not divided
		apbxPrescalar = 1;
	} else {
		apbxPrescalar = apbPrescalars[temp - 4];
	}

	pclk = (SystemClk / ahbPrescalar) / apbxPrescalar;

	return pclk;
}
