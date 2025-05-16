/*
 * globals.h
 *
 *  Created on: May 14, 2025
 *      Author: Leon Fauth
 */

#ifndef INC_GLOBALS_H_
#define INC_GLOBALS_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

extern volatile uint8_t enableFlag;
extern volatile uint8_t supplyOKFlag;
extern volatile uint8_t errorFlag;
extern volatile uint8_t OT_flag;
extern volatile uint8_t OCPinFlag;

extern volatile uint16_t adc_buffer[6]; // 6 ADC channels

extern volatile uint16_t adc_24V;
extern volatile uint16_t adc_tempMOSFET;
extern volatile uint16_t adc_uSenseLamp;
extern volatile uint16_t adc_iSenseLamp;
extern volatile uint16_t adc_lampIntensity;
extern volatile uint16_t adc_iSenseIn;

#endif /* INC_GLOBALS_H_ */
