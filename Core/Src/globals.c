/*
 * globals.c
 *
 *  Created on: May 14, 2025
 *      Author: Leon Fauth
 */

#include "globals.h"

volatile uint8_t enableFlag = 0;
volatile uint8_t supplyOKFlag = 0;
volatile uint8_t errorFlag = 0;
volatile uint8_t OT_flag = 0;
volatile uint8_t OCPinFlag = 0;

volatile uint8_t tim6_irq_request = 0;
volatile uint8_t tim6_slowIrq_request = 0;

// volatile uint16_t adc_buffer[6] = {0, 0, 0, 0, 0, 0}; // 6 ADC channels
volatile uint16_t adc_DMA[6] = {0, 0, 0, 0, 0, 0}; // 6 ADC channels
//volatile uint16_t adc_buffer[6][2] = {{0,0}, {0,0}, {0,0}, {0,0}, {0,0}, {0,0}}; // 6 ADC channels, Old and New value
volatile uint16_t adc_buffer[6] = {0,0,0,0,0,0}; // 6 ADC channels, Old and New value



volatile uint16_t adc_24V = 0;
volatile uint16_t adc_tempMOSFET = 0;
volatile uint16_t adc_uSenseLamp = 0;
volatile uint16_t adc_iSenseLamp = 0;
volatile uint16_t adc_lampIntensity = 0;
volatile uint16_t adc_iSenseIn = 0;

volatile uint16_t dutyControl = 0;

volatile uint8_t numberADCchannels = 6;
