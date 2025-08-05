/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32g0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32g0xx_it.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "globals.h" // global variables

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */


uint16_t k_slowIT = 0;

volatile uint16_t risingEdge = 0;
volatile uint16_t fallingEdge = 0;

uint8_t iADCchannels = 0;
uint16_t currentMeasurement = 0;
uint16_t weightedOldValue = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern ADC_HandleTypeDef hadc1;
extern DAC_HandleTypeDef hdac1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim6;
extern TIM_HandleTypeDef htim16;
extern UART_HandleTypeDef huart2;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32G0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel 1 interrupt.
  */
void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */


  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

	// All ADC channels transferred via DMA
  for (iADCchannels = 0; iADCchannels < numberADCchannels; iADCchannels++) {
	  currentMeasurement = adc_DMA[iADCchannels]>>2; // divide current measurement value by 4
	  weightedOldValue = (adc_buffer[iADCchannels][0]*3)>>2; // multiply with 3/4
	  adc_buffer[iADCchannels][1] = currentMeasurement + weightedOldValue; // new averaged value
	  adc_buffer[iADCchannels][0] = adc_buffer[iADCchannels][1]; // transfer value from current to old
  }

	adc_24V = adc_buffer[0][1]; // 4095 = 36.3 V
	adc_tempMOSFET = adc_buffer[1][1]; // 2482 = 2V = 90°C
	adc_uSenseLamp = adc_buffer[2][1];
	adc_iSenseLamp = adc_buffer[3][1];
	adc_lampIntensity = adc_buffer[4][1];
	adc_iSenseIn = adc_buffer[5][1]; // 2707 = 24V,  4095 = 0.825 A

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
  * @brief This function handles ADC1, COMP1 and COMP2 interrupts (COMP interrupts through EXTI lines 17 and 18).
  */
void ADC1_COMP_IRQHandler(void)
{
  /* USER CODE BEGIN ADC1_COMP_IRQn 0 */

  /* USER CODE END ADC1_COMP_IRQn 0 */
  HAL_ADC_IRQHandler(&hadc1);
  /* USER CODE BEGIN ADC1_COMP_IRQn 1 */

  /* USER CODE END ADC1_COMP_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
  //risingEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
  //fallingEdge = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);


  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM6, DAC1 and LPTIM1 interrupts (LPTIM1 interrupt through EXTI line 29).
  */
void TIM6_DAC_LPTIM1_IRQHandler(void)
{
  /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 0 */

  /* USER CODE END TIM6_DAC_LPTIM1_IRQn 0 */
  HAL_TIM_IRQHandler(&htim6);
  HAL_DAC_IRQHandler(&hdac1);
  /* USER CODE BEGIN TIM6_DAC_LPTIM1_IRQn 1 */

  // 1 kHz interrupt
  tim6_irq_request = 1;


  // 1 Hz interrupt
  if (k_slowIT <1000) {
	  k_slowIT++;
  }
  else if (k_slowIT >999) {
	  k_slowIT = 0;
	  tim6_slowIrq_request = 1;

  }


  /* USER CODE END TIM6_DAC_LPTIM1_IRQn 1 */
}

/**
  * @brief This function handles TIM16 global interrupt.
  */
void TIM16_IRQHandler(void)
{
  /* USER CODE BEGIN TIM16_IRQn 0 */

  /* USER CODE END TIM16_IRQn 0 */
  HAL_TIM_IRQHandler(&htim16);
  /* USER CODE BEGIN TIM16_IRQn 1 */

  /* USER CODE END TIM16_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */



  /* USER CODE END USART2_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
