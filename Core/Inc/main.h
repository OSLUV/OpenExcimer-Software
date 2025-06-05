/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MCU_Remote_Pin GPIO_PIN_9
#define MCU_Remote_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOC
#define _24V_Sense_Pin GPIO_PIN_0
#define _24V_Sense_GPIO_Port GPIOA
#define temp_MOSFET_Pin GPIO_PIN_1
#define temp_MOSFET_GPIO_Port GPIOA
#define Usense_Lamp_Pin GPIO_PIN_3
#define Usense_Lamp_GPIO_Port GPIOA
#define Ilevel_DAC_Pin GPIO_PIN_4
#define Ilevel_DAC_GPIO_Port GPIOA
#define DAC2_Pin GPIO_PIN_5
#define DAC2_GPIO_Port GPIOA
#define Status_Pin GPIO_PIN_7
#define Status_GPIO_Port GPIOA
#define IsenseLamp_Pin GPIO_PIN_0
#define IsenseLamp_GPIO_Port GPIOB
#define LampIntensity_Pin GPIO_PIN_1
#define LampIntensity_GPIO_Port GPIOB
#define Isense_In_Pin GPIO_PIN_2
#define Isense_In_GPIO_Port GPIOB
#define DRV_Pin GPIO_PIN_8
#define DRV_GPIO_Port GPIOA
#define GPIO_IN_ALIVE_Pin GPIO_PIN_9
#define GPIO_IN_ALIVE_GPIO_Port GPIOA
#define PWM_out_ALIVE_Pin GPIO_PIN_6
#define PWM_out_ALIVE_GPIO_Port GPIOC
#define Isense_MOS_Pin GPIO_PIN_4
#define Isense_MOS_GPIO_Port GPIOB
#define DRV_Mask_Pin GPIO_PIN_5
#define DRV_Mask_GPIO_Port GPIOB
#define UV_LED_Pin GPIO_PIN_6
#define UV_LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
