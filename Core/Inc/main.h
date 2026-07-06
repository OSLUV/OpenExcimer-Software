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
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOC
#define _24V_Sense_Pin GPIO_PIN_0
#define _24V_Sense_GPIO_Port GPIOA
#define temp_MOSFET_Pin GPIO_PIN_1
#define temp_MOSFET_GPIO_Port GPIOA
#define PowerSet_Pin GPIO_PIN_5
#define PowerSet_GPIO_Port GPIOA
#define OCP_Pin GPIO_PIN_6
#define OCP_GPIO_Port GPIOA
#define Status_Pin GPIO_PIN_7
#define Status_GPIO_Port GPIOA
#define IsenseLamp_Pin GPIO_PIN_0
#define IsenseLamp_GPIO_Port GPIOB
#define Usense_Lamp_Pin GPIO_PIN_1
#define Usense_Lamp_GPIO_Port GPIOB
#define Isense_In_Pin GPIO_PIN_2
#define Isense_In_GPIO_Port GPIOB
#define DRV_Pin GPIO_PIN_8
#define DRV_GPIO_Port GPIOA
#define OCP_Reset_Pin GPIO_PIN_6
#define OCP_Reset_GPIO_Port GPIOC
#define PotiSelect_Pin GPIO_PIN_12
#define PotiSelect_GPIO_Port GPIOA
#define Isense_MOS_Pin GPIO_PIN_4
#define Isense_MOS_GPIO_Port GPIOB
#define PWM_INPUT_Pin GPIO_PIN_5
#define PWM_INPUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
