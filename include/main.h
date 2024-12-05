/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define PWM_F_R_Pin GPIO_PIN_0
#define PWM_F_R_GPIO_Port GPIOA
#define PWM_F_L_Pin GPIO_PIN_1
#define PWM_F_L_GPIO_Port GPIOA
#define PWM_B_R_Pin GPIO_PIN_2
#define PWM_B_R_GPIO_Port GPIOA
#define PWM_B_L_Pin GPIO_PIN_3
#define PWM_B_L_GPIO_Port GPIOA
#define ADC_LEFT_Pin GPIO_PIN_0
#define ADC_LEFT_GPIO_Port GPIOB
#define ADC_RIGHT_Pin GPIO_PIN_1
#define ADC_RIGHT_GPIO_Port GPIOB
#define CAN_S_Pin GPIO_PIN_10
#define CAN_S_GPIO_Port GPIOA
#define LEFT_BRAKE_Pin GPIO_PIN_3
#define LEFT_BRAKE_GPIO_Port GPIOB
#define RIGHT_BRAKE_Pin GPIO_PIN_4
#define RIGHT_BRAKE_GPIO_Port GPIOB
#define ENABLE_FRONT_Pin GPIO_PIN_5
#define ENABLE_FRONT_GPIO_Port GPIOB
#define ENABLE_BACK_Pin GPIO_PIN_6
#define ENABLE_BACK_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
