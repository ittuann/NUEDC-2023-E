/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef unsigned char bool_t;
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY1_EXTI_IRQn EXTI3_IRQn
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define KEY0_EXTI_IRQn EXTI4_IRQn
#define MATRIX_OUT1_Pin GPIO_PIN_0
#define MATRIX_OUT1_GPIO_Port GPIOF
#define MATRIX_OUT2_Pin GPIO_PIN_1
#define MATRIX_OUT2_GPIO_Port GPIOF
#define MATRIX_OUT3_Pin GPIO_PIN_2
#define MATRIX_OUT3_GPIO_Port GPIOF
#define MATRIX_IN1_Pin GPIO_PIN_3
#define MATRIX_IN1_GPIO_Port GPIOF
#define MATRIX_IN2_Pin GPIO_PIN_4
#define MATRIX_IN2_GPIO_Port GPIOF
#define MATRIX_IN3_Pin GPIO_PIN_5
#define MATRIX_IN3_GPIO_Port GPIOF
#define MATRIX_OUT4_Pin GPIO_PIN_6
#define MATRIX_OUT4_GPIO_Port GPIOF
#define MATRIX_IN4_Pin GPIO_PIN_7
#define MATRIX_IN4_GPIO_Port GPIOF
#define SWITCH_Pin GPIO_PIN_8
#define SWITCH_GPIO_Port GPIOF
#define LED0R_Pin GPIO_PIN_9
#define LED0R_GPIO_Port GPIOF
#define LED1G_Pin GPIO_PIN_10
#define LED1G_GPIO_Port GPIOF
#define PWM_X_Pin GPIO_PIN_9
#define PWM_X_GPIO_Port GPIOE
#define PWM_Y_Pin GPIO_PIN_11
#define PWM_Y_GPIO_Port GPIOE
#define NORFLASH_SPICS_Pin GPIO_PIN_14
#define NORFLASH_SPICS_GPIO_Port GPIOB
#define LASER_Pin GPIO_PIN_8
#define LASER_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
