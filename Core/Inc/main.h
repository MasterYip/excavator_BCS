/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "usart.h"
#include "ExcavatorHeaderFile.h"
	
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PWM01_Pin GPIO_PIN_0
#define PWM01_GPIO_Port GPIOA
#define PWM02_Pin GPIO_PIN_1
#define PWM02_GPIO_Port GPIOA
#define PWM05_Pin GPIO_PIN_6
#define PWM05_GPIO_Port GPIOA
#define PWM06_Pin GPIO_PIN_7
#define PWM06_GPIO_Port GPIOA
#define PWM07_Pin GPIO_PIN_0
#define PWM07_GPIO_Port GPIOB
#define PWM08_Pin GPIO_PIN_1
#define PWM08_GPIO_Port GPIOB
#define PWM13_Pin GPIO_PIN_6
#define PWM13_GPIO_Port GPIOC
#define PWM14_Pin GPIO_PIN_7
#define PWM14_GPIO_Port GPIOC
#define PWM15_Pin GPIO_PIN_8
#define PWM15_GPIO_Port GPIOC
#define PWM16_Pin GPIO_PIN_9
#define PWM16_GPIO_Port GPIOC
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define CS_IMU1_Pin GPIO_PIN_15
#define CS_IMU1_GPIO_Port GPIOA
#define CS_IMU2_Pin GPIO_PIN_10
#define CS_IMU2_GPIO_Port GPIOC
#define ICM20602_SCK_Pin GPIO_PIN_12
#define ICM20602_SCK_GPIO_Port GPIOC
#define CS_IMU4_Pin GPIO_PIN_2
#define CS_IMU4_GPIO_Port GPIOD
#define ICM20602_MOSI_Pin GPIO_PIN_3
#define ICM20602_MOSI_GPIO_Port GPIOB
#define CS_IMU3_Pin GPIO_PIN_4
#define CS_IMU3_GPIO_Port GPIOB
#define ICM20602_MISO_Pin GPIO_PIN_5
#define ICM20602_MISO_GPIO_Port GPIOB
#define PWM09_Pin GPIO_PIN_6
#define PWM09_GPIO_Port GPIOB
#define PWM10_Pin GPIO_PIN_7
#define PWM10_GPIO_Port GPIOB
#define PWM11_Pin GPIO_PIN_8
#define PWM11_GPIO_Port GPIOB
#define PWM12_Pin GPIO_PIN_9
#define PWM12_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
