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
#include "stm32h7xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define UART_LEG2_Pin GPIO_PIN_2
#define UART_LEG2_GPIO_Port GPIOA
#define ADC_BAT_Pin GPIO_PIN_4
#define ADC_BAT_GPIO_Port GPIOC
#define ADC_CHARGE_Pin GPIO_PIN_1
#define ADC_CHARGE_GPIO_Port GPIOB
#define Backside_Pin GPIO_PIN_2
#define Backside_GPIO_Port GPIOB
#define UART_LEG3_Pin GPIO_PIN_10
#define UART_LEG3_GPIO_Port GPIOB
#define UART_HEAD_Pin GPIO_PIN_13
#define UART_HEAD_GPIO_Port GPIOB
#define UART_LEG1_Pin GPIO_PIN_14
#define UART_LEG1_GPIO_Port GPIOB
#define Power_out_Pin GPIO_PIN_8
#define Power_out_GPIO_Port GPIOD
#define Power_in_Pin GPIO_PIN_9
#define Power_in_GPIO_Port GPIOD
#define LED_Pin GPIO_PIN_11
#define LED_GPIO_Port GPIOD
#define TOUCH1_MOUTH_Pin GPIO_PIN_11
#define TOUCH1_MOUTH_GPIO_Port GPIOC
#define upperComputerPower_5V_Pin GPIO_PIN_0
#define upperComputerPower_5V_GPIO_Port GPIOD
#define TOUCH0_HEAD_Pin GPIO_PIN_2
#define TOUCH0_HEAD_GPIO_Port GPIOD
#define Servo_Power_12V_Pin GPIO_PIN_4
#define Servo_Power_12V_GPIO_Port GPIOD
#define Abdomen_Pin GPIO_PIN_5
#define Abdomen_GPIO_Port GPIOD
#define BUZZER_Pin GPIO_PIN_6
#define BUZZER_GPIO_Port GPIOD
#define FAN_Pin GPIO_PIN_7
#define FAN_GPIO_Port GPIOD
#define MPU_SCL_Pin GPIO_PIN_6
#define MPU_SCL_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define upperComputerPower_PIN GPIO_PIN_0
#define upperComputerPower_GPIO_Port GPIOD
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
