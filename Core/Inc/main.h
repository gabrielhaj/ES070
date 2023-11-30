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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "encoder.h"
#include "motors.h"
#include <lineFollower.h>
#include "lineSensors.h"
#include "buttons.h"
#include "odometry.h"
#include "pid.h"
#include "lcd.h"
#include "encoder.h"
#include <stdio.h>
#include <string.h>

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
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define LeftMotorPWM_Pin GPIO_PIN_0
#define LeftMotorPWM_GPIO_Port GPIOC
#define RightMotorPWM_Pin GPIO_PIN_1
#define RightMotorPWM_GPIO_Port GPIOC
#define LightSensor1_Pin GPIO_PIN_0
#define LightSensor1_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LightSensor2_Pin GPIO_PIN_6
#define LightSensor2_GPIO_Port GPIOA
#define EnterBt_Pin GPIO_PIN_5
#define EnterBt_GPIO_Port GPIOC
#define EnterBt_EXTI_IRQn EXTI9_5_IRQn
#define LeftMotorIn1_Pin GPIO_PIN_12
#define LeftMotorIn1_GPIO_Port GPIOB
#define LightSensor3_Pin GPIO_PIN_13
#define LightSensor3_GPIO_Port GPIOB
#define LightSensor4_Pin GPIO_PIN_15
#define LightSensor4_GPIO_Port GPIOB
#define DownBt_Pin GPIO_PIN_7
#define DownBt_GPIO_Port GPIOC
#define DownBt_EXTI_IRQn EXTI9_5_IRQn
#define LeftBt_Pin GPIO_PIN_8
#define LeftBt_GPIO_Port GPIOC
#define LeftBt_EXTI_IRQn EXTI9_5_IRQn
#define RightBt_Pin GPIO_PIN_9
#define RightBt_GPIO_Port GPIOC
#define RightBt_EXTI_IRQn EXTI9_5_IRQn
#define LightSensor5_Pin GPIO_PIN_9
#define LightSensor5_GPIO_Port GPIOA
#define RightMotorIn4_Pin GPIO_PIN_10
#define RightMotorIn4_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define LeftWheelEnc_Pin GPIO_PIN_4
#define LeftWheelEnc_GPIO_Port GPIOB
#define RightWheelEnc_Pin GPIO_PIN_5
#define RightWheelEnc_GPIO_Port GPIOB
#define UpBt_Pin GPIO_PIN_6
#define UpBt_GPIO_Port GPIOB
#define UpBt_EXTI_IRQn EXTI9_5_IRQn
#define RightMotorIn3_Pin GPIO_PIN_7
#define RightMotorIn3_GPIO_Port GPIOB
#define LeftMotorIn2_Pin GPIO_PIN_9
#define LeftMotorIn2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
