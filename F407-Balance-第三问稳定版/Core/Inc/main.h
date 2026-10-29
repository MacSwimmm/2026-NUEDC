/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define START_BUTTON_Pin GPIO_PIN_0
#define START_BUTTON_GPIO_Port GPIOA
#define LED_BOARD_Pin GPIO_PIN_1
#define LED_BOARD_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* PE2/PE3 两键菜单：全部内部上拉、低电平按下。 */
#define TARGET_LOCK_BUTTON_Pin GPIO_PIN_2
#define TARGET_LOCK_BUTTON_GPIO_Port GPIOE

/* F4-MIX 循迹底盘新增引脚；原 PA0/PA1 以及 OLED 的 PB6/PB7 均保持原用途。 */
#define TRACK_MOTOR_AIN1_Pin GPIO_PIN_0
#define TRACK_MOTOR_AIN1_GPIO_Port GPIOC
#define TRACK_MOTOR_AIN2_Pin GPIO_PIN_1
#define TRACK_MOTOR_AIN2_GPIO_Port GPIOC
#define TRACK_MOTOR_BIN1_Pin GPIO_PIN_2
#define TRACK_MOTOR_BIN1_GPIO_Port GPIOC
#define TRACK_MOTOR_BIN2_Pin GPIO_PIN_3
#define TRACK_MOTOR_BIN2_GPIO_Port GPIOC

#define TRACK_ENCODER_A_CH1_Pin GPIO_PIN_6
#define TRACK_ENCODER_A_CH1_GPIO_Port GPIOC
#define TRACK_ENCODER_A_CH2_Pin GPIO_PIN_7
#define TRACK_ENCODER_A_CH2_GPIO_Port GPIOC
#define TRACK_ENCODER_B_CH1_Pin GPIO_PIN_12
#define TRACK_ENCODER_B_CH1_GPIO_Port GPIOD
#define TRACK_ENCODER_B_CH2_Pin GPIO_PIN_13
#define TRACK_ENCODER_B_CH2_GPIO_Port GPIOD

#define TRACK_C1_Pin GPIO_PIN_0
#define TRACK_C1_GPIO_Port GPIOD
#define TRACK_C2_Pin GPIO_PIN_1
#define TRACK_C2_GPIO_Port GPIOD
#define TRACK_C3_Pin GPIO_PIN_2
#define TRACK_C3_GPIO_Port GPIOD
#define TRACK_C4_Pin GPIO_PIN_3
#define TRACK_C4_GPIO_Port GPIOD
#define TRACK_C5_Pin GPIO_PIN_4
#define TRACK_C5_GPIO_Port GPIOD
#define TRACK_C6_Pin GPIO_PIN_5
#define TRACK_C6_GPIO_Port GPIOD
#define TRACK_C7_Pin GPIO_PIN_6
#define TRACK_C7_GPIO_Port GPIOD
#define TRACK_C8_Pin GPIO_PIN_7
#define TRACK_C8_GPIO_Port GPIOD

#define MENU_NEXT_BUTTON_Pin GPIO_PIN_3
#define MENU_NEXT_BUTTON_GPIO_Port GPIOE

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
