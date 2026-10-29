/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_BOARD_GPIO_Port, LED_BOARD_Pin, GPIO_PIN_RESET);

  /* F4-MIX TB6612 上电保持方向输入为低，避免初始化阶段产生车轮输出。 */
  HAL_GPIO_WritePin(GPIOC, TRACK_MOTOR_AIN1_Pin|TRACK_MOTOR_AIN2_Pin|
                           TRACK_MOTOR_BIN1_Pin|TRACK_MOTOR_BIN2_Pin,
                           GPIO_PIN_RESET);

  /*Configure GPIO pin : START_BUTTON_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(START_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_BOARD_Pin */
  GPIO_InitStruct.Pin = LED_BOARD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED_BOARD_GPIO_Port, &GPIO_InitStruct);

  /* TIM3_CH1/CH2：左右轮 PWM，保持 F4-MIX 的 PA6/PA7 分配。 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* TB6612 AIN1/AIN2/BIN1/BIN2。 */
  GPIO_InitStruct.Pin = TRACK_MOTOR_AIN1_Pin|TRACK_MOTOR_AIN2_Pin|
                        TRACK_MOTOR_BIN1_Pin|TRACK_MOTOR_BIN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 左轮编码器：TIM8_CH1/CH2，重映射到 PC6/PC7，避开原 PA0/PA1。 */
  GPIO_InitStruct.Pin = TRACK_ENCODER_A_CH1_Pin|TRACK_ENCODER_A_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* 八路循迹输入：PD0~PD7，内部上拉；C1 会由 BSP 临时改为高阻。 */
  GPIO_InitStruct.Pin = TRACK_C1_Pin|TRACK_C2_Pin|TRACK_C3_Pin|TRACK_C4_Pin|
                        TRACK_C5_Pin|TRACK_C6_Pin|TRACK_C7_Pin|TRACK_C8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* 右轮编码器：TIM4_CH1/CH2，重映射到 PD12/PD13，避开 PA0/PA1。 */
  GPIO_InitStruct.Pin = TRACK_ENCODER_B_CH1_Pin|TRACK_ENCODER_B_CH2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* 两键菜单：PE3=下一项，PE2=短按确认/动作、长按返回。 */
  GPIO_InitStruct.Pin = TARGET_LOCK_BUTTON_Pin|MENU_NEXT_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
