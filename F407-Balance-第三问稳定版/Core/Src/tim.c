#include "tim.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

/*
 * 本文件只为移植的 F4-MIX 底盘提供定时器：
 *   TIM3 CH1/CH2：左右轮 PWM；
 *   TIM8 CH1/CH2：左轮编码器（由原 PA0/PA1 重映射到 PC6/PC7）；
 *   TIM4 CH1/CH2：右轮编码器（由原 PB6/PB7 重映射到 PD12/PD13）。
 * 不占用稳定版原有 PA0、PA1、PB6、PB7。
 */

static void Track_EncoderTimerInit(TIM_HandleTypeDef *htim,
                                   TIM_TypeDef *instance,
                                   uint32_t ic1_filter,
                                   uint32_t ic2_filter)
{
    TIM_Encoder_InitTypeDef encoder = {0};

    htim->Instance = instance;
    htim->Init.Prescaler = 0U;
    htim->Init.CounterMode = TIM_COUNTERMODE_UP;
    htim->Init.Period = 0xFFFFU;
    htim->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    encoder.EncoderMode = TIM_ENCODERMODE_TI12;
    encoder.IC1Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC1Prescaler = TIM_ICPSC_DIV1;
    encoder.IC1Filter = ic1_filter;
    encoder.IC2Polarity = TIM_ICPOLARITY_RISING;
    encoder.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    encoder.IC2Prescaler = TIM_ICPSC_DIV1;
    encoder.IC2Filter = ic2_filter;

    if (HAL_TIM_Encoder_Init(htim, &encoder) != HAL_OK) {
        Error_Handler();
    }
}

void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef channel = {0};

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 83U;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 99U;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
        Error_Handler();
    }

    channel.OCMode = TIM_OCMODE_PWM1;
    channel.Pulse = 0U;
    channel.OCPolarity = TIM_OCPOLARITY_HIGH;
    channel.OCFastMode = TIM_OCFAST_DISABLE;
    if ((HAL_TIM_PWM_ConfigChannel(&htim3, &channel, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_ConfigChannel(&htim3, &channel, TIM_CHANNEL_2) != HAL_OK)) {
        Error_Handler();
    }
}

void MX_TIM4_Init(void)
{
    /* F4-MIX 右轮原参数：CH1 filter=6，CH2 filter=0。 */
    Track_EncoderTimerInit(&htim4, TIM4, 6U, 0U);
}

void MX_TIM8_Init(void)
{
    /* F4-MIX 左轮原参数：CH1/CH2 filter 均为 6。 */
    Track_EncoderTimerInit(&htim8, TIM8, 6U, 6U);
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    if ((htim != NULL) && (htim->Instance == TIM3)) {
        __HAL_RCC_TIM3_CLK_ENABLE();
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    if (htim == NULL) {
        return;
    }
    if (htim->Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    } else if (htim->Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    }
}
