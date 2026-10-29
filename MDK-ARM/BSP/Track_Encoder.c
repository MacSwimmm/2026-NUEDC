#include "Track_Encoder.h"
#include "tim.h"

/* F4-MIX/bsp/bsp_encoder.c 原计数与方向换算；只适配了定时器实例。 */
volatile int32_t g_encoderA_cnt = 0;
volatile int32_t g_encoderB_cnt = 0;

void bsp_encoder_init(void)
{
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);
    if ((HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL) != HAL_OK) ||
        (HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL) != HAL_OK)) {
        Error_Handler();
    }
    g_encoderA_cnt = 0;
    g_encoderB_cnt = 0;
}

void bsp_encoder_update(void)
{
    int16_t count_a = (int16_t)__HAL_TIM_GET_COUNTER(&htim8);
    int16_t count_b = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    __HAL_TIM_SET_COUNTER(&htim4, 0U);

    /* F4-MIX 原方向：物理前进反馈为原始编码器增量取反。 */
    g_encoderA_cnt = -(int32_t)count_a;
    g_encoderB_cnt = -(int32_t)count_b;
}
