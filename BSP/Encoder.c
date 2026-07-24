#include "Encoder.h"

#include "ti_msp_dl_config.h"

static volatile int32_t g_left_count = 0;
static volatile int32_t g_right_count = 0;

void Encoder_Init(void)
{
    g_left_count = 0;
    g_right_count = 0;

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);
}

/**
 * @brief 原子读取本控制周期的左右编码器计数并清零。
 *
 * GPIO 中断可能在任意时刻修改计数，因此“读取+清零”期间短暂关闭全局中断，
 * 避免边沿刚好落在两条语句之间而丢失。
 */
void Encoder_ReadAndReset(int32_t *left_count, int32_t *right_count)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    *left_count = g_left_count;
    *right_count = g_right_count;
    g_left_count = 0;
    g_right_count = 0;
    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief GPIOA/GPIOB 共用的 GROUP1 中断入口，软件完成两套 A/B 正交解码。
 *
 * 当前只统计 A、B 两相的上升沿，所以一圈计数倍率为 2。方向判断方法是：
 * 某一相出现上升沿时读取另一相电平，根据两相先后关系决定加一或减一。
 */
void GROUP1_IRQHandler(void)
{
    uint32_t left_status = DL_GPIO_getEnabledInterruptStatus(
        ENCODERA_PORT, ENCODERA_E1A_PIN | ENCODERA_E1B_PIN);
    uint32_t right_status = DL_GPIO_getEnabledInterruptStatus(
        ENCODERB_PORT, ENCODERB_E2A_PIN | ENCODERB_E2B_PIN);

    if ((left_status & ENCODERA_E1A_PIN) != 0U) {
        g_left_count +=
            (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1B_PIN) != 0U) ? 1 : -1;
    }
    else if ((left_status & ENCODERA_E1B_PIN) != 0U) {
        g_left_count +=
            (DL_GPIO_readPins(ENCODERA_PORT, ENCODERA_E1A_PIN) == 0U) ? 1 : -1;
    }

    if ((right_status & ENCODERB_E2A_PIN) != 0U) {
        g_right_count +=
            (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2B_PIN) != 0U) ? 1 : -1;
    }
    else if ((right_status & ENCODERB_E2B_PIN) != 0U) {
        g_right_count +=
            (DL_GPIO_readPins(ENCODERB_PORT, ENCODERB_E2A_PIN) == 0U) ? 1 : -1;
    }

    DL_GPIO_clearInterruptStatus(ENCODERA_PORT, left_status);
    DL_GPIO_clearInterruptStatus(ENCODERB_PORT, right_status);
}
