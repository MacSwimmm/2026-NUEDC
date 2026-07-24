#include "Actuator.h"

#include "ti_msp_dl_config.h"

#define MAGNET_PWM_PERIOD_COUNTS     4000U
#define MECHANISM_PWM_PERIOD_COUNTS  8000U

static uint16_t ClampU16(uint16_t value, uint16_t maximum)
{
    return (value > maximum) ? maximum : value;
}

void Actuator_Init(void)
{
    /* 所有扩展执行器都从明确的关闭状态启动。 */
    Actuator_SetMagnetPermille(0U);
    Actuator_SetMechanismPWM(0U, 0U);
    Actuator_SetBuzzer(false);
    Actuator_SetGeneralEnable(false);
}

/**
 * @brief 设置 PA2 磁铁/功率模块 PWM，占空比单位为千分比 0~1000。
 * @note  PA2 只能驱动外部 MOS 管或驱动模块，不能直接给电磁铁供电。
 */
void Actuator_SetMagnetPermille(uint16_t duty_permille)
{
    uint32_t compare;

    if (duty_permille > 1000U) duty_permille = 1000U;
    compare = ((uint32_t)MAGNET_PWM_PERIOD_COUNTS * duty_permille) / 1000U;
    DL_Timer_setCaptureCompareValue(MAGNET_PWM_INST, compare,
                                    GPIO_MAGNET_PWM_C1_IDX);
}

void Actuator_SetMechanismPWM(uint16_t channel0, uint16_t channel1)
{
    DL_Timer_setCaptureCompareValue(
        MECHANISM_PWM_INST,
        ClampU16(channel0, MECHANISM_PWM_PERIOD_COUNTS),
        GPIO_MECHANISM_PWM_C0_IDX);
    DL_Timer_setCaptureCompareValue(
        MECHANISM_PWM_INST,
        ClampU16(channel1, MECHANISM_PWM_PERIOD_COUNTS),
        GPIO_MECHANISM_PWM_C1_IDX);
}

void Actuator_SetBuzzer(bool enabled)
{
    if (enabled) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_BUZZER_PIN);
    }
}

void Actuator_SetGeneralEnable(bool enabled)
{
    if (enabled) {
        DL_GPIO_setPins(GENERAL_EN_PORT, GENERAL_EN_PIN_EN_PIN);
    } else {
        DL_GPIO_clearPins(GENERAL_EN_PORT, GENERAL_EN_PIN_EN_PIN);
    }
}
