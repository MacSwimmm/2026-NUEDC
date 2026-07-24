#include "Servo.h"

#include "bsp_config.h"
#include "ti_msp_dl_config.h"

/* 保存实际限幅后的脉宽，供 OLED 系统页显示。 */
static uint16_t g_servo_pulse[2] = {
    SERVO_NEUTRAL_PULSE_US, SERVO_NEUTRAL_PULSE_US
};

static uint16_t ClampPulse(uint16_t pulse_us)
{
    /* 限幅保护舵机机械行程，实际安全范围仍应按具体舵机和机构校准。 */
    if (pulse_us < SERVO_MIN_PULSE_US) return SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) return SERVO_MAX_PULSE_US;
    return pulse_us;
}

void Servo_Init(void)
{
    /* 上电先回中位，避免沿用调试器复位前的任意比较值。 */
    Servo_SetPulseUS(1U, SERVO_NEUTRAL_PULSE_US);
    Servo_SetPulseUS(2U, SERVO_NEUTRAL_PULSE_US);
}

/**
 * @brief 设置舵机高电平脉宽。
 * @note  TIMG8 已配置为 1 us/计数，因此比较值可以直接写入微秒数。
 */
void Servo_SetPulseUS(uint8_t channel, uint16_t pulse_us)
{
    uint16_t value = ClampPulse(pulse_us);

    if (channel == 1U) {
        g_servo_pulse[0] = value;
        DL_Timer_setCaptureCompareValue(SERVO_PWM_INST, value,
                                        GPIO_SERVO_PWM_C0_IDX);
    } else if (channel == 2U) {
        g_servo_pulse[1] = value;
        DL_Timer_setCaptureCompareValue(SERVO_PWM_INST, value,
                                        GPIO_SERVO_PWM_C1_IDX);
    }
}

void Servo_SetAngle(uint8_t channel, uint8_t angle_deg)
{
    uint32_t pulse;

    /* 使用整数线性插值，避免在底层驱动引入浮点运算。 */
    if (angle_deg > 180U) angle_deg = 180U;
    pulse = SERVO_MIN_PULSE_US +
            ((uint32_t)(SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * angle_deg) / 180U;
    Servo_SetPulseUS(channel, (uint16_t)pulse);
}

uint16_t Servo_GetPulseUS(uint8_t channel)
{
    if (channel == 1U) return g_servo_pulse[0];
    if (channel == 2U) return g_servo_pulse[1];
    return 0U;
}
