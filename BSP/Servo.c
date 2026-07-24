#include "Servo.h"

#include "bsp_config.h"
#include "ti_msp_dl_config.h"

static uint16_t g_servo_pulse[2] = {
    SERVO_NEUTRAL_PULSE_US, SERVO_NEUTRAL_PULSE_US
};

static uint16_t ClampPulse(uint16_t pulse_us)
{
    if (pulse_us < SERVO_MIN_PULSE_US) return SERVO_MIN_PULSE_US;
    if (pulse_us > SERVO_MAX_PULSE_US) return SERVO_MAX_PULSE_US;
    return pulse_us;
}

void Servo_Init(void)
{
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
