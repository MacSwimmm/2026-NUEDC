#include "Motor.h"

#include "bsp_config.h"
#include "ti_msp_dl_config.h"

static volatile int32_t g_left_pwm = 0;
static volatile int32_t g_right_pwm = 0;

static int32_t ClampPWM(int32_t pwm)
{
    if (pwm > MOTOR_PWM_LIMIT_COUNTS) return MOTOR_PWM_LIMIT_COUNTS;
    if (pwm < -MOTOR_PWM_LIMIT_COUNTS) return -MOTOR_PWM_LIMIT_COUNTS;
    return pwm;
}

/**
 * @brief 将一路带符号 PWM 写入 TB6612。
 * @param pwm            正数正转、负数反转、0 停车
 * @param direction_port AIN/BIN 所在 GPIO 端口
 * @param pin1/pin2      对应的两根方向引脚
 * @param compare_index  TIMA1 的 C0 或 C1 比较通道
 */
static void ApplyOneMotor(int32_t pwm, GPIO_Regs *direction_port,
                          uint32_t pin1, uint32_t pin2, uint32_t compare_index)
{
    uint32_t magnitude = (uint32_t)((pwm < 0) ? -pwm : pwm);

    if (pwm > 0) {
        DL_GPIO_clearPins(direction_port, pin1);
        DL_GPIO_setPins(direction_port, pin2);
    } else if (pwm < 0) {
        DL_GPIO_setPins(direction_port, pin1);
        DL_GPIO_clearPins(direction_port, pin2);
    } else {
        /* 两个方向脚同时为低，配合 PWM=0 保持确定的停车状态。 */
        DL_GPIO_clearPins(direction_port, pin1 | pin2);
    }

    DL_Timer_setCaptureCompareValue(PWM_0_INST, magnitude, compare_index);
}

void Motor_Init(void)
{
    Motor_Stop();
}

void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm)
{
    g_left_pwm = ClampPWM(left_pwm);
    g_right_pwm = ClampPWM(right_pwm);

    ApplyOneMotor(g_left_pwm, AIN_PORT, AIN_AIN1_PIN, AIN_AIN2_PIN,
                  GPIO_PWM_0_C0_IDX);
    ApplyOneMotor(g_right_pwm, BIN_PORT, BIN_BIN1_PIN, BIN_BIN2_PIN,
                  GPIO_PWM_0_C1_IDX);
}

void Motor_Stop(void)
{
    Motor_SetPWM(0, 0);
}

int32_t Motor_GetLeftPWM(void)
{
    return g_left_pwm;
}

int32_t Motor_GetRightPWM(void)
{
    return g_right_pwm;
}
