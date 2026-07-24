#ifndef G3507_BASE_MOTOR_H
#define G3507_BASE_MOTOR_H

#include <stdint.h>

/*
 * TB6612 双电机驱动接口。
 * 正负号表示方向，绝对值表示 PWM 计数；STBY 已硬件接 3.3 V，不占用 MCU 引脚。
 */
void Motor_Init(void);
void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm);
void Motor_Stop(void);
int32_t Motor_GetLeftPWM(void);
int32_t Motor_GetRightPWM(void);

#endif
