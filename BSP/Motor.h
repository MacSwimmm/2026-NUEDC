#ifndef G3507_BASE_MOTOR_H
#define G3507_BASE_MOTOR_H

#include <stdint.h>

/**
 * @file Motor.h
 * @brief TB6612 双路方向与 PWM 输出接口。
 *
 * 正负号表示方向，绝对值表示 PWM 计数；STBY 已硬件接 3.3 V，不占用 MCU 引脚。
 * 左右 PWM 分别使用 TIMA1_C0/PB2 和 TIMA1_C1/PB3，共用 10 kHz 周期但占空比独立。
 */

/** 将两个方向通道和 PWM 比较值初始化为停车状态。 */
void Motor_Init(void);

/** 设置左右电机带符号 PWM，函数内部会限制到安全计数范围。 */
void Motor_SetPWM(int32_t left_pwm, int32_t right_pwm);

/** 两路方向脚与 PWM 均写零。 */
void Motor_Stop(void);

/** 返回最近一次实际写入左电机的限幅后带符号 PWM。 */
int32_t Motor_GetLeftPWM(void);

/** 返回最近一次实际写入右电机的限幅后带符号 PWM。 */
int32_t Motor_GetRightPWM(void);

#endif
