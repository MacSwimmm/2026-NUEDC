#ifndef G3507_BASE_MOTOR_H
#define G3507_BASE_MOTOR_H

#include <stdint.h>

/**
 * Keil Watch 电机镜像。
 * command 字段按“车辆相对方向”填写：正数=该侧车轮向车辆前进方向转。
 * applied 字段为完成安装方向补偿后，实际写入 Motor BSP 的带符号 PWM。
 */
typedef struct {
    uint8_t enable;                   /* Watch 写 1 才允许输出，写 0 立即停车 */
    int32_t left_command_pwm;         /* Watch 可写：左轮车辆相对 PWM */
    int32_t right_command_pwm;        /* Watch 可写：右轮车辆相对 PWM */
    int32_t left_applied_pwm;         /* 实际写入左电机驱动的限幅后 PWM */
    int32_t right_applied_pwm;        /* 实际写入右电机驱动的限幅后 PWM */
} MotorWatch_t;

/** 非 static、volatile 全局变量，可直接添加到 Keil Watch。 */
extern volatile MotorWatch_t g_motor_watch;

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
