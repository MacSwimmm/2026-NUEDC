#ifndef __PID_H
#define __PID_H

/**
 ******************************************************************************
 * @file    PID.h
 * @brief   与具体传感器、电机和物理单位无关的离散位置式 PID。
 *
 * @details
 * 当前实现采用：
 *   error[k]    = target[k] - current[k]
 *   integral[k] = integral[k-1] + error[k]
 *   output[k]   = Kp*error[k] + Ki*integral[k]
 *               + Kd*(error[k] - error[k-1])
 *
 * 公式中没有显式乘除采样周期 dt，因此要求调用周期基本固定，并把 dt 的影响
 * 吸收到 Ki、Kd 中。以后改变控制周期后，原来的 Ki/Kd 不能直接照搬。
 ******************************************************************************
 */

#include <stdint.h>

typedef struct
{
    float Kp;              /* 比例系数：决定对当前误差的直接响应强度 */
    float Ki;              /* 积分系数：消除长期静差，但过大容易积分振荡 */
    float Kd;              /* 微分系数：抑制误差变化趋势，但会放大反馈噪声 */

    float target;          /* 本次计算的目标值，单位由调用者定义 */
    float current;         /* 本次计算的反馈值，必须与 target 同单位 */

    float error;           /* 当前误差 target-current */
    float last_error;      /* 上一次误差，用于离散微分项 */
    float integral;        /* 误差离散累加值，用于积分项 */

    float max_integral;    /* 积分绝对值限幅，属于简单抗积分饱和措施 */
    float max_output;      /* 最终输出绝对值限幅，保护后级执行机构 */

    float output;          /* 最近一次计算并限幅后的输出 */
} PID_t;

/** @brief 写入 PID 参数和限幅，并清空全部历史状态。 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int);
/** @brief 完成一次位置式 PID 运算，返回限幅后的输出。 */
float PID_Calculate(PID_t *pid, float target, float current);
/** @brief 清除目标、反馈、误差、积分和输出；不修改 Kp/Ki/Kd 与限幅。 */
void PID_Reset(PID_t *pid);

#endif /* __PID_H */
