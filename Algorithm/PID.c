#include "PID.h"

volatile PIDWatch_t g_pid_watch;

/*
 * 本文件只保存控制器状态并执行公式，不包含电机方向、编码器极性或硬件写寄存器。
 * 这些与具体车辆相关的约定统一由 APP 和 BSP 负责。
 */

void PID_WatchInit(float kp, float ki, float output_limit)
{
    g_pid_watch.reset_request = 0U;

    g_pid_watch.left.kp = kp;
    g_pid_watch.left.ki = ki;
    g_pid_watch.left.output_limit = output_limit;
    g_pid_watch.left.target = 0.0f;
    g_pid_watch.left.feedback = 0.0f;
    g_pid_watch.left.error = 0.0f;
    g_pid_watch.left.output = 0.0f;

    g_pid_watch.right.kp = kp;
    g_pid_watch.right.ki = ki;
    g_pid_watch.right.output_limit = output_limit;
    g_pid_watch.right.target = 0.0f;
    g_pid_watch.right.feedback = 0.0f;
    g_pid_watch.right.error = 0.0f;
    g_pid_watch.right.output = 0.0f;
}

/**
 * @brief 初始化一套增量式 PI。
 * @param pid          控制器实例
 * @param kp           比例系数，决定误差变化时的响应强度
 * @param ki           积分系数，决定持续误差带来的 PWM 增量
 * @param output_limit PWM 输出绝对值上限
 */
void PID_IncrementalInit(IncrementalPI_t *pid, float kp, float ki, float output_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->output_limit = output_limit;
    PID_IncrementalReset(pid);
}

/**
 * @brief 清空误差历史和累计输出。
 * @note  停车、重新使能或传感器异常时必须复位，否则旧的累计 PWM 可能导致冲车。
 */
void PID_IncrementalReset(IncrementalPI_t *pid)
{
    pid->last_error = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief 计算一次增量式 PI。
 *
 * 使用公式：
 *   u(k) = u(k-1) + Kp[e(k)-e(k-1)] + Ki*e(k)
 *
 * 这里 target/current 的单位为 m/s，返回值是电机 PWM 计数。
 * 输出到达限幅后直接钳位，避免累计值继续越界；停车时由调用方显式 Reset，
 * 因此不会把停车前的积分量带入下一次启动。
 */
float PID_IncrementalCalculate(IncrementalPI_t *pid, float target, float current)
{
    float error = target - current;

    /* 增量项叠加到上周期输出，随后保存误差供下一周期使用。 */
    pid->output += pid->kp * (error - pid->last_error) + pid->ki * error;
    pid->last_error = error;

    if (pid->output > pid->output_limit) {
        pid->output = pid->output_limit;
    } else if (pid->output < -pid->output_limit) {
        pid->output = -pid->output_limit;
    }

    return pid->output;
}
