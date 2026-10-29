#include "PID.h"

/*
 * 本文件只实现离散位置式 PID 数学运算，不直接读取摄像头/JY901S，也不发送电机
 * 命令。这样同一个 PID_t 既可用于小球位置外环，也可用于未来的摆杆角度内环。
 *
 * 注意：当前算法未显式使用 dt，积分是 error 的逐周期累加，微分是相邻误差差值。
 * 因此实际调参必须在控制周期固定后进行，改变调用频率会改变 Ki、Kd 的实际效果。
 */

/**
 * @brief  初始化 PID 参数和历史状态。
 * @param  pid      PID 结构体指针
 * @param  kp       比例系数，越大对当前位置误差的响应越强
 * @param  ki       积分系数，用于消除静差；过大容易积累后冲过目标
 * @param  kd       微分系数，用于抑制误差变化趋势；过大容易放大视觉噪声
 * @param  max_out  输出绝对值限幅，单位由上层执行器接口定义
 * @param  max_int  积分状态绝对值限幅，用于减轻积分饱和
 * @note   本函数会调用 PID_Reset()，因此重新写参数后历史积分和误差全部清零。
 */
void PID_Init(PID_t *pid, float kp, float ki, float kd, float max_out, float max_int)
{
    /* 参数与内部状态分开保存，便于 Watch 同时观察增益、误差和输出。 */
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->max_output = max_out;
    pid->max_integral = max_int;

    PID_Reset(pid);
}

/**
 * @brief  清空 PID 历史状态。
 * @note   停止控制、反馈失效、模式切换或目标发生大幅阶跃时，应由上层决定是否
 *         调用，防止旧积分和旧微分状态在恢复控制后造成突然输出。
 *         Kp/Ki/Kd 和 max_* 不会被本函数修改。
 */
void PID_Reset(PID_t *pid)
{
    pid->target = 0.0f;
    pid->current = 0.0f;
    pid->error = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->output = 0.0f;
}

/**
 * @brief  位置式 PID 计算。
 * @param  pid      PID 状态结构体，每个独立闭环必须使用独立实例
 * @param  target   期望值
 * @param  current  当前反馈值
 * @retval PID 输出，单位由调用者定义
 *
 * @details 计算过程依次为：误差 -> 积分累加与限幅 -> P/I/D 求和 -> 保存历史误差
 * -> 输出限幅。这里采用简单“积分状态钳位”，并不是反算式抗积分饱和。
 */
float PID_Calculate(PID_t *pid, float target, float current)
{
    pid->target = target;
    pid->current = current;

    /* 统一使用“目标减反馈”。若执行器方向相反，应在系统符号约定中处理。 */
    pid->error = pid->target - pid->current;

    /* 离散积分：固定周期下每调用一次累加一次当前误差。 */
    pid->integral += pid->error;
    if (pid->integral > pid->max_integral) {
        pid->integral = pid->max_integral;
    } else if (pid->integral < -pid->max_integral) {
        pid->integral = -pid->max_integral;
    }

    /* 位置式输出 = 比例项 + 积分项 + 相邻误差差分形成的微分项。 */
    pid->output = (pid->Kp * pid->error) +
                  (pid->Ki * pid->integral) +
                  (pid->Kd * (pid->error - pid->last_error));

    /* 必须在完成微分计算后再覆盖 last_error。 */
    pid->last_error = pid->error;

    /* 最终输出限幅应与后级允许的最大角度、速度或步数范围对应。 */
    if (pid->output > pid->max_output) {
        pid->output = pid->max_output;
    } else if (pid->output < -pid->max_output) {
        pid->output = -pid->max_output;
    }

    return pid->output;
}
