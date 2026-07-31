#ifndef G3507_BASE_PID_H
#define G3507_BASE_PID_H

#include <stdint.h>

/**
 * @file PID.h
 * @brief 两轮速度闭环使用的增量式 PI 数学模块。
 *
 * 本工程只需要车轮速度闭环，因此不引入通用而复杂的控制框架。每个车轮
 * 各保存一份误差历史和 PWM 累计值。算法不接触定时器，调用方约定输入为
 * m/s、输出为 PWM 定时器计数。
 */
typedef struct {
    float kp;            /* 比例系数，作用于本次与上次误差之差 */
    float ki;            /* 积分系数，作用于本次速度误差 */
    float last_error;    /* 上一控制周期误差，单位 m/s */
    float output;        /* 累计 PWM 输出，单位为定时器计数 */
    float output_limit;  /* 输出绝对值限幅，单位为定时器计数 */
} IncrementalPI_t;

/** 单侧速度 PI 的 Keil Watch 镜像。参数字段可写，运行量字段由控制循环刷新。 */
typedef struct {
    float kp;             /* Watch 可写：比例系数 */
    float ki;             /* Watch 可写：积分系数 */
    float output_limit;   /* Watch 可写：PWM 输出限幅 */
    float target;         /* 实时目标速度，单位 m/s */
    float feedback;       /* 实时反馈速度，单位 m/s */
    float error;          /* target-feedback，单位 m/s */
    float output;         /* PI 输出，单位 PWM 计数 */
} PIDChannelWatch_t;

typedef struct {
    uint8_t reset_request; /* Watch 写 1，请求闭环在下一周期清空两侧 PI 历史 */
    PIDChannelWatch_t left;
    PIDChannelWatch_t right;
} PIDWatch_t;

/** 非 static、volatile 全局变量，可直接添加到 Keil Watch。 */
extern volatile PIDWatch_t g_pid_watch;

/** 初始化左右两侧 Watch 参数和运行量；不启动闭环。 */
void PID_WatchInit(float kp, float ki, float output_limit);

/** 配置 PI 参数和输出限幅，并把历史状态清零。 */
void PID_IncrementalInit(IncrementalPI_t *pid, float kp, float ki, float output_limit);

/** 清除历史误差和累计输出，停车或重新使能时必须调用。 */
void PID_IncrementalReset(IncrementalPI_t *pid);

/** 执行一次增量式 PI；target/current 单位为 m/s，返回 PWM 计数。 */
float PID_IncrementalCalculate(IncrementalPI_t *pid, float target, float current);

#endif
