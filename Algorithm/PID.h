#ifndef G3507_BASE_PID_H
#define G3507_BASE_PID_H

/*
 * 增量式 PI 控制器。
 *
 * 本工程只需要车轮速度闭环，因此不引入通用而复杂的控制框架。每个车轮
 * 各保存一份误差历史和 PWM 累计值，输出单位由调用者定义为定时器计数。
 */
typedef struct {
    float kp;
    float ki;
    float last_error;
    float output;
    float output_limit;
} IncrementalPI_t;

void PID_IncrementalInit(IncrementalPI_t *pid, float kp, float ki, float output_limit);
void PID_IncrementalReset(IncrementalPI_t *pid);
float PID_IncrementalCalculate(IncrementalPI_t *pid, float target, float current);

#endif
