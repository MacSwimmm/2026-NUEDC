#include "Track_PID.h"

/*
 * F4-MIX 位置式 PID 原算法。仅函数/类型加 Track 前缀，计算顺序、积分方式、
 * 微分方式和限幅方式均保持不变。
 */
void TrackPID_Init(TrackPID_t *pid,
                   uint8_t mode,
                   const float parameters[3],
                   float max_out,
                   float max_iout)
{
    if ((pid == NULL) || (parameters == NULL)) return;
    pid->mode = mode;
    pid->Kp = parameters[0];
    pid->Ki = parameters[1];
    pid->Kd = parameters[2];
    pid->OutMax = max_out;
    pid->max_iout = max_iout;
    pid->D_error = 0.0f;
    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->Out = 0.0f;
    pid->ErrorInt = 0.0f;
}

float TrackPID_Calculate(TrackPID_t *pid, float measured, float target)
{
    if (pid == NULL) return 0.0f;

    if (pid->mode == TRACK_PID_POSITION) {
        pid->Target = target;
        pid->Actual = measured;
        pid->Error0 = pid->Target - pid->Actual;
        pid->ErrorInt += pid->Error0;
        Track_LimitMax(pid->ErrorInt, pid->max_iout);
        pid->D_error = pid->Error0 - pid->Error1;
        pid->Out = pid->Kp * pid->Error0
                 + pid->Ki * pid->ErrorInt
                 + pid->Kd * pid->D_error;
        Track_LimitMax(pid->Out, pid->OutMax);
        pid->Error1 = pid->Error0;
    }
    return pid->Out;
}
