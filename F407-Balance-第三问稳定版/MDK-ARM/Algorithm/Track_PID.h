#ifndef __TRACK_PID_H
#define __TRACK_PID_H

#include <stddef.h>
#include <stdint.h>

/* 从 F4-MIX/Algorithm/pid.h 原样移植，仅为避免影响稳定版 PID 而改名。 */
#define Track_LimitMax(input, max)       \
    do {                                 \
        if ((input) > (max)) {           \
            (input) = (max);             \
        } else if ((input) < -(max)) {   \
            (input) = -(max);            \
        }                                \
    } while (0)

enum TRACK_PID_MODE
{
    TRACK_PID_POSITION = 0
};

typedef struct {
    uint8_t mode;
    float Target;
    float Actual;
    float Out;
    float Kp;
    float Ki;
    float Kd;
    float Error0;
    float Error1;
    float ErrorInt;
    float D_error;
    float OutMax;
    float max_iout;
} TrackPID_t;

void TrackPID_Init(TrackPID_t *pid,
                   uint8_t mode,
                   const float parameters[3],
                   float max_out,
                   float max_iout);
float TrackPID_Calculate(TrackPID_t *pid, float measured, float target);

#endif /* __TRACK_PID_H */
