#ifndef G3507_BASE_LINE_TRACK_H
#define G3507_BASE_LINE_TRACK_H

#include <stdbool.h>

/*
 * 简洁的循迹差速控制器。
 * 它没有赛道状态机，只根据当前横向误差计算左右轮目标速度；丢线立即停车。
 */
typedef struct {
    float base_speed_mps;
    float kp;
    float kd;
    float last_error;
    float target_limit_mps;
} LineTrack_t;

void LineTrack_Init(LineTrack_t *controller, float base_speed_mps,
                    float kp, float kd, float target_limit_mps);
void LineTrack_Reset(LineTrack_t *controller);
void LineTrack_SetBaseSpeed(LineTrack_t *controller, float base_speed_mps);
void LineTrack_Calculate(LineTrack_t *controller, float line_error, bool line_valid,
                         float *left_target_mps, float *right_target_mps);

#endif
