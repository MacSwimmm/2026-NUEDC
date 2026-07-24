#include "LineTrack.h"

static float ClampFloat(float value, float lower, float upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

void LineTrack_Init(LineTrack_t *controller, float base_speed_mps,
                    float kp, float kd, float target_limit_mps)
{
    controller->base_speed_mps = base_speed_mps;
    controller->kp = kp;
    controller->kd = kd;
    controller->target_limit_mps = target_limit_mps;
    controller->last_error = 0.0f;
}

void LineTrack_Reset(LineTrack_t *controller)
{
    controller->last_error = 0.0f;
}

void LineTrack_SetBaseSpeed(LineTrack_t *controller, float base_speed_mps)
{
    controller->base_speed_mps = base_speed_mps;
}

/**
 * @brief 根据八路传感器误差生成左右轮速度目标。
 *
 * 正误差表示黑线在车体左侧：左轮减速、右轮加速，车辆向左修正。
 * 如果一条黑线都没有检测到，两个目标立即置零，不保留旧转向命令。
 */
void LineTrack_Calculate(LineTrack_t *controller, float line_error, bool line_valid,
                         float *left_target_mps, float *right_target_mps)
{
    float correction;

    if (!line_valid) {
        controller->last_error = 0.0f;
        *left_target_mps = 0.0f;
        *right_target_mps = 0.0f;
        return;
    }

    correction = controller->kp * line_error +
                 controller->kd * (line_error - controller->last_error);
    controller->last_error = line_error;

    *left_target_mps = ClampFloat(controller->base_speed_mps - correction,
                                  -controller->target_limit_mps,
                                  controller->target_limit_mps);
    *right_target_mps = ClampFloat(controller->base_speed_mps + correction,
                                   -controller->target_limit_mps,
                                   controller->target_limit_mps);
}
