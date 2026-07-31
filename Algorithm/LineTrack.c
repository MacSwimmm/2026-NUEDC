#include "LineTrack.h"

/** 返回浮点数绝对值，避免引入数学库。 */
static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 将浮点值限制在闭区间内。 */
static float ClampFloat(float value, float lower, float upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

void LineTrack_Init(LineTrack_t *controller,
                    float maximum_base_speed_mps,
                    float minimum_base_speed_mps,
                    float error_slowdown_mps_per_error,
                    float kp, float entry_kp, float curve_kp,
                    float entry_error_abs, float curve_error_abs,
                    float kd,
                    float correction_limit_mps,
                    float target_limit_mps,
                    float recovery_kick_speed_mps,
                    float recovery_speed_mps,
                    uint32_t recovery_kick_ms,
                    uint32_t lost_protect_ms,
                    float direction_deadband)
{
    LineTrack_SetSpeedProfile(controller,
                              maximum_base_speed_mps,
                              minimum_base_speed_mps,
                              error_slowdown_mps_per_error,
                              target_limit_mps);
    controller->kp = kp;
    controller->entry_kp = entry_kp;
    controller->curve_kp = curve_kp;
    controller->entry_error_abs = AbsFloat(entry_error_abs);
    controller->curve_error_abs = AbsFloat(curve_error_abs);
    if (controller->entry_error_abs > controller->curve_error_abs) {
        float temporary = controller->entry_error_abs;
        controller->entry_error_abs = controller->curve_error_abs;
        controller->curve_error_abs = temporary;
    }
    controller->kd = kd;
    controller->correction_limit_mps = AbsFloat(correction_limit_mps);
    controller->recovery_kick_speed_mps =
        AbsFloat(recovery_kick_speed_mps);
    controller->recovery_speed_mps = AbsFloat(recovery_speed_mps);
    controller->recovery_kick_ms = recovery_kick_ms;
    controller->lost_protect_ms = lost_protect_ms;
    controller->direction_deadband = AbsFloat(direction_deadband);
    LineTrack_Reset(controller);
}

void LineTrack_Reset(LineTrack_t *controller)
{
    controller->last_error = 0.0f;
    controller->derivative = 0.0f;
    controller->active_kp = controller->kp;
    controller->p_term_mps = 0.0f;
    controller->d_term_mps = 0.0f;
    controller->base_speed_mps = 0.0f;
    controller->correction_mps = 0.0f;
    controller->lost_time_ms = 0U;
    controller->last_direction = 0;
    controller->recovery_direction = 0;
    controller->line_was_visible = false;
    controller->line_ever_seen = false;
    controller->recovering = false;
    controller->running = false;
    controller->line_fault = false;
}

void LineTrack_SetSpeedProfile(LineTrack_t *controller,
                               float maximum_base_speed_mps,
                               float minimum_base_speed_mps,
                               float error_slowdown_mps_per_error,
                               float target_limit_mps)
{
    if (minimum_base_speed_mps > maximum_base_speed_mps) {
        float temporary = minimum_base_speed_mps;
        minimum_base_speed_mps = maximum_base_speed_mps;
        maximum_base_speed_mps = temporary;
    }

    controller->maximum_base_speed_mps = AbsFloat(maximum_base_speed_mps);
    controller->minimum_base_speed_mps = AbsFloat(minimum_base_speed_mps);
    controller->error_slowdown_mps_per_error =
        AbsFloat(error_slowdown_mps_per_error);
    controller->target_limit_mps = AbsFloat(target_limit_mps);
    controller->requested_base_speed_mps =
        controller->maximum_base_speed_mps;
}

void LineTrack_SetBaseSpeed(LineTrack_t *controller, float base_speed_mps)
{
    controller->requested_base_speed_mps = ClampFloat(
        base_speed_mps,
        controller->minimum_base_speed_mps,
        controller->maximum_base_speed_mps);
}

void LineTrack_Calculate(LineTrack_t *controller,
                         float line_error,
                         bool line_valid,
                         uint32_t sample_time_ms,
                         float *left_target_mps,
                         float *right_target_mps)
{
    if (line_valid) {
        float error_abs = AbsFloat(line_error);
        float base_speed = controller->maximum_base_speed_mps -
            error_abs * controller->error_slowdown_mps_per_error;
        float active_kp = controller->kp;
        float correction;

        base_speed = ClampFloat(base_speed,
                                controller->minimum_base_speed_mps,
                                controller->maximum_base_speed_mps);
        if (base_speed > controller->requested_base_speed_mps) {
            base_speed = controller->requested_base_speed_mps;
        }

        /* 源工程重捕获首拍不计算误差差分，避免沿用丢线前的 D 项历史。 */
        controller->derivative = controller->line_was_visible ?
            (line_error - controller->last_error) : 0.0f;
        if (error_abs >= controller->curve_error_abs) {
            active_kp = controller->curve_kp;
        } else if (error_abs >= controller->entry_error_abs) {
            active_kp = controller->entry_kp;
        }
        controller->active_kp = active_kp;
        controller->p_term_mps = active_kp * line_error;
        controller->d_term_mps = controller->kd * controller->derivative;
        correction = controller->p_term_mps + controller->d_term_mps;
        correction = ClampFloat(correction,
                                -controller->correction_limit_mps,
                                controller->correction_limit_mps);

        /* 完整沿用源工程符号：left=base+correction，right=base-correction。 */
        *left_target_mps = ClampFloat(base_speed + correction,
                                      -controller->target_limit_mps,
                                      controller->target_limit_mps);
        *right_target_mps = ClampFloat(base_speed - correction,
                                       -controller->target_limit_mps,
                                       controller->target_limit_mps);

        if (line_error < -controller->direction_deadband) {
            controller->last_direction = -1;
        } else if (line_error > controller->direction_deadband) {
            controller->last_direction = 1;
        }

        controller->last_error = line_error;
        controller->base_speed_mps = base_speed;
        controller->correction_mps = correction;
        controller->lost_time_ms = 0U;
        controller->recovery_direction = controller->last_direction;
        controller->line_was_visible = true;
        controller->line_ever_seen = true;
        controller->recovering = false;
        controller->running = true;
        return;
    }

    controller->line_was_visible = false;
    controller->derivative = 0.0f;
    controller->p_term_mps = 0.0f;
    controller->d_term_mps = 0.0f;
    controller->base_speed_mps = 0.0f;
    controller->correction_mps = 0.0f;
    controller->recovering = false;
    controller->running = false;

    if (controller->lost_time_ms < controller->lost_protect_ms) {
        uint32_t remaining = controller->lost_protect_ms -
                             controller->lost_time_ms;
        controller->lost_time_ms +=
            (sample_time_ms < remaining) ? sample_time_ms : remaining;
    }

    if (controller->line_ever_seen &&
        (controller->lost_time_ms < controller->lost_protect_ms)) {
        int8_t last_direction = controller->last_direction;
        int8_t direction;
        float recovery_speed;

        /* 未越过方向死区时，以最后有效误差符号确定最后真实转向。 */
        if (last_direction == 0) {
            last_direction = (controller->last_error < 0.0f) ? -1 : 1;
        }
        /* 全白后从第一拍开始固定向最后真实转向的反方向搜索。 */
        direction = (int8_t)(-last_direction);
        recovery_speed = (controller->lost_time_ms <=
                          controller->recovery_kick_ms) ?
                         controller->recovery_kick_speed_mps :
                         controller->recovery_speed_mps;

        controller->recovery_direction = direction;
        controller->recovering = true;
        controller->running = true;
        if (direction < 0) {
            *left_target_mps = -recovery_speed;
            *right_target_mps = recovery_speed;
        } else {
            *left_target_mps = recovery_speed;
            *right_target_mps = -recovery_speed;
        }
        controller->correction_mps =
            (*left_target_mps - *right_target_mps) * 0.5f;
        return;
    }

    *left_target_mps = 0.0f;
    *right_target_mps = 0.0f;
    if (controller->lost_time_ms >= controller->lost_protect_ms) {
        controller->line_fault = true;
    }
}
