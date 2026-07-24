#include "LineTrack.h"

/* 将速度目标限制在机械允许范围内，防止大误差把单轮目标推得过高。 */
static float ClampFloat(float value, float lower, float upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

void LineTrack_Init(LineTrack_t *controller, float base_speed_mps,
                    float kp, float kd, float target_limit_mps)
{
    /* 参数由 APP/app_config.h 提供，算法层不依赖任何具体板级配置。 */
    controller->base_speed_mps = base_speed_mps;
    controller->kp = kp;
    controller->kd = kd;
    controller->target_limit_mps = target_limit_mps;
    controller->last_error = 0.0f;
}

void LineTrack_Reset(LineTrack_t *controller)
{
    /* 只存在一阶历史量，清零即可消除下一次启动时的微分突变。 */
    controller->last_error = 0.0f;
}

void LineTrack_SetBaseSpeed(LineTrack_t *controller, float base_speed_mps)
{
    /* 调用方负责保证该值处于允许档位范围；最终单轮目标仍会再次限幅。 */
    controller->base_speed_mps = base_speed_mps;
}

/**
 * @brief 根据八路传感器误差生成左右轮速度目标。
 *
 * 正误差表示黑线在车体左侧：左轮减速、右轮加速，车辆向左修正。
 * 如果一条黑线都没有检测到，两个目标立即置零，不保留旧转向命令。
 *
 * 修正量计算式：correction = Kp*error + Kd*(error-last_error)。
 * 左右目标分别为 base-correction 与 base+correction，随后各自限制到
 * ±target_limit_mps。这里的 D 项是离散误差差分，控制周期固定为 5 ms，
 * 因此更改控制周期后必须重新评估 Kd，不能只修改定时器而沿用原参数。
 */
void LineTrack_Calculate(LineTrack_t *controller, float line_error, bool line_valid,
                         float *left_target_mps, float *right_target_mps)
{
    float correction;

    if (!line_valid) {
        /* 丢线时不猜测赛道方向，直接停车是基础工程最可控的行为。 */
        controller->last_error = 0.0f;
        *left_target_mps = 0.0f;
        *right_target_mps = 0.0f;
        return;
    }

    /* 先计算公共转向修正，再以相反符号叠加到左右轮。 */
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
