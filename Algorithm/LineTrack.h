#ifndef G3507_BASE_LINE_TRACK_H
#define G3507_BASE_LINE_TRACK_H

#include <stdbool.h>

/**
 * @file LineTrack.h
 * @brief 八路循迹误差到左右轮目标速度的简单 PD 差速算法。
 *
 * 本模块只做数值计算，不读取 GPIO、不写电机，也不判断弯道或路口类型。
 * APP 每 5 ms 传入一次横向误差，算法按照以下约定输出左右轮目标速度：
 * - 正误差：黑线位于车体左侧，左轮减速、右轮加速；
 * - 负误差：黑线位于车体右侧，右轮减速、左轮加速；
 * - line_valid=false：立即输出双轮 0 m/s，并清除微分历史。
 */
typedef struct {
    float base_speed_mps;     /* 直线行驶基础速度，单位 m/s */
    float kp;                 /* 横向误差比例系数，输出量纲为 m/s */
    float kd;                 /* 相邻控制周期误差差分系数，输出量纲为 m/s */
    float last_error;         /* 上一周期横向误差，仅用于计算差分项 */
    float target_limit_mps;   /* 单轮目标速度绝对值上限，单位 m/s */
} LineTrack_t;

/** 初始化控制器参数并清空误差历史。 */
void LineTrack_Init(LineTrack_t *controller, float base_speed_mps,
                    float kp, float kd, float target_limit_mps);

/** 仅清空历史误差；停车和重新使能时调用，避免保留旧微分冲击。 */
void LineTrack_Reset(LineTrack_t *controller);

/** 修改基础速度，单位 m/s；不会改变 Kp、Kd 或输出限幅。 */
void LineTrack_SetBaseSpeed(LineTrack_t *controller, float base_speed_mps);

/** 根据本周期循迹误差计算左右轮目标速度，输出单位均为 m/s。 */
void LineTrack_Calculate(LineTrack_t *controller, float line_error, bool line_valid,
                         float *left_target_mps, float *right_target_mps);

#endif
