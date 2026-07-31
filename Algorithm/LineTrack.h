#ifndef G3507_BASE_LINE_TRACK_H
#define G3507_BASE_LINE_TRACK_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file LineTrack.h
 * @brief GitHub 源工程八路循迹逻辑的无硬件数值实现。
 *
 * 本模块完整保存源算法的动态基础速度、离散 PD、最后转向、丢线原地搜线和
 * 6 s 停车保护。GPIO 采样、电机速度 PI 和 PWM 仍由当前工程原有层负责。
 *
 * 误差方向与源工程一致：负值表示黑线偏左，正值表示黑线偏右。
 */
typedef struct {
    /* 初始化后保持不变的源算法参数，速度单位均为 m/s。 */
    float maximum_base_speed_mps;
    float minimum_base_speed_mps;
    float error_slowdown_mps_per_error;
    float kp;                    /* 直线小误差 P */
    float entry_kp;              /* 入弯中等误差 P */
    float curve_kp;              /* 弯中大误差 P */
    float entry_error_abs;
    float curve_error_abs;
    float kd;
    float correction_limit_mps;
    float target_limit_mps;
    float recovery_kick_speed_mps;
    float recovery_speed_mps;
    float direction_deadband;
    uint32_t recovery_kick_ms;
    uint32_t lost_protect_ms;

    /* Race_Control/调试接口给出的速度上限，不改变源算法的误差降速公式。 */
    float requested_base_speed_mps;

    /* 每周期更新的源算法状态和只读诊断量。 */
    float last_error;
    float derivative;
    float active_kp;
    float p_term_mps;
    float d_term_mps;
    float base_speed_mps;
    float correction_mps;
    uint32_t lost_time_ms;
    int8_t last_direction;       /* -1 左转，+1 右转，0 尚未越过方向死区 */
    int8_t recovery_direction;   /* 本次丢线实际采用的原地搜线方向 */
    bool line_was_visible;       /* false 时下一次重捕获首拍 D 项强制为 0 */
    bool line_ever_seen;
    bool recovering;
    bool running;
    bool line_fault;
} LineTrack_t;

/** 初始化全部源循迹参数并清空一次运行的历史状态。 */
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
                    float direction_deadband);

/** 清空误差、方向和丢线历史；保留参数与当前请求速度上限。 */
void LineTrack_Reset(LineTrack_t *controller);

/**
 * 切换正常有线循迹的速度档，只更新基础速度、误差降速和单轮目标上限。
 * 转向 P/D、丢线恢复方向和保护时间均保持不变；调用方应在停车状态切换。
 */
void LineTrack_SetSpeedProfile(LineTrack_t *controller,
                               float maximum_base_speed_mps,
                               float minimum_base_speed_mps,
                               float error_slowdown_mps_per_error,
                               float target_limit_mps);

/** 设置允许的基础速度上限；当前有线速度公式范围为 60~100 RPM。 */
void LineTrack_SetBaseSpeed(LineTrack_t *controller, float base_speed_mps);

/**
 * 执行一次完整源循迹步骤。
 *
 * 有线时保持原动态基础速度、分段 P 和固定差速上限，不引入额外阶段状态。
 * 丢线时立即向最后真实转向的反方向快速打一把，再沿该反方向慢速搜线；
 * 连续丢线达到保护时间后输出 0 并置 line_fault。
 */
void LineTrack_Calculate(LineTrack_t *controller,
                         float line_error,
                         bool line_valid,
                         uint32_t sample_time_ms,
                         float *left_target_mps,
                         float *right_target_mps);

#endif
