#ifndef __LINE_TRACK_CONTROL_H
#define __LINE_TRACK_CONTROL_H

#include <stdint.h>

/* 以下控制参数逐项取自 F4-MIX/app/app_chassis.h。 */
#define MOTOR_SPEED_PID_KP                 (5.00f)
#define MOTOR_SPEED_PID_KI                 (0.10f)
#define MOTOR_SPEED_PID_KD                 (0.10f)
#define MOTOR_SPEED_PID_MAX_OUT            (150.0f)
#define MOTOR_SPEED_PID_MAX_IOUT           (40.0f)
#define MOTOR_ENCODER_FAULT_TIMEOUT_MS     (500U)
#define MOTOR_ENCODER_FAULT_MIN_TARGET_SPEED (5.0f)
#define MOTOR_ENCODER_FAULT_MIN_PWM        (15)

#define TRACK_BASE_SPEED                   (30.0f)
#define TRACK_TASK3_BASE_SPEED             (20.0f)
#define TRACK_TASK4_BASE_SPEED             (20.0f)
#define TRACK_BALANCE_ACCEL_SPEED_PER_S    (20.0f)
#define TRACK_TASK3_RAMP_SPEED_PER_S       (20.0f)
#define TRACK_TASK3_STOP_RAMP_SPEED_PER_S  (10.0f)
#define TRACK_TASK4_RAMP_SPEED_PER_S       (10.0f)
#define TRACK_TASK4_STOP_RAMP_SPEED_PER_S  (5.0f)
#define TRACK_BALANCE_STOP_SPEED_EPSILON   (0.05f)

/*
 * 仅慢速模式恢复 F4-MIX Task4 的底盘加速度前馈。快速、直行模式保持原样，
 * 不产生滚球前馈，也不要求滚球闭环已经启动。
 */
#define TRACK_TASK4_ACCEL_FILTER_OLD_WEIGHT (0.80f)
#define TRACK_TASK4_START_ACCEL_FF_GAIN     (0.15f)
#define TRACK_TASK4_RUNNING_ACCEL_FF_GAIN   (0.17f)
#define TRACK_TASK4_STOP_ACCEL_FF_GAIN      (0.18f)
#define TRACK_TASK4_ACCEL_FF_DIRECTION_SIGN (-1.0f)
#define TRACK_TASK4_ACCEL_FF_MAX_DEG        (8.0f)
#define TRACK_TASK4_ACCEL_FF_SLEW_DEG_PER_S (20.0f)
#define TRACK_MIN_BASE_SPEED               (12.0f)
#define TRACK_MAX_WHEEL_SPEED              (35.0f)
#define TRACK_ANGLE_PID_KP                 (0.045f)
#define TRACK_ANGLE_PID_KI                 (0.0002f)
#define TRACK_ANGLE_PID_KD                 (0.008f)
#define TRACK_ANGLE_PID_MAX_OUT            (18.0f)
#define TRACK_ANGLE_PID_MAX_IOUT           (20000.0f)
#define TRACK_MAX_CORRECTION_SPEED         (18.0f)
#define TRACK_RECOVERY_SPEED               (10.0f)
#define TRACK_LOST_PROTECT_MS              (6000U)
#define TRACK_DIRECTION_DEADBAND           (25)
#define TRACK_FINISH_SENSOR_MASK           ((1U << 6U) | (1U << 7U))
#define TRACK_FINISH_CONFIRM_MS            (30U)
#define TRACK_FINISH_STOP_DELAY_MS         (800U)
#define TRACK_TASK3_FINISH_SENSOR_MASK     (1U << 5U)
#define TRACK_TASK3_FINISH_STOP_DELAY_MS   (0U)

typedef enum {
    LINE_TRACK_STATE_IDLE = 0,
    LINE_TRACK_STATE_TASK1_TRACKING,
    LINE_TRACK_STATE_TASK1_FINISHED,
    LINE_TRACK_STATE_TASK3_BALANCE,
    LINE_TRACK_STATE_TASK3_FINISHED,
    LINE_TRACK_STATE_TASK4_BALANCE,
    LINE_TRACK_STATE_TASK4_FINISHED
} LineTrackState_t;

typedef enum {
    LINE_TRACK_MODE_FAST = 0,
    LINE_TRACK_MODE_STRAIGHT,
    LINE_TRACK_MODE_SLOW
} LineTrackMode_t;

typedef struct {
    uint8_t initialized;
    uint8_t state;
    uint8_t car_mode;
    uint8_t track_bits;
    int16_t line_error;
    float left_speed;
    float right_speed;
    float left_speed_set;
    float right_speed_set;
    int32_t left_pwm;
    int32_t right_pwm;
    uint8_t left_encoder_fault;
    uint8_t right_encoder_fault;
    uint8_t finish_pending;
    uint8_t finish_armed;
    uint32_t elapsed_time_ms;
    uint32_t final_time_ms;
    uint32_t fast_start_count;
    uint32_t slow_start_count;
    uint32_t straight_start_count;
    uint32_t stop_request_count;
    /* 兼容既有 Watch 布局；循迹按键现在不再受 Balance 状态门控。 */
    uint32_t balance_not_ready_reject_count;
    uint32_t recovery_enter_count;
    uint32_t control_count;
    uint8_t balance_feedforward_active;
    float acceleration_source_speed;
    float longitudinal_acceleration_filtered;
    float acceleration_feedforward_degrees;
} LineTrackDebug_t;

typedef struct {
    uint8_t active;                    /* 仅慢速模式运行/减速停车时为 1 */
    float source_speed;                /* F4-MIX 斜坡后的纵向速度指令 */
    float acceleration_filtered;       /* 纵向加速度低通结果 */
    float feedforward_degrees;         /* 应叠加到摆球电机目标的角度 */
} LineTrackBalanceFeedforward_t;

extern volatile LineTrackDebug_t g_line_track_debug;

void LineTrack_Init(void);
void LineTrack_Process(void);
uint8_t LineTrack_RequestStart(LineTrackMode_t mode);
uint8_t LineTrack_RequestStop(void);
uint8_t LineTrack_GetTelemetrySnapshot(LineTrackDebug_t *snapshot);
uint8_t LineTrack_GetBalanceFeedforwardSnapshot(
    LineTrackBalanceFeedforward_t *snapshot);

#endif /* __LINE_TRACK_CONTROL_H */
