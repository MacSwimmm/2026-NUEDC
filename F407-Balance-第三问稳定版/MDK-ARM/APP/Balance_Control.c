#include "Balance_Control.h"

#include "Camera.h"
#include "LineTrack_Control.h"
#include "Motor_Command.h"
#include "Motor_Feedback.h"
#include "Track_Keys.h"
#include "UI_Menu.h"
#include "main.h"

#include <float.h>
#include <limits.h>
#include <string.h>

#define BALANCE_BUTTON_DEBOUNCE_MS       30U
#define BALANCE_SAMPLE_MIN_DT_MS         10U
#define BALANCE_SAMPLE_NORMAL_MAX_MS    100U
#define BALANCE_SAMPLE_VALID_MAX_MS     150U
#define BALANCE_NOMINAL_DT_S           0.050f
#define BALANCE_ZERO_VERIFY_DEG          1.0f
#define BALANCE_SCORE_LIMIT_MS          BALANCE_TRAJECTORY_SCORE_LIMIT_MS
#define BALANCE_OFFICIAL_ERROR_MM       BALANCE_TRAJECTORY_OFFICIAL_ERROR_MM
#define BALANCE_POSITIVE_TURN_PREVIEW_S  0.15f
#define BALANCE_TRAJECTORY_P_FAR_LIMIT_PULSE    52.0f
#define BALANCE_TRAJECTORY_P_NEAR_LIMIT_PULSE   26.0f
#define BALANCE_TRAJECTORY_D_TARGET_LIMIT_PULSE  90.0f
#define BALANCE_START_RAMP_MIN_SCALE     0.40f
#define BALANCE_AUTO_START_AFTER_POWER_ON   1U
#define BALANCE_ORIGIN_TARGET_MM             0
#define BALANCE_POSITIVE_NEAR_RESPONSE_SCALE  0.35f
#define BALANCE_NEGATIVE_NEAR_RESPONSE_SCALE  0.25f
#define BALANCE_HOLD_DEADBAND_MM                1.0f
#define BALANCE_HOLD_DEADBAND_SPEED_MM_S       15.0f
#define BALANCE_STRONG_LOCK_DEADBAND_MM          0.25f
#define BALANCE_STRONG_LOCK_DEADBAND_SPEED_MM_S  6.0f
#define BALANCE_STRONG_LOCK_PREDICT_S             0.060f
#define BALANCE_STRONG_LOCK_PREDICT_LIMIT_MM      6.0f
#define BALANCE_STRONG_LOCK_KP_SCALE               1.3333333f
#define BALANCE_STRONG_LOCK_KI_SCALE               1.5000000f
#define BALANCE_STRONG_LOCK_KD_SCALE               1.1363636f
#define BALANCE_STRONG_LOCK_FOLLOW_SCALE           1.2500000f
#define BALANCE_STRONG_LOCK_SLEW_SCALE             1.1666667f
#define BALANCE_STRONG_LOCK_STEP_SCALE             1.1666667f
#define BALANCE_HOLD_D_NEAR_ZONE_MM             12.0f
#define BALANCE_HOLD_D_VELOCITY_DEADBAND_MM_S   15.0f
#define BALANCE_HOLD_INTEGRAL_MAX_SPEED_MM_S    15.0f
#define BALANCE_RETURN_ZERO_MAX_SLEW_PULSE_S  700.0f
#define BALANCE_RETURN_ZERO_MAX_STEP_PULSE      35.0f
#define BALANCE_TARGET_LEVEL_QUERY_MS           50U
#define BALANCE_TARGET_LEVEL_TIMEOUT_MS       3000U

/* F4-MIX 使用 3200 pulse/rev；当前工程以水平零点为 0 pulse。 */
#define BALANCE_MOTOR_PULSES_PER_REVOLUTION  3200.0f
#define BALANCE_PULSES_PER_DEGREE \
    (BALANCE_MOTOR_PULSES_PER_REVOLUTION / 360.0f)
#define BALANCE_CAMERA_VELOCITY_ABS_MAX_CM_S  200.0f

/* 仅供 MOVE_POSITIVE / MOVE_NEGATIVE / HOLD_NEGATIVE 评分状态使用。 */
#define BALANCE_SCORING_POSITION_PID_KP                 1.05f
#define BALANCE_SCORING_POSITION_PID_KI                 0.06f
#define BALANCE_SCORING_POSITION_PID_KD                 0.01f
#define BALANCE_SCORING_POSITION_PID_MAX_OUT_CM_S      50.0f
#define BALANCE_SCORING_POSITION_PID_MAX_IOUT_CM_S     10.0f
#define BALANCE_SCORING_POSITION_INTEGRAL_DEADBAND_CM   0.10f
#define BALANCE_SCORING_VELOCITY_PID_KP                 1.60f
#define BALANCE_SCORING_VELOCITY_PID_KI                 0.001f
#define BALANCE_SCORING_VELOCITY_PID_KD                 1.90f

/* 当前实物机械方向：正 pulse 使小球向左（-x），负 pulse 使小球向右（+x）。 */
#define BALANCE_BALL_DIRECTION_PER_POSITIVE_PULSE  (-1.0f)

#define BALANCE_TRACE_FLAG_STATE_CHANGE  0x01U
#define BALANCE_TRACE_FLAG_VISION_HOLD   0x02U
#define BALANCE_TRACE_FLAG_FAULT         0x04U
#define BALANCE_TRACE_FLAG_ABNORMAL_DT   0x08U
#define BALANCE_TRACE_FLAG_COMPLETE      0x10U

typedef enum
{
    BALANCE_SAMPLE_NONE = 0,
    BALANCE_SAMPLE_REBUILT,
    BALANCE_SAMPLE_NORMAL,
    BALANCE_SAMPLE_ABNORMAL,
    BALANCE_SAMPLE_TOO_FAST
} BalanceSampleResult_t;

typedef struct
{
    float error0;
    float error1;
    float error_integral;
    float p_output;
    float i_output;
    float d_output;
    float output;
} BalanceCascadePidState_t;

typedef struct
{
    float position_output_cm_s;
    float target_velocity_cm_s;
    float velocity_output_degrees;
    float feedforward_degrees;
    float desired_correction_degrees;
    uint8_t feedforward_active;
} BalanceCascadeRuntime_t;

/*
 * 控制算法仍需要完整运行状态，但这些细节不再全部暴露到 Keil Watch。
 * 对外的 g_balance_debug 只保留现场最关键的约 30 个字段；通信原始细节继续在
 * g_motor_command_debug / g_camera_debug 中按需查看，避免一个结构体塞满 Watch。
 */
typedef struct
{
    uint8_t state;
    uint8_t previous_state;
    uint8_t fault;
    uint8_t start_reject_reason;
    uint32_t state_enter_tick;
    uint32_t run_start_tick;
    uint32_t run_elapsed_ms;
    uint32_t completion_time_ms;
    uint8_t completion_recorded;
    uint8_t time_score_failed;
    uint8_t within_5s;
    uint8_t score_pass;
    uint8_t pa0_consumed;
    uint8_t start_ready;
    uint8_t trajectory_ready;
    uint8_t trajectory_started;
    uint8_t trajectory_ready_count;
    uint8_t button_raw_level;
    uint8_t button_debounced_level;
    uint8_t button_release_seen;
    uint32_t button_debounce_tick;
    uint32_t button_press_count;
    uint32_t button_rejected_count;
    uint8_t target_button_raw_level;
    uint8_t target_button_debounced_level;
    uint32_t target_button_press_count;
    uint32_t target_button_rejected_count;
    uint32_t watch_start_count;
    uint8_t camera_online;
    uint8_t target_found;
    uint32_t frame_sequence;
    uint32_t valid_sequence;
    uint32_t processed_valid_sequence;
    uint32_t last_frame_age_ms;
    uint32_t last_valid_age_ms;
    uint32_t sample_dt_ms;
    int16_t position_raw_mm;
    int16_t position_used_mm;
    float velocity_raw_mm_s;
    float velocity_filtered_mm_s;
    uint8_t velocity_history_valid;
    uint8_t vision_lost_streak;
    uint32_t fast_sample_count;
    uint32_t abnormal_dt_count;
    uint32_t coordinate_range_reject_count;
    uint32_t vision_hold_count;
    uint32_t vision_fault_count;
    int16_t target_mm;
    float error_mm;
    float integral_error_mm_s;
    float p_rate_pulse_s;
    float i_rate_pulse_s;
    float d_rate_pulse_s;
    float centering_rate_pulse_s;
    float drive_rate_raw_pulse_s;
    float drive_rate_limited_pulse_s;
    uint8_t approach_brake_active;
    float approach_rate_limit_pulse_s;
    float command_position_pulse;
    int32_t final_requested_pulse;
    int32_t last_transmitted_pulse;
    int32_t last_acked_pulse;
    uint32_t soft_limit_count;
    uint32_t slew_limit_count;
    uint32_t integral_freeze_count;
    uint32_t tuning_error_mask;
    uint32_t tuning_reject_count;
    uint32_t tuning_accept_count;
    uint8_t control_profile;
    uint8_t profile_exit_confirm_count;
    uint32_t profile_switch_count;
    uint8_t motor_owner;
    uint8_t motor_command_state;
    uint32_t motor_request_id;
    uint8_t motor_command_type;
    uint32_t motor_tx_tick;
    uint32_t motor_ack_deadline;
    uint8_t motor_retry_count;
    uint8_t motor_ack_status;
    uint32_t motor_tx_fail_count;
    uint32_t motor_ack_timeout_count;
    uint32_t motor_ack_reject_count;
    uint32_t motor_busy_defer_count;
    uint32_t hard_limit_reject_count;
    int32_t pending_latest_pulse;
    uint8_t pending_latest_valid;
    int16_t positive_turn_position_mm;
    float positive_turn_velocity_mm_s;
    int16_t positive_turn_error_mm;
    int16_t positive_turn_abs_error_mm;
    int16_t positive_peak_position_mm;
    int16_t positive_overshoot_mm;
    int16_t negative_min_position_mm;
    int16_t negative_overshoot_mm;
    int16_t negative_stable_window_max_abs_error_mm;
    uint8_t official_endpoint_error_pass;
    uint8_t official_time_pass;
    uint8_t run_fault_free;
} BalanceRuntime_t;

volatile BalanceTuning_t g_balance_tuning;
volatile BalanceDebug_t g_balance_debug;
volatile uint32_t g_balance_command;
volatile BalanceTraceBuffer_t g_balance_trace;
volatile BalanceControlState_t g_balance_state;

static volatile BalanceDebug_t *const s_public_debug = &g_balance_debug;
static BalanceRuntime_t s_balance_runtime;

/* 本文件以下原有算法字段访问统一指向私有运行态。 */
#define g_balance_debug s_balance_runtime

static BalanceTuning_t s_tuning;
static uint8_t s_tuning_valid;
static uint32_t s_last_frame_sequence;
static uint32_t s_last_valid_tick;
static int16_t s_last_position_mm;
static uint8_t s_velocity_history_valid;
static uint8_t s_button_raw;
static uint8_t s_button_debounced;
static uint32_t s_button_raw_change_tick;
static uint8_t s_start_confirm_count;
static uint8_t s_positive_confirm_count;
static uint32_t s_negative_stable_start_tick;
static BalanceState_t s_resume_state;
static uint32_t s_state_request_id;
static uint8_t s_start_zero_phase;
static uint8_t s_trace_pending_flags;
static uint8_t s_return_is_fault;
static uint8_t s_return_stop_done;
static uint32_t s_return_request_id;
static uint32_t s_return_last_tick;
static uint8_t s_disable_phase;
static uint32_t s_disable_request_id;
static uint32_t s_seen_motor_completion_id;
static uint8_t s_vision_hold_range_fault;
static uint32_t s_last_physical_reject_count;
static uint8_t s_auto_start_completed;
static uint8_t s_target_level_phase;
static uint8_t s_target_exit_pending;
static uint32_t s_target_level_next_query_tick;
static uint32_t s_target_capture_baseline_valid_sequence;
static BalanceCascadePidState_t s_position_pid;
static BalanceCascadePidState_t s_velocity_pid;
static BalanceCascadeRuntime_t s_cascade_runtime;

static float Balance_AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static int16_t Balance_AbsInt16(int16_t value)
{
    return (value < 0) ? (int16_t)(-value) : value;
}

static uint8_t Balance_FloatValid(float value)
{
    return (uint8_t)((value == value) &&
                     (value <= FLT_MAX) &&
                     (value >= -FLT_MAX));
}

static float Balance_ClampFloat(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static int32_t Balance_RoundToInt32(float value)
{
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) :
                             (int32_t)(value - 0.5f);
}

static void Balance_ResetCascadePid(BalanceCascadePidState_t *pid)
{
    if (pid == NULL) return;
    memset(pid, 0, sizeof(*pid));
}

/* 与 F4-MIX chassis_ball_pid_update() 保持相同的按帧积分和显式 D 项语义。 */
static float Balance_CascadePidUpdate(BalanceCascadePidState_t *pid,
                                      float measured,
                                      float target,
                                      float derivative_error,
                                      uint8_t integral_enabled,
                                      float kp,
                                      float ki,
                                      float kd,
                                      float max_output,
                                      float max_integral_output)
{
    float ki_abs;
    float integral_error_limit;

    if (pid == NULL) return 0.0f;

    pid->error1 = pid->error0;
    pid->error0 = target - measured;
    if (integral_enabled != 0U) {
        pid->error_integral += pid->error0;
    } else {
        pid->error_integral = 0.0f;
    }

    ki_abs = Balance_AbsFloat(ki);
    if ((ki_abs > 0.000001f) && (max_integral_output > 0.0f)) {
        integral_error_limit = max_integral_output / ki_abs;
        pid->error_integral = Balance_ClampFloat(
            pid->error_integral,
            -integral_error_limit,
            integral_error_limit);
        pid->i_output = Balance_ClampFloat(
            ki * pid->error_integral,
            -max_integral_output,
            max_integral_output);
    } else {
        pid->error_integral = 0.0f;
        pid->i_output = 0.0f;
    }

    pid->p_output = kp * pid->error0;
    pid->d_output = kd * derivative_error;
    pid->output = Balance_ClampFloat(
        pid->p_output + pid->i_output + pid->d_output,
        -max_output,
        max_output);
    return pid->output;
}

static uint8_t Balance_StateIsMotion(BalanceState_t state)
{
    return (uint8_t)((state == BALANCE_STATE_MOVE_POSITIVE) ||
                     (state == BALANCE_STATE_MOVE_NEGATIVE) ||
                     (state == BALANCE_STATE_HOLD_NEGATIVE) ||
                     (state == BALANCE_STATE_HOLD_ORIGIN) ||
                     (state == BALANCE_STATE_HOLD_CAPTURED_TARGET));
}

static uint8_t Balance_StateIsStrongHold(BalanceState_t state)
{
    return (uint8_t)((state == BALANCE_STATE_HOLD_ORIGIN) ||
                     (state == BALANCE_STATE_HOLD_CAPTURED_TARGET));
}

static uint8_t Balance_StateIsRunning(BalanceState_t state)
{
    return (uint8_t)(((state >= BALANCE_STATE_START_QUERY_MOTOR) &&
                      (state <= BALANCE_STATE_RETURN_ZERO)) ||
                     (state == BALANCE_STATE_HOLD_ORIGIN) ||
                     (state == BALANCE_STATE_TARGET_LEVELING) ||
                     (state == BALANCE_STATE_TARGET_WAIT_CAPTURE) ||
                     (state == BALANCE_STATE_HOLD_CAPTURED_TARGET));
}

static void Balance_SetState(BalanceState_t state)
{
    if ((BalanceState_t)g_balance_debug.state == state) {
        return;
    }
    g_balance_debug.previous_state = g_balance_debug.state;
    g_balance_debug.state = (uint8_t)state;
    g_balance_debug.state_enter_tick = HAL_GetTick();
    s_trace_pending_flags |= BALANCE_TRACE_FLAG_STATE_CHANGE;
}

static void Balance_ClearPid(void)
{
    Balance_ResetCascadePid(&s_position_pid);
    Balance_ResetCascadePid(&s_velocity_pid);
    memset(&s_cascade_runtime, 0, sizeof(s_cascade_runtime));
    g_balance_debug.integral_error_mm_s = 0.0f;
    g_balance_debug.i_rate_pulse_s = 0.0f;
    g_balance_debug.centering_rate_pulse_s = 0.0f;
}

static void Balance_ResetHoldProfile(void)
{
    g_balance_debug.control_profile = BALANCE_CONTROL_PROFILE_NEAR;
    g_balance_debug.profile_exit_confirm_count = 0U;
}

static void Balance_UpdateHoldProfile(BalanceSampleResult_t sample)
{
    float abs_error = Balance_AbsFloat(
        (float)g_balance_debug.target_mm -
        (float)g_balance_debug.position_used_mm);
    float abs_velocity = Balance_AbsFloat(
        g_balance_debug.velocity_filtered_mm_s);

    if (g_balance_debug.control_profile == BALANCE_CONTROL_PROFILE_NEAR) {
        if ((abs_error >= (float)s_tuning.far_enter_position_mm) ||
            ((sample == BALANCE_SAMPLE_NORMAL) &&
             (abs_velocity >= s_tuning.far_enter_speed_mm_s))) {
            g_balance_debug.control_profile = BALANCE_CONTROL_PROFILE_FAR;
            g_balance_debug.profile_exit_confirm_count = 0U;
            g_balance_debug.profile_switch_count++;
            Balance_ClearPid();
        }
        return;
    }

    if ((sample == BALANCE_SAMPLE_NORMAL) &&
        (abs_error <= (float)s_tuning.far_exit_position_mm) &&
        (abs_velocity <= s_tuning.far_exit_speed_mm_s)) {
        if (g_balance_debug.profile_exit_confirm_count < UINT8_MAX) {
            g_balance_debug.profile_exit_confirm_count++;
        }
        if (g_balance_debug.profile_exit_confirm_count >=
            s_tuning.far_exit_confirm_samples) {
            g_balance_debug.control_profile = BALANCE_CONTROL_PROFILE_NEAR;
            g_balance_debug.profile_exit_confirm_count = 0U;
            g_balance_debug.profile_switch_count++;
            Balance_ClearPid();
        }
    } else {
        g_balance_debug.profile_exit_confirm_count = 0U;
    }
}

static void Balance_ClearTrace(void)
{
    memset((void *)&g_balance_trace, 0, sizeof(g_balance_trace));
    s_trace_pending_flags = 0U;
}

static void Balance_ClearScoreSummary(void)
{
    g_balance_debug.completion_time_ms = 0U;
    g_balance_debug.completion_recorded = 0U;
    g_balance_debug.time_score_failed = 0U;
    g_balance_debug.within_5s = 0U;
    g_balance_debug.score_pass = 0U;
    g_balance_debug.positive_turn_position_mm = 0;
    g_balance_debug.positive_turn_velocity_mm_s = 0.0f;
    g_balance_debug.positive_turn_error_mm = 0;
    g_balance_debug.positive_turn_abs_error_mm = 0;
    g_balance_debug.positive_peak_position_mm = INT16_MIN;
    g_balance_debug.positive_overshoot_mm = 0;
    g_balance_debug.negative_min_position_mm = INT16_MAX;
    g_balance_debug.negative_overshoot_mm = 0;
    g_balance_debug.negative_stable_window_max_abs_error_mm = 0;
    g_balance_debug.official_endpoint_error_pass = 0U;
    g_balance_debug.official_time_pass = 0U;
}

static void Balance_ClearRunData(void)
{
    uint8_t pa0_consumed = g_balance_debug.pa0_consumed;
    uint8_t trajectory_started = g_balance_debug.trajectory_started;
    uint32_t button_press_count = g_balance_debug.button_press_count;
    uint32_t button_rejected_count = g_balance_debug.button_rejected_count;
    uint32_t watch_start_count = g_balance_debug.watch_start_count;

    g_balance_debug.run_start_tick = HAL_GetTick();
    g_balance_debug.run_elapsed_ms = 0U;
    Balance_ClearScoreSummary();
    g_balance_debug.fault = BALANCE_FAULT_NONE;
    g_balance_debug.run_fault_free = 1U;
    g_balance_debug.error_mm = 0.0f;
    g_balance_debug.p_rate_pulse_s = 0.0f;
    g_balance_debug.i_rate_pulse_s = 0.0f;
    g_balance_debug.d_rate_pulse_s = 0.0f;
    g_balance_debug.centering_rate_pulse_s = 0.0f;
    g_balance_debug.drive_rate_raw_pulse_s = 0.0f;
    g_balance_debug.drive_rate_limited_pulse_s = 0.0f;
    g_balance_debug.approach_brake_active = 0U;
    g_balance_debug.approach_rate_limit_pulse_s = 0.0f;
    g_balance_debug.command_position_pulse = 0.0f;
    g_balance_debug.final_requested_pulse = 0;
    g_balance_debug.integral_error_mm_s = 0.0f;
    g_balance_debug.soft_limit_count = 0U;
    g_balance_debug.slew_limit_count = 0U;
    g_balance_debug.integral_freeze_count = 0U;
    Balance_ResetHoldProfile();
    g_balance_debug.profile_switch_count = 0U;
    g_balance_debug.vision_lost_streak = 0U;
    g_balance_debug.pa0_consumed = pa0_consumed;
    g_balance_debug.trajectory_ready = 0U;
    g_balance_debug.trajectory_ready_count = 0U;
    g_balance_debug.trajectory_started = trajectory_started;
    g_balance_debug.button_press_count = button_press_count;
    g_balance_debug.button_rejected_count = button_rejected_count;
    g_balance_debug.watch_start_count = watch_start_count;

    s_positive_confirm_count = 0U;
    s_negative_stable_start_tick = 0U;
    s_state_request_id = 0U;    s_start_zero_phase = 0U;
    s_return_request_id = 0U;
    s_return_is_fault = 0U;
    s_return_stop_done = 0U;
    s_disable_phase = 0U;
    s_disable_request_id = 0U;
    Balance_ClearTrace();
}

/**
 * @brief 为 PA0 轨迹清空独立计时/评分数据，但保留当前电机目标和速度历史。
 * @note 轨迹从 HOLD_ORIGIN 原地接管，禁止在这里重新清零、重新使能或把命令脉冲归零。
 */
static void Balance_ClearTrajectoryRunData(uint32_t now)
{
    g_balance_debug.run_start_tick = now;
    g_balance_debug.run_elapsed_ms = 0U;
    Balance_ClearScoreSummary();
    g_balance_debug.run_fault_free = 1U;
    g_balance_debug.error_mm = 0.0f;
    g_balance_debug.p_rate_pulse_s = 0.0f;
    g_balance_debug.i_rate_pulse_s = 0.0f;
    g_balance_debug.d_rate_pulse_s = 0.0f;
    g_balance_debug.centering_rate_pulse_s = 0.0f;
    g_balance_debug.drive_rate_raw_pulse_s = 0.0f;
    g_balance_debug.drive_rate_limited_pulse_s = 0.0f;
    g_balance_debug.approach_brake_active = 0U;
    g_balance_debug.approach_rate_limit_pulse_s = 0.0f;
    g_balance_debug.integral_error_mm_s = 0.0f;
    g_balance_debug.soft_limit_count = 0U;
    g_balance_debug.slew_limit_count = 0U;
    g_balance_debug.integral_freeze_count = 0U;
    g_balance_debug.profile_switch_count = 0U;
    g_balance_debug.vision_lost_streak = 0U;
    g_balance_debug.trajectory_ready = 0U;
    g_balance_debug.trajectory_ready_count = 0U;
    g_balance_debug.trajectory_started = 1U;
    g_balance_debug.pa0_consumed = 1U;

    Balance_ClearPid();
    Balance_ResetHoldProfile();
    s_positive_confirm_count = 0U;
    s_negative_stable_start_tick = 0U;
    Balance_ClearTrace();
}

static void Balance_LoadDefaultTuning(void)
{
    /*
     * F4-MIX 串级滚球控制：
     *   位置环：位置 cm -> 目标球速 cm/s，D 项直接使用 -实测球速；
     *   速度环：目标球速 - 实测球速 -> 电机角度修正；
     *   电机角度按 3200 pulse/rev 转成当前水平零点相对 pulse。
     * 摄像头第二字段暂按现有整数 cm/s 读取，随后执行 0.30/0.70 低通。
     * 最终目标仍受 405 degree/s、20.25 degree/帧以及 -340/+330 pulse 保护。
     */
    memset((void *)&g_balance_tuning, 0, sizeof(g_balance_tuning));

    g_balance_tuning.kp = BALANCE_POSITION_PID_KP_DEFAULT;
    g_balance_tuning.ki = BALANCE_POSITION_PID_KI_DEFAULT;
    g_balance_tuning.kd = BALANCE_POSITION_PID_KD_DEFAULT;
    g_balance_tuning.command_centering_gain_s = 20.0f;
    g_balance_tuning.velocity_old_weight =
        BALANCE_VELOCITY_OLD_WEIGHT_DEFAULT;

    /* 以下两个旧字段只为 Watch/上位机兼容保留，串级控制不读取。 */
    g_balance_tuning.integral_window_mm = 15.0f;
    g_balance_tuning.integral_output_limit_pulse = 8.0f;

    /* F4-MIX 位置环/速度环输出及积分限幅，单位保持 cm/s 与 degree。 */
    g_balance_tuning.position_pid_max_out_cm_s =
        BALANCE_POSITION_PID_MAX_OUT_CM_S_DEFAULT;
    g_balance_tuning.position_pid_max_iout_cm_s =
        BALANCE_POSITION_PID_MAX_IOUT_CM_S_DEFAULT;
    g_balance_tuning.position_integral_deadband_cm =
        BALANCE_POSITION_INTEGRAL_DEADBAND_CM_DEFAULT;
    g_balance_tuning.velocity_pid_kp = BALANCE_VELOCITY_PID_KP_DEFAULT;
    g_balance_tuning.velocity_pid_ki = BALANCE_VELOCITY_PID_KI_DEFAULT;
    g_balance_tuning.velocity_pid_kd = BALANCE_VELOCITY_PID_KD_DEFAULT;
    g_balance_tuning.velocity_pid_max_out_deg =
        BALANCE_VELOCITY_PID_MAX_OUT_DEG_DEFAULT;
    g_balance_tuning.velocity_pid_max_iout_deg =
        BALANCE_VELOCITY_PID_MAX_IOUT_DEG_DEFAULT;

    /*
     * PA0 轨迹的正向折返点。HOLD_ORIGIN 目标仍固定为 0 mm，不读取该字段。
     */
    g_balance_tuning.positive_target_mm = 50;

    /*
     * PA0 轨迹的最终保持点。到达稳定窗后进入 HOLD_NEGATIVE 并持续闭环保持。
     */
    g_balance_tuning.negative_target_mm = -50;

    /*
     * 负向目标位置边界，单位 pulse。它必须严格等于
     * MOTOR_HARD_MIN_PULSE（-340，右端）；最终发送层会再次执行
     * 同一机械硬门检查。该边界依赖“摆杆水平位置已正确清零”和已确认的方向映射。
     * 它不是常规调参项，禁止为了获得更大动作绕过或扩大。
     */
    g_balance_tuning.soft_min_pulse = MOTOR_HARD_MIN_PULSE;

    /*
     * 正向目标位置边界，单位 pulse。它必须严格等于
     * MOTOR_HARD_MAX_PULSE（+330，左端），否则 tuning_error_mask 的 bit9 置位。
     * 该值属于最终机械安全边界而非 PID 增益；首次换机构、改传动或重设零点后，
     * 必须重新做低速、无球、可断电的机械验证，不能直接沿用旧机构结论。
     */
    g_balance_tuning.soft_max_pulse = MOTOR_HARD_MAX_PULSE;

    /*
     * 摆杆绝对目标的最大跟随速度，单位 pulse/s；控制输出在正负方向均受此限幅。
     * STOP_RETURN_ZERO 使用独立的 700 pulse/s、35 pulse/帧安全回零上限，不会因为
     * 稳球需要快速制动而同步变得激进。
     *
     * 增大：允许更快建立倾角，缩短大误差阶段时间；过大时机械冲击、过冲和通信
     *       目标堆积风险增大。
     * 减小：动作更柔和，但可能达不到 5 s 时间要求或无法及时制动外扰。
     * F4-MIX 的 405 degree/s 按 3200 pulse/rev 换算后正好是 3600 pulse/s。
     */
    g_balance_tuning.max_slew_pulse_per_s = 3600.0f;

    /*
     * 单次摄像头有效采样允许改变的最大绝对位置量，单位 pulse/帧。
     * 实际有效速度上限为 min(max_slew_pulse_per_s, max_step_pulse/dt_s)。当前视觉
     * 周期约 50～60 ms，F4-MIX 的 20.25 degree/帧换算后正好是 180 pulse/帧。
     *
     * 增大：单帧响应更强，但摄像头抖动或异常 dt 时可能产生较大机械跳变。
     * 减小：命令更细腻，但过小会重新形成制动相位滞后。固件不限制取值范围，
     * 最终目标仍由 -340/+330 pulse 硬边界兜底。
     */
    g_balance_tuning.max_step_pulse = 180;

    /*
     * 首次进入 MOVE_POSITIVE 后的正向软启动时长，单位 ms。
     * 在该时间内，只把“继续向右加速”的输出从 BALANCE_START_RAMP_MIN_SCALE 逐渐放大
     * 到 100%；若 D 项已经要求反向制动，则制动力不会被软启动削弱。
     *
     * 增大：起步更柔和、机械冲击更小，但首段响应变慢。
     * 减小：更快建立倾角，但容易起步过猛。0 表示关闭；固件不限制非零取值范围。
     */
    g_balance_tuning.initial_motion_ramp_ms = 0U;

    /*
     * 正向距离减速区宽度，单位 mm。进入该区后，P 绝对目标会从远端 52 pulse
     * 逐步降到近端 26 pulse，同时继续限制向 +x 推进的目标变化率；D 制动不削弱。
     * 增大：更早减速，通常减小 +50 mm 过冲，但可能提前变慢、到点时间增加。
     * 减小：更晚减速，响应快但过冲风险上升。固件不限制取值范围。
     */
    g_balance_tuning.positive_decel_start_mm = 40;

    /*
     * 正向减速区最靠近目标时，仍允许“继续向 +x 推进”的最低速度上限，单位 pulse/s。
     * 减小：端点附近更慢、制动更保守，但过小会爬行甚至难以进入到达窗口。
     * 增大：更容易快速触达目标，但 +50 mm 过冲可能增大；固件不再强制它小于 max_slew。
     * 调法：先用 decel_start 决定何时减速，再用 near_rate 微调端点最后一段速度。
     */
    g_balance_tuning.positive_near_rate_pulse_s = 25.0f;

    /*
     * 负向距离减速区宽度，单位 mm。进入该区后，P 绝对目标会从远端 52 pulse
     * 逐步降到近端 26 pulse，并继续限制向 -x 推进的目标变化率。85 mm 使减速
     * 从小球约位于 +35 mm 时开始，而不是等高速经过 O 点后才处理。
     * 增大：更早减速、负端过冲通常减小；过大时折返后大部分路程都受速度包络限制。
     * 减小：负向到达更快，但可能高速穿过 -50 mm。固件不限制取值范围。
     */
    g_balance_tuning.negative_decel_start_mm = 85;

    /*
     * 负向目标附近继续向 -x 推进的最低速度上限，单位 pulse/s。
     * 减小会增强最终慢速靠近和稳定能力，但太小会拖长完成时间；增大能缩短到达时间，
     * 但会增加 -50 mm 最低位置过冲和来回摆动；固件不再强制它小于 max_slew。
     */
    g_balance_tuning.negative_near_rate_pulse_s = 18.0f;

    /*
     * 启动门控使用的摄像头有效坐标新鲜度，单位 ms；最近有效坐标超过该时间即视为
     * 启动数据陈旧，START/自动启动会被拒绝。它主要影响启动阶段，不等同于运行中
     * 的 vision_continue_ms；进入 VISION_HOLD 后会一直等待下一帧有效坐标。
     * 增大：可容忍视觉周期抖动，但可能拿较旧坐标启动；减小：更严格，但帧率稍慢时
     * 容易无法 start_ready。应略大于摄像头正常最大帧间隔，固件不限制取值范围。
     */
    g_balance_tuning.camera_fresh_ms = 120U;

    /*
     * 兼容旧 Watch/遥测的历史字段。当前上电流程由人工保证机构水平，小球可位于
     * 任意有效视觉坐标，因此启动与清零状态机不再读取本字段。
     */
    g_balance_tuning.start_position_tolerance_mm = 10;

    /*
     * 兼容旧 Watch/遥测的历史字段。当前启动只要求连续有效视觉帧，不再以小球速度
     * 判断机构是否水平；上电时的真实水平由人工保证。
     */
    g_balance_tuning.start_speed_tolerance_mm_s = 20.0f;

    /*
     * 启动前要求连续满足“摄像头在线、识别到小球且坐标新鲜”的有效采样数。
     * 增大：可过滤偶发正确帧，启动更可靠，但等待时间更长；减小：启动更快但抗噪差。
     * 例如 20 Hz 视觉下，1 帧约 50 ms，3 帧约 150 ms。固件不限制取值范围。
     */
    g_balance_tuning.start_confirm_samples = 3U;

    /*
     * PA0/评分轨迹只能在 HOLD_ORIGIN 中、连续 3 个正常视觉样本均满足
     * |x|<=10 mm、|v|<=20 mm/s 时接受。按键不满足条件时只记录拒绝，不锁存请求。
     */
    g_balance_tuning.trajectory_start_position_tolerance_mm = 10;
    g_balance_tuning.trajectory_start_speed_tolerance_mm_s = 20.0f;
    g_balance_tuning.trajectory_start_confirm_samples = 3U;

    /*
     * 正向低速折返的位置窗口，单位 mm。球进入
     * [positive_target-position_tolerance, positive_target] 且速度足够低时累计确认；
     * 实际使用时还会钳位到官方 +/-10 mm 范围，防止 Watch 误设导致过早折返。
     * 增大：允许更早折返，通常能减小过冲，但转向点可能离 +50 mm 更远。
     * 减小：端点更精确，但可能必须到达/越过目标后才折返。固件不限制取值范围。
     */
    g_balance_tuning.positive_position_tolerance_mm =
        BALANCE_OFFICIAL_ERROR_MM;

    /*
     * 正向低速窗口允许的最大绝对小球速度，单位 mm/s。
     * 仅用于官方位置窗口内的连续低速确认；150 ms 预测已经判断会到达 +50 mm 时，
     * 会直接折返而不再等待速度下降。
     * 增大：更容易提前切换负目标、可减少越界等待，但可能在速度仍高时过早转向。
     * 减小：要求更慢才提前折返，判定更稳但可能拖到越过目标。固件不限制取值范围。
     */
    g_balance_tuning.positive_speed_tolerance_mm_s = 15.0f;

    /*
     * 正向低速窗口需要连续满足的位置/速度样本数，单位帧。
     * 增大：可防止单帧噪声导致提前折返，但会增加确认延迟和过冲；减小：折返更快，
     * 但更容易被偶发坐标触发。预测位置达到正目标时计数会直接置满，不受此延迟限制。
     * 20 Hz 下 2 帧约为 100 ms。固件不限制取值范围。
     */
    g_balance_tuning.positive_confirm_samples = 2U;

    /*
     * 最终 -50 mm 稳定判定的位置窗口，单位 mm，即要求 |error| 不超过该值。
     * 增大：更容易进入稳定计时，但完成点精度下降；减小：精度更高，但摄像头噪声、
     * 机械摩擦可能导致计时反复清零。比赛官方 ±10 mm 判定仍由独立常量复核，
     * 不应靠放宽该窗口掩盖控制性能。固件不限制取值范围。
     */
    g_balance_tuning.negative_position_tolerance_mm = 5;

    /*
     * 最终负目标稳定判定允许的最大绝对速度，单位 mm/s。
     * 增大：球仍有较明显运动时也可能开始稳定计时，容易过早宣布完成。
     * 减小：要求更接近静止，完成更可信，但噪声或轻微摆动会使计时难以连续。
     * 应结合 velocity_filtered_mm_s 的静止噪声底选择。固件不限制取值范围。
     */
    g_balance_tuning.negative_speed_tolerance_mm_s = 12.0f;

    /*
     * 小球同时满足负端位置窗口和速度窗口后，必须连续保持的时间，单位 ms。
     * 中途任一条件不满足就重新计时。
     * 增大：完成判定更稳健，但总用时增加，可能影响 5 s 评分。
     * 减小：更快完成，但可能把一次短暂停顿误认为稳定。固件不限制取值范围。
     */
    g_balance_tuning.negative_stable_ms = 300U;

    /*
     * PA0 状态机保留独立倍率入口；默认全部为 1.0，即与 F4-MIX 双环参数一致。
     */
    g_balance_tuning.trajectory_kp_scale =
        BALANCE_TRAJECTORY_KP_SCALE_DEFAULT;
    g_balance_tuning.trajectory_ki_scale =
        BALANCE_TRAJECTORY_KI_SCALE_DEFAULT;
    g_balance_tuning.trajectory_kd_scale =
        BALANCE_TRAJECTORY_KD_SCALE_DEFAULT;
    g_balance_tuning.trajectory_follow_gain_scale = 1.0f;
    g_balance_tuning.trajectory_slew_scale =
        BALANCE_TRAJECTORY_SLEW_SCALE_DEFAULT;
    g_balance_tuning.trajectory_step_scale =
        BALANCE_TRAJECTORY_STEP_SCALE_DEFAULT;

    /*
     * 连续收到 N/非法帧达到多少帧后，从运动状态进入 VISION_HOLD 并冻结新控制输出。
     * 有效坐标会把连续计数清零。它与 vision_continue_ms 是并行保护：按帧计数或按
     * 最后有效坐标时间，只要任一先达到条件就会冻结。
     * 增大：容忍短时识别闪断，但球失控后继续沿用旧目标的时间可能更长。
     * 减小：更快冻结、更保守，但偶发漏检容易打断控制。20 Hz 下 8 帧约 400 ms；
     * 固件不限制取值范围。
     */
    g_balance_tuning.vision_lost_confirm_frames = 8U;

    /*
     * 从最后一帧有效坐标起，允许运动状态继续保持最近目标的最长时间，单位 ms。
     * 超时后立即进入 VISION_HOLD，停止根据过期视觉继续产生新位置命令；若只是收到
     * 连续 N 帧，也可能由 vision_lost_confirm_frames 更早触发冻结。
     * 增大：短暂串口/识别间断不易打断动作，但盲行风险上升。
     * 减小：更快冻结，但对帧率抖动更敏感。固件不限制取值范围。
     */
    g_balance_tuning.vision_continue_ms = 400U;

    /*
     * 兼容旧 Watch 的历史字段，当前控制逻辑不再读取。设置为 0 明确表示：进入
     * VISION_HOLD 后没有最终故障超时，无论丢失多久都冻结新控制并等待下一帧有效坐标。
     */
    g_balance_tuning.vision_hold_ms = 0U;

    /*
     * 双档 HOLD_ORIGIN：原点附近继续使用上面的 1.8/0.30/1.10 参数；只有位置超过
     * 55 mm，或小球速度超过 140 mm/s 时锁定远距离档。远距离档降低 P/D 和目标
     * 跟随强度，并把 P 分项及总绝对目标限制在机械硬边界以内，避免大外扰时反复
     * 打满 -340/+330 pulse。返回近端档必须连续 10 个正常样本同时满足
     * |position|<=30 mm、|velocity|<=50 mm/s，因而不会在高速穿过 O 点时硬切回，
     * 同时避免远端档在约 22～26 mm 的低速静差处无法进入近端积分区。
     */
    /* 远端兼容字段也与 F4-MIX 位置环保持一致，避免 Watch 中出现第二套 PID。 */
    g_balance_tuning.far_kp = BALANCE_POSITION_PID_KP_DEFAULT;
    g_balance_tuning.far_ki = BALANCE_POSITION_PID_KI_DEFAULT;
    g_balance_tuning.far_kd = BALANCE_POSITION_PID_KD_DEFAULT;
    g_balance_tuning.far_command_centering_gain_s = 15.0f;
    g_balance_tuning.far_max_slew_pulse_per_s = 2200.0f;
    g_balance_tuning.far_p_target_limit_pulse = 100.0f;
    g_balance_tuning.far_total_target_limit_pulse = 165.0f;
    g_balance_tuning.far_enter_speed_mm_s = 140.0f;
    g_balance_tuning.far_exit_speed_mm_s = 50.0f;
    g_balance_tuning.far_max_step_pulse = 110;
    g_balance_tuning.far_enter_position_mm = 55;
    g_balance_tuning.far_exit_position_mm = 30;
    g_balance_tuning.far_exit_confirm_samples = 10U;

    /*
     * 默认参数装载后立即建立第一份运行快照。运行时复核只拒绝 NaN/Inf，并保证
     * 最终位置边界仍严格等于 MOTOR_HARD_MIN/MAX_PULSE。
     */
    memcpy(&s_tuning, (const void *)&g_balance_tuning, sizeof(s_tuning));
    s_tuning_valid = 1U;
    g_balance_debug.tuning_accept_count = 1U;
}

static uint32_t Balance_ValidateTuning(const BalanceTuning_t *tuning)
{
    uint32_t error = 0U;

    /* PID 增益不设固件范围；拒绝 NaN/Inf，并校验远端档结构参数必须逻辑成立。 */
    if (Balance_FloatValid(tuning->kp) == 0U) error |= (1UL << 0);
    if (Balance_FloatValid(tuning->ki) == 0U) error |= (1UL << 1);
    if (Balance_FloatValid(tuning->kd) == 0U) error |= (1UL << 2);
    if ((Balance_FloatValid(tuning->velocity_old_weight) == 0U) ||
        (tuning->velocity_old_weight < 0.0f) ||
        (tuning->velocity_old_weight >= 1.0f)) error |= (1UL << 3);
    if (Balance_FloatValid(tuning->integral_window_mm) == 0U) error |= (1UL << 4);
    if (Balance_FloatValid(tuning->integral_output_limit_pulse) == 0U) error |= (1UL << 5);
    if ((Balance_FloatValid(tuning->position_pid_max_out_cm_s) == 0U) ||
        (Balance_FloatValid(tuning->position_pid_max_iout_cm_s) == 0U) ||
        (Balance_FloatValid(tuning->velocity_pid_max_out_deg) == 0U) ||
        (Balance_FloatValid(tuning->velocity_pid_max_iout_deg) == 0U) ||
        (tuning->position_pid_max_out_cm_s <= 0.0f) ||
        (tuning->position_pid_max_iout_cm_s < 0.0f) ||
        (tuning->velocity_pid_max_out_deg <= 0.0f) ||
        (tuning->velocity_pid_max_iout_deg < 0.0f)) error |= (1UL << 5);
    if ((Balance_FloatValid(tuning->position_integral_deadband_cm) == 0U) ||
        (tuning->position_integral_deadband_cm < 0.0f)) error |= (1UL << 4);
    if (Balance_FloatValid(tuning->velocity_pid_kp) == 0U) error |= (1UL << 0);
    if (Balance_FloatValid(tuning->velocity_pid_ki) == 0U) error |= (1UL << 1);
    if (Balance_FloatValid(tuning->velocity_pid_kd) == 0U) error |= (1UL << 2);
    if (Balance_FloatValid(tuning->far_kp) == 0U) error |= (1UL << 6);
    if (Balance_FloatValid(tuning->far_ki) == 0U) error |= (1UL << 7);
    if (Balance_FloatValid(tuning->far_kd) == 0U) error |= (1UL << 11);
    if ((Balance_FloatValid(tuning->far_command_centering_gain_s) == 0U) ||
        (tuning->far_command_centering_gain_s <= 0.0f)) error |= (1UL << 12);
    if ((Balance_FloatValid(tuning->far_max_slew_pulse_per_s) == 0U) ||
        (tuning->far_max_slew_pulse_per_s <= 0.0f)) error |= (1UL << 13);
    if ((Balance_FloatValid(tuning->far_enter_speed_mm_s) == 0U) ||
        (tuning->far_enter_speed_mm_s <= 0.0f)) error |= (1UL << 15);
    if ((Balance_FloatValid(tuning->far_exit_speed_mm_s) == 0U) ||
        (tuning->far_exit_speed_mm_s <= 0.0f)) error |= (1UL << 16);
    if ((Balance_FloatValid(tuning->far_p_target_limit_pulse) == 0U) ||
        (tuning->far_p_target_limit_pulse <= 0.0f)) error |= (1UL << 18);
    if ((Balance_FloatValid(tuning->far_total_target_limit_pulse) == 0U) ||
        (tuning->far_total_target_limit_pulse <= 0.0f)) error |= (1UL << 19);

    /* 唯一不可调的数值边界：当前机械行程 -340 / +330 pulse。 */
    if (tuning->soft_min_pulse != MOTOR_HARD_MIN_PULSE) error |= (1UL << 8);
    if (tuning->soft_max_pulse != MOTOR_HARD_MAX_PULSE) error |= (1UL << 9);
    if (Balance_FloatValid(tuning->max_slew_pulse_per_s) == 0U) error |= (1UL << 10);
    if (Balance_FloatValid(tuning->start_speed_tolerance_mm_s) == 0U) error |= (1UL << 14);
    if (Balance_FloatValid(tuning->positive_speed_tolerance_mm_s) == 0U) error |= (1UL << 17);
    if (Balance_FloatValid(tuning->negative_speed_tolerance_mm_s) == 0U) error |= (1UL << 20);
    if (Balance_FloatValid(tuning->positive_near_rate_pulse_s) == 0U) error |= (1UL << 25);
    if (Balance_FloatValid(tuning->negative_near_rate_pulse_s) == 0U) error |= (1UL << 27);
    if (Balance_FloatValid(tuning->command_centering_gain_s) == 0U) error |= (1UL << 31);
    if ((Balance_FloatValid(tuning->trajectory_kp_scale) == 0U) ||
        (Balance_FloatValid(tuning->trajectory_ki_scale) == 0U) ||
        (Balance_FloatValid(tuning->trajectory_kd_scale) == 0U) ||
        (Balance_FloatValid(tuning->trajectory_follow_gain_scale) == 0U) ||
        (Balance_FloatValid(tuning->trajectory_slew_scale) == 0U) ||
        (Balance_FloatValid(tuning->trajectory_step_scale) == 0U) ||
        (tuning->trajectory_kp_scale <= 0.0f) ||
        (tuning->trajectory_ki_scale <= 0.0f) ||
        (tuning->trajectory_kd_scale <= 0.0f) ||
        (tuning->trajectory_follow_gain_scale <= 0.0f) ||
        (tuning->trajectory_slew_scale <= 0.0f) ||
        (tuning->trajectory_step_scale <= 0.0f)) error |= (1UL << 26);

    if ((tuning->trajectory_start_position_tolerance_mm <= 0) ||
        (Balance_FloatValid(tuning->trajectory_start_speed_tolerance_mm_s) == 0U) ||
        (tuning->trajectory_start_speed_tolerance_mm_s <= 0.0f) ||
        (tuning->trajectory_start_confirm_samples == 0U)) error |= (1UL << 28);
    if ((tuning->positive_target_mm <= 0) ||
        (tuning->negative_target_mm >= 0) ||
        (tuning->positive_target_mm > CAMERA_X_PHYSICAL_LIMIT_MM) ||
        (tuning->negative_target_mm < -CAMERA_X_PHYSICAL_LIMIT_MM)) error |= (1UL << 29);
    if ((tuning->positive_position_tolerance_mm <= 0) ||
        (tuning->positive_confirm_samples == 0U) ||
        (tuning->negative_position_tolerance_mm <= 0) ||
        (tuning->negative_stable_ms == 0U) ||
        (Balance_FloatValid(tuning->positive_speed_tolerance_mm_s) == 0U) ||
        (Balance_FloatValid(tuning->negative_speed_tolerance_mm_s) == 0U) ||
        (tuning->positive_speed_tolerance_mm_s <= 0.0f) ||
        (tuning->negative_speed_tolerance_mm_s <= 0.0f) ||
        (tuning->positive_decel_start_mm <= 0) ||
        (tuning->negative_decel_start_mm <= 0) ||
        (tuning->positive_near_rate_pulse_s <= 0.0f) ||
        (tuning->negative_near_rate_pulse_s <= 0.0f)) error |= (1UL << 30);

    if ((tuning->far_enter_position_mm <= tuning->far_exit_position_mm) ||
        (tuning->far_exit_position_mm < 0)) error |= (1UL << 21);
    if (tuning->far_max_step_pulse <= 0) error |= (1UL << 22);
    if (tuning->far_exit_confirm_samples == 0U) error |= (1UL << 23);
    if ((tuning->far_p_target_limit_pulse >
         tuning->far_total_target_limit_pulse) ||
        (tuning->far_total_target_limit_pulse >
         (float)MOTOR_HARD_MAX_PULSE) ||
        (tuning->far_total_target_limit_pulse >
         (float)(-MOTOR_HARD_MIN_PULSE))) error |= (1UL << 24);

    return error;
}

static void Balance_UpdateTuningSnapshot(void)
{
    BalanceTuning_t first;
    BalanceTuning_t second;
    uint32_t error;

    memcpy(&first, (const void *)&g_balance_tuning, sizeof(first));
    __DMB();
    memcpy(&second, (const void *)&g_balance_tuning, sizeof(second));
    __DMB();

    if (memcmp(&first, &second, sizeof(first)) != 0) {
        g_balance_debug.tuning_error_mask = 0x80000000UL;
        g_balance_debug.tuning_reject_count++;
        return;
    }
    error = Balance_ValidateTuning(&first);
    g_balance_debug.tuning_error_mask = error;
    if (error != 0U) {
        g_balance_debug.tuning_reject_count++;
        return;
    }
    if (memcmp(&first, &s_tuning, sizeof(first)) != 0) {
        if ((first.kp != s_tuning.kp) ||
            (first.ki != s_tuning.ki) ||
            (first.kd != s_tuning.kd) ||
            (first.velocity_pid_kp != s_tuning.velocity_pid_kp) ||
            (first.velocity_pid_ki != s_tuning.velocity_pid_ki) ||
            (first.velocity_pid_kd != s_tuning.velocity_pid_kd) ||
            (first.position_pid_max_out_cm_s !=
             s_tuning.position_pid_max_out_cm_s) ||
            (first.position_pid_max_iout_cm_s !=
             s_tuning.position_pid_max_iout_cm_s) ||
            (first.velocity_pid_max_out_deg !=
             s_tuning.velocity_pid_max_out_deg) ||
            (first.velocity_pid_max_iout_deg !=
             s_tuning.velocity_pid_max_iout_deg) ||
            (first.position_integral_deadband_cm !=
             s_tuning.position_integral_deadband_cm)) {
            Balance_ClearPid();
        }
        s_tuning = first;
        g_balance_debug.tuning_accept_count++;
    }
    s_tuning_valid = 1U;
}

static void Balance_UpdateMotorMirror(void)
{
    g_balance_debug.motor_owner = g_motor_command_debug.owner;
    g_balance_debug.motor_command_state = g_motor_command_debug.state;
    g_balance_debug.motor_request_id = g_motor_command_debug.request_id;
    g_balance_debug.motor_command_type = g_motor_command_debug.command_type;
    g_balance_debug.motor_tx_tick = g_motor_command_debug.tx_tick;
    g_balance_debug.motor_ack_deadline = g_motor_command_debug.ack_deadline;
    g_balance_debug.motor_retry_count = g_motor_command_debug.retry_count;
    g_balance_debug.motor_ack_status = g_motor_command_debug.ack_status;
    g_balance_debug.motor_tx_fail_count = g_motor_command_debug.tx_fail_count;
    g_balance_debug.motor_ack_timeout_count = g_motor_command_debug.ack_timeout_count;
    g_balance_debug.motor_ack_reject_count = g_motor_command_debug.ack_reject_count;
    g_balance_debug.motor_busy_defer_count = g_motor_command_debug.busy_defer_count;
    g_balance_debug.hard_limit_reject_count =
        g_motor_command_debug.hard_limit_reject_count;
    g_balance_debug.pending_latest_pulse =
        g_motor_command_debug.pending_latest_pulse;
    g_balance_debug.pending_latest_valid =
        g_motor_command_debug.pending_latest_valid;
    g_balance_debug.last_transmitted_pulse =
        g_motor_command_debug.last_transmitted_pulse;
    g_balance_debug.last_acked_pulse = g_motor_command_debug.last_acked_pulse;
}

/** @brief 将私有完整运行态压缩为 Keil Watch 需要的关键公开字段。 */
static void Balance_UpdatePublicDebug(void)
{
    s_public_debug->state = s_balance_runtime.state;
    s_public_debug->fault = s_balance_runtime.fault;
    s_public_debug->start_reject_reason =
        s_balance_runtime.start_reject_reason;
    s_public_debug->start_ready = s_balance_runtime.start_ready;
    s_public_debug->trajectory_ready = s_balance_runtime.trajectory_ready;
    s_public_debug->trajectory_started = s_balance_runtime.trajectory_started;
    s_public_debug->trajectory_completed =
        s_balance_runtime.completion_recorded;
    s_public_debug->trajectory_ready_count =
        s_balance_runtime.trajectory_ready_count;
    s_public_debug->pa0_consumed = s_balance_runtime.pa0_consumed;
    s_public_debug->run_elapsed_ms = s_balance_runtime.run_elapsed_ms;
    s_public_debug->completion_time_ms =
        s_balance_runtime.completion_time_ms;
    s_public_debug->score_pass = s_balance_runtime.score_pass;
    s_public_debug->positive_turn_position_mm =
        s_balance_runtime.positive_turn_position_mm;
    s_public_debug->positive_turn_abs_error_mm =
        s_balance_runtime.positive_turn_abs_error_mm;
    s_public_debug->positive_peak_position_mm =
        s_balance_runtime.positive_peak_position_mm;
    s_public_debug->positive_overshoot_mm =
        s_balance_runtime.positive_overshoot_mm;
    s_public_debug->negative_min_position_mm =
        s_balance_runtime.negative_min_position_mm;
    s_public_debug->negative_overshoot_mm =
        s_balance_runtime.negative_overshoot_mm;
    s_public_debug->negative_stable_window_max_abs_error_mm =
        s_balance_runtime.negative_stable_window_max_abs_error_mm;
    s_public_debug->official_endpoint_error_pass =
        s_balance_runtime.official_endpoint_error_pass;
    s_public_debug->official_time_pass =
        s_balance_runtime.official_time_pass;
    s_public_debug->run_fault_free = s_balance_runtime.run_fault_free;

    s_public_debug->button_raw_level = s_balance_runtime.button_raw_level;
    s_public_debug->button_debounced_level =
        s_balance_runtime.button_debounced_level;
    s_public_debug->button_release_seen =
        s_balance_runtime.button_release_seen;
    s_public_debug->target_button_raw_level =
        s_balance_runtime.target_button_raw_level;
    s_public_debug->target_button_debounced_level =
        s_balance_runtime.target_button_debounced_level;
    s_public_debug->target_button_press_count =
        s_balance_runtime.target_button_press_count;
    s_public_debug->target_button_rejected_count =
        s_balance_runtime.target_button_rejected_count;

    s_public_debug->camera_online = s_balance_runtime.camera_online;
    s_public_debug->target_found = s_balance_runtime.target_found;
    s_public_debug->last_valid_age_ms =
        s_balance_runtime.last_valid_age_ms;
    s_public_debug->sample_dt_ms = s_balance_runtime.sample_dt_ms;
    s_public_debug->position_used_mm = s_balance_runtime.position_used_mm;
    s_public_debug->velocity_filtered_mm_s =
        s_balance_runtime.velocity_filtered_mm_s;
    s_public_debug->vision_lost_streak =
        s_balance_runtime.vision_lost_streak;
    s_public_debug->vision_hold_count = s_balance_runtime.vision_hold_count;
    s_public_debug->vision_fault_count = s_balance_runtime.vision_fault_count;

    s_public_debug->target_mm = s_balance_runtime.target_mm;
    s_public_debug->error_mm = s_balance_runtime.error_mm;
    s_public_debug->drive_rate_pulse_s =
        s_balance_runtime.drive_rate_limited_pulse_s;
    s_public_debug->centering_rate_pulse_s =
        s_balance_runtime.centering_rate_pulse_s;
    s_public_debug->approach_brake_active =
        s_balance_runtime.approach_brake_active;
    s_public_debug->approach_rate_limit_pulse_s =
        s_balance_runtime.approach_rate_limit_pulse_s;
    s_public_debug->command_position_pulse =
        s_balance_runtime.command_position_pulse;
    s_public_debug->final_requested_pulse =
        s_balance_runtime.final_requested_pulse;
    s_public_debug->last_transmitted_pulse =
        s_balance_runtime.last_transmitted_pulse;
    s_public_debug->last_acked_pulse = s_balance_runtime.last_acked_pulse;
    s_public_debug->tuning_error_mask =
        s_balance_runtime.tuning_error_mask;

    s_public_debug->motor_owner = s_balance_runtime.motor_owner;
    s_public_debug->motor_command_state =
        s_balance_runtime.motor_command_state;
    s_public_debug->motor_retry_count = s_balance_runtime.motor_retry_count;
    s_public_debug->motor_tx_fail_count =
        s_balance_runtime.motor_tx_fail_count;
    s_public_debug->motor_ack_timeout_count =
        s_balance_runtime.motor_ack_timeout_count;
    s_public_debug->motor_ack_reject_count =
        s_balance_runtime.motor_ack_reject_count;
    s_public_debug->hard_limit_reject_count =
        s_balance_runtime.hard_limit_reject_count;
    s_public_debug->control_profile = s_balance_runtime.control_profile;
    s_public_debug->profile_exit_confirm_count =
        s_balance_runtime.profile_exit_confirm_count;
    s_public_debug->profile_switch_count =
        s_balance_runtime.profile_switch_count;
    s_public_debug->position_pid_output_cm_s =
        s_cascade_runtime.position_output_cm_s;
    s_public_debug->target_velocity_cm_s =
        s_cascade_runtime.target_velocity_cm_s;
    s_public_debug->velocity_pid_output_deg =
        s_cascade_runtime.velocity_output_degrees;
    s_public_debug->acceleration_feedforward_deg =
        s_cascade_runtime.feedforward_degrees;
    s_public_debug->desired_correction_deg =
        s_cascade_runtime.desired_correction_degrees;
    s_public_debug->line_track_feedforward_active =
        s_cascade_runtime.feedforward_active;
}

static void Balance_InitTargetButton(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* PE2 使用内部上拉，外部按键只需在按下时把 PE2 接到 GND。 */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    gpio.Pin = TARGET_LOCK_BUTTON_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TARGET_LOCK_BUTTON_GPIO_Port, &gpio);
}

static uint8_t Balance_UpdateButton(uint32_t now)
{
    uint8_t raw = (uint8_t)(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port,
                                             START_BUTTON_Pin) == GPIO_PIN_SET);
    uint8_t press_event = 0U;

    g_balance_debug.button_raw_level = raw;
    if (raw != s_button_raw) {
        s_button_raw = raw;
        s_button_raw_change_tick = now;
        g_balance_debug.button_debounce_tick = now;
    }
    if ((raw != s_button_debounced) &&
        ((uint32_t)(now - s_button_raw_change_tick) >=
         BALANCE_BUTTON_DEBOUNCE_MS)) {
        s_button_debounced = raw;
        g_balance_debug.button_debounced_level = raw;
        if (raw == 0U) {
            g_balance_debug.button_release_seen = 1U;
        } else {
            press_event = 1U;
            g_balance_debug.button_press_count++;
        }
    }
    if ((s_button_debounced == 0U) &&
        (g_balance_debug.button_release_seen == 0U) &&
        ((uint32_t)(now - s_button_raw_change_tick) >=
         BALANCE_BUTTON_DEBOUNCE_MS)) {
        g_balance_debug.button_release_seen = 1U;
    }
    return press_event;
}

static uint8_t Balance_UpdateTargetButton(uint32_t now)
{
    uint8_t raw;
    uint8_t press_event;

    (void)now;
    raw = bsp_track_key_is_pressed(TRACK_KEY_OK) ? 1U : 0U;
    press_event = UI_MenuTakeTargetPress();
    g_balance_debug.target_button_raw_level = raw;
    g_balance_debug.target_button_debounced_level = raw;
    if (press_event != 0U) {
        g_balance_debug.target_button_press_count++;
    }
    return press_event;
}

static void Balance_UpdateCameraMirror(const Camera_Data_t *camera, uint32_t now)
{
    g_balance_debug.camera_online = camera->online;
    g_balance_debug.target_found = camera->target_found;
    g_balance_debug.frame_sequence = camera->frame_sequence;
    g_balance_debug.valid_sequence = camera->valid_sequence;
    g_balance_debug.last_frame_age_ms =
        (camera->last_frame_tick == 0U) ? UINT32_MAX :
        (uint32_t)(now - camera->last_frame_tick);
    g_balance_debug.last_valid_age_ms =
        (camera->last_valid_tick == 0U) ? UINT32_MAX :
        (uint32_t)(now - camera->last_valid_tick);
    g_balance_debug.position_raw_mm = camera->x;
    g_balance_debug.coordinate_range_reject_count =
        g_camera_debug.physical_reject_count;
}

static BalanceSampleResult_t Balance_ProcessNewSample(const Camera_Data_t *camera)
{
    uint32_t dt_ms;
    float raw_velocity;

    if (camera->valid_sequence == g_balance_debug.processed_valid_sequence) {
        return BALANCE_SAMPLE_NONE;
    }
    g_balance_debug.processed_valid_sequence = camera->valid_sequence;
    g_balance_debug.position_used_mm = camera->x;
    raw_velocity = Balance_ClampFloat(
        (float)camera->y,
        -BALANCE_CAMERA_VELOCITY_ABS_MAX_CM_S,
        BALANCE_CAMERA_VELOCITY_ABS_MAX_CM_S) * 10.0f;

    if (s_velocity_history_valid == 0U) {
        s_last_position_mm = camera->x;
        s_last_valid_tick = camera->last_valid_tick;
        s_velocity_history_valid = 1U;
        g_balance_debug.velocity_history_valid = 1U;
        g_balance_debug.velocity_raw_mm_s = raw_velocity;
        g_balance_debug.velocity_filtered_mm_s = raw_velocity;
        g_balance_debug.sample_dt_ms = 0U;
        return BALANCE_SAMPLE_REBUILT;
    }

    dt_ms = (uint32_t)(camera->last_valid_tick - s_last_valid_tick);
    g_balance_debug.sample_dt_ms = dt_ms;
    if (dt_ms < BALANCE_SAMPLE_MIN_DT_MS) {
        g_balance_debug.fast_sample_count++;
        s_trace_pending_flags |= BALANCE_TRACE_FLAG_ABNORMAL_DT;
        return BALANCE_SAMPLE_TOO_FAST;
    }

    if (dt_ms >= BALANCE_SAMPLE_VALID_MAX_MS) {
        g_balance_debug.abnormal_dt_count++;
        s_last_position_mm = camera->x;
        s_last_valid_tick = camera->last_valid_tick;
        g_balance_debug.velocity_raw_mm_s = raw_velocity;
        g_balance_debug.velocity_filtered_mm_s = raw_velocity;
        s_trace_pending_flags |= BALANCE_TRACE_FLAG_ABNORMAL_DT;
        return BALANCE_SAMPLE_ABNORMAL;
    }

    if (dt_ms > BALANCE_SAMPLE_NORMAL_MAX_MS) {
        g_balance_debug.abnormal_dt_count++;
        s_last_position_mm = camera->x;
        s_last_valid_tick = camera->last_valid_tick;
        g_balance_debug.velocity_raw_mm_s = raw_velocity;
        g_balance_debug.velocity_filtered_mm_s = raw_velocity;
        s_trace_pending_flags |= BALANCE_TRACE_FLAG_ABNORMAL_DT;
        return BALANCE_SAMPLE_ABNORMAL;
    }

    g_balance_debug.velocity_raw_mm_s = raw_velocity;
    g_balance_debug.velocity_filtered_mm_s =
        s_tuning.velocity_old_weight *
        g_balance_debug.velocity_filtered_mm_s +
        (1.0f - s_tuning.velocity_old_weight) * raw_velocity;
    s_last_position_mm = camera->x;
    s_last_valid_tick = camera->last_valid_tick;
    return BALANCE_SAMPLE_NORMAL;
}

static void Balance_UpdateStartReady(const Camera_Data_t *camera,
                                     BalanceSampleResult_t sample,
                                     uint32_t now)
{
    if ((camera->online == 0U) || (camera->target_found == 0U) ||
        (camera->last_valid_tick == 0U) ||
        ((uint32_t)(now - camera->last_valid_tick) >
         s_tuning.camera_fresh_ms)) {
        s_start_confirm_count = 0U;
        g_balance_debug.start_ready = 0U;
        return;
    }

    if ((sample == BALANCE_SAMPLE_NONE) ||
        (sample == BALANCE_SAMPLE_TOO_FAST) ||
        (sample == BALANCE_SAMPLE_ABNORMAL)) {
        if ((sample == BALANCE_SAMPLE_TOO_FAST) ||
            (sample == BALANCE_SAMPLE_ABNORMAL)) {
            s_start_confirm_count = 0U;
        }
        g_balance_debug.start_ready =
            (uint8_t)(s_start_confirm_count >= s_tuning.start_confirm_samples);
        return;
    }

    /* 机构水平由上电前人工保证；小球在任意有效坐标均可开始水平建零。 */
    if (s_start_confirm_count < 255U) {
        s_start_confirm_count++;
    }
    g_balance_debug.start_ready =
        (uint8_t)(s_start_confirm_count >= s_tuning.start_confirm_samples);
}

static BalanceStartReject_t Balance_GetStartReject(const Camera_Data_t *camera,
                                                   uint32_t now)
{
    if (g_balance_debug.button_release_seen == 0U) {
        return BALANCE_START_REJECT_BUTTON_NOT_RELEASED_AFTER_POWER_ON;
    }
    if (s_tuning_valid == 0U) {
        return BALANCE_START_REJECT_TUNING_NOT_VALID;
    }
    if (camera->online == 0U) {
        return BALANCE_START_REJECT_CAMERA_OFFLINE;
    }
    if (camera->target_found == 0U) {
        return BALANCE_START_REJECT_TARGET_NOT_FOUND;
    }
    if ((camera->last_valid_tick == 0U) ||
        ((uint32_t)(now - camera->last_valid_tick) >
         s_tuning.camera_fresh_ms)) {
        return BALANCE_START_REJECT_COORDINATE_STALE;
    }
    if (g_balance_debug.start_ready == 0U) {
        return BALANCE_START_REJECT_TARGET_NOT_CONFIRMED;
    }
    if (Motor_CommandGetOwner() != MOTOR_OWNER_NONE) {
        return BALANCE_START_REJECT_MOTOR_OWNER_BUSY;
    }
    return BALANCE_START_REJECT_NONE;
}

/** @brief 更新评分轨迹的 O 点稳定门控；主循环空转不重复累计同一视觉帧。 */
static void Balance_UpdateTrajectoryReady(const Camera_Data_t *camera,
                                          BalanceSampleResult_t sample,
                                          uint32_t now)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    if ((state != BALANCE_STATE_HOLD_ORIGIN) ||
        (g_balance_debug.trajectory_started != 0U) ||
        (camera->online == 0U) ||
        (camera->target_found == 0U) ||
        (camera->last_valid_tick == 0U) ||
        ((uint32_t)(now - camera->last_valid_tick) >
         s_tuning.camera_fresh_ms) ||
        (Motor_CommandGetOwner() != MOTOR_OWNER_BALANCE)) {
        g_balance_debug.trajectory_ready_count = 0U;
        g_balance_debug.trajectory_ready = 0U;
        return;
    }

    if (sample == BALANCE_SAMPLE_NONE) {
        return;
    }
    if ((sample != BALANCE_SAMPLE_NORMAL) ||
        (Balance_AbsInt16(g_balance_debug.position_used_mm) >
         s_tuning.trajectory_start_position_tolerance_mm) ||
        (Balance_AbsFloat(g_balance_debug.velocity_filtered_mm_s) >
         s_tuning.trajectory_start_speed_tolerance_mm_s)) {
        g_balance_debug.trajectory_ready_count = 0U;
        g_balance_debug.trajectory_ready = 0U;
        return;
    }

    if (g_balance_debug.trajectory_ready_count < UINT8_MAX) {
        g_balance_debug.trajectory_ready_count++;
    }
    g_balance_debug.trajectory_ready = (uint8_t)
        (g_balance_debug.trajectory_ready_count >=
         s_tuning.trajectory_start_confirm_samples);
}

static BalanceStartReject_t Balance_GetTrajectoryStartReject(
    const Camera_Data_t *camera,
    uint32_t now)
{
    if (g_balance_debug.trajectory_started != 0U) {
        return BALANCE_START_REJECT_TRAJECTORY_ALREADY_USED;
    }
    if ((BalanceState_t)g_balance_debug.state != BALANCE_STATE_HOLD_ORIGIN) {
        return BALANCE_START_REJECT_NOT_HOLDING_ORIGIN;
    }
    if (s_tuning_valid == 0U) {
        return BALANCE_START_REJECT_TUNING_NOT_VALID;
    }
    if (camera->online == 0U) {
        return BALANCE_START_REJECT_CAMERA_OFFLINE;
    }
    if (camera->target_found == 0U) {
        return BALANCE_START_REJECT_TARGET_NOT_FOUND;
    }
    if ((camera->last_valid_tick == 0U) ||
        ((uint32_t)(now - camera->last_valid_tick) >
         s_tuning.camera_fresh_ms)) {
        return BALANCE_START_REJECT_COORDINATE_STALE;
    }
    if (Motor_CommandGetOwner() != MOTOR_OWNER_BALANCE) {
        return BALANCE_START_REJECT_MOTOR_OWNER_BUSY;
    }
    if (Balance_AbsInt16(g_balance_debug.position_used_mm) >
        s_tuning.trajectory_start_position_tolerance_mm) {
        return BALANCE_START_REJECT_BALL_NOT_AT_ORIGIN;
    }
    if (Balance_AbsFloat(g_balance_debug.velocity_filtered_mm_s) >
        s_tuning.trajectory_start_speed_tolerance_mm_s) {
        return BALANCE_START_REJECT_BALL_MOVING_TOO_FAST;
    }
    if (g_balance_debug.trajectory_ready == 0U) {
        return BALANCE_START_REJECT_TARGET_NOT_CONFIRMED;
    }
    return BALANCE_START_REJECT_NONE;
}

/** @brief 接受 PA0/PE2 评分轨迹请求；不重复建零、不重新使能或改变电机所有权。 */
static void Balance_TryStartTrajectory(const Camera_Data_t *camera,
                                       uint32_t now)
{
    BalanceStartReject_t reject =
        Balance_GetTrajectoryStartReject(camera, now);

    g_balance_debug.start_reject_reason = (uint8_t)reject;
    if (reject != BALANCE_START_REJECT_NONE) {
        g_balance_debug.button_rejected_count++;
        return;
    }

    Balance_ClearTrajectoryRunData(now);
    g_balance_debug.target_mm = s_tuning.positive_target_mm;
    g_balance_state.target_x_cm =
        (float)s_tuning.positive_target_mm / 10.0f;
    g_balance_state.target_locked = 1U;
    g_balance_debug.start_reject_reason = BALANCE_START_REJECT_NONE;
    Balance_SetState(BALANCE_STATE_MOVE_POSITIVE);
}

static BalanceFault_t Balance_MotorFaultFromResult(MotorRequestResult_t result,
                                                   BalanceFault_t operation_fault)
{
    if (result == MOTOR_REQUEST_TX_FAILED) {
        return BALANCE_FAULT_MOTOR_TX_FAILED;
    }
    if (result == MOTOR_REQUEST_ACK_TIMEOUT) {
        return BALANCE_FAULT_MOTOR_ACK_TIMEOUT;
    }
    if (result == MOTOR_REQUEST_DEVICE_REJECTED) {
        return BALANCE_FAULT_MOTOR_ACK_REJECTED;
    }
    if (result == MOTOR_REQUEST_HARD_LIMIT_REJECTED) {
        return BALANCE_FAULT_HARD_LIMIT_REQUEST;
    }
    return operation_fault;
}

static void Balance_LatchFault(BalanceFault_t fault, uint8_t communication_unsafe)
{
    if (g_balance_debug.fault == BALANCE_FAULT_NONE) {
        g_balance_debug.fault = (uint8_t)fault;
    }
    g_balance_debug.run_fault_free = 0U;
    if (g_balance_debug.trajectory_started != 0U) {
        g_balance_debug.score_pass = 0U;
    }
    Balance_ClearPid();
    g_balance_debug.drive_rate_raw_pulse_s = 0.0f;
    g_balance_debug.drive_rate_limited_pulse_s = 0.0f;
    s_negative_stable_start_tick = 0U;
    s_trace_pending_flags |= BALANCE_TRACE_FLAG_FAULT;
    Motor_CommandSafetyTakeover();

    if (communication_unsafe != 0U) {
        Balance_SetState(BALANCE_STATE_FAULT_LATCHED);
        s_disable_phase = 0U;
        s_disable_request_id = Motor_CommandRequestStop(MOTOR_OWNER_SAFETY);
    } else {
        s_return_is_fault = 1U;
        s_return_stop_done = 0U;
        s_return_request_id = 0U;
        s_return_last_tick = HAL_GetTick();
        Balance_SetState(BALANCE_STATE_RETURN_ZERO);
    }
}

static uint8_t Balance_CanBeginTargetLeveling(BalanceState_t state)
{
    if (Balance_StateIsStrongHold(state) != 0U) {
        return 1U;
    }
    if ((state == BALANCE_STATE_HOLD_NEGATIVE) &&
        (g_balance_debug.completion_recorded != 0U)) {
        return 1U;
    }
    if (state == BALANCE_STATE_VISION_HOLD) {
        if (Balance_StateIsStrongHold(s_resume_state) != 0U) {
            return 1U;
        }
        if ((s_resume_state == BALANCE_STATE_HOLD_NEGATIVE) &&
            (g_balance_debug.completion_recorded != 0U)) {
            return 1U;
        }
    }
    return 0U;
}

static void Balance_ResetUiTaskData(void)
{
    g_balance_debug.run_start_tick = 0U;
    g_balance_debug.run_elapsed_ms = 0U;
    g_balance_debug.trajectory_started = 0U;
    g_balance_debug.pa0_consumed = 0U;
    g_balance_debug.trajectory_ready = 0U;
    g_balance_debug.trajectory_ready_count = 0U;
    g_balance_debug.start_reject_reason = BALANCE_START_REJECT_NONE;
    Balance_ClearScoreSummary();
    s_positive_confirm_count = 0U;
    s_negative_stable_start_tick = 0U;
}

static void Balance_SelectOriginAfterUiExit(void)
{
    g_balance_debug.target_mm = BALANCE_ORIGIN_TARGET_MM;
    g_balance_state.target_x_cm = 0.0f;
    g_balance_state.target_locked = 1U;
    Balance_ClearPid();
    Balance_ResetHoldProfile();
}

/** @brief 长按退出任务：取消评分/任意点任务，并安全回到默认 0 点闭环。 */
static void Balance_HandleUiExit(void)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;
    BalanceState_t active_state = (state == BALANCE_STATE_VISION_HOLD) ?
                                  s_resume_state : state;
    uint8_t task_active = (uint8_t)
        ((g_balance_debug.trajectory_started != 0U) ||
         (active_state == BALANCE_STATE_MOVE_POSITIVE) ||
         (active_state == BALANCE_STATE_MOVE_NEGATIVE) ||
         (active_state == BALANCE_STATE_HOLD_NEGATIVE) ||
         (active_state == BALANCE_STATE_TARGET_LEVELING) ||
         (active_state == BALANCE_STATE_TARGET_WAIT_CAPTURE) ||
         (active_state == BALANCE_STATE_HOLD_CAPTURED_TARGET));

    if (task_active == 0U) {
        return;
    }

    Balance_ResetUiTaskData();
    Balance_SelectOriginAfterUiExit();

    if (state == BALANCE_STATE_TARGET_LEVELING) {
        /* 当前回水平命令/位置查询继续完成，验证到 0 后直接恢复 HOLD_ORIGIN。 */
        s_target_exit_pending = 1U;
        return;
    }

    s_target_exit_pending = 0U;
    if (state == BALANCE_STATE_VISION_HOLD) {
        /* 视觉丢失期间不产生新运动；恢复首帧转入 0 点闭环。 */
        s_resume_state = BALANCE_STATE_HOLD_ORIGIN;
    } else {
        Balance_SetState(BALANCE_STATE_HOLD_ORIGIN);
    }
}

static void Balance_BeginTargetLeveling(void)
{
    if (Motor_CommandGetOwner() != MOTOR_OWNER_BALANCE) {
        g_balance_debug.start_reject_reason =
            BALANCE_START_REJECT_MOTOR_OWNER_BUSY;
        g_balance_debug.target_button_rejected_count++;
        return;
    }

    /* 暂停稳球输出，保留 Balance 所有权，并用统一命令层把轴送回已验证的 0 pulse。 */
    Balance_ClearPid();
    Balance_ResetHoldProfile();
    g_balance_debug.p_rate_pulse_s = 0.0f;
    g_balance_debug.d_rate_pulse_s = 0.0f;
    g_balance_debug.drive_rate_raw_pulse_s = 0.0f;
    g_balance_debug.drive_rate_limited_pulse_s = 0.0f;
    g_balance_debug.approach_brake_active = 0U;
    g_balance_debug.approach_rate_limit_pulse_s = 0.0f;
    g_balance_debug.trajectory_ready = 0U;
    g_balance_debug.trajectory_ready_count = 0U;
    g_balance_debug.start_reject_reason = BALANCE_START_REJECT_NONE;
    g_balance_state.target_locked = 0U;
    s_positive_confirm_count = 0U;
    s_negative_stable_start_tick = 0U;
    s_state_request_id = 0U;
    s_target_level_phase = 0U;
    s_target_exit_pending = 0U;
    s_target_level_next_query_tick = HAL_GetTick();
    s_resume_state = BALANCE_STATE_IDLE;
    Balance_SetState(BALANCE_STATE_TARGET_LEVELING);
}

static void Balance_ProcessTargetLeveling(uint32_t now)
{
    MotorRequestResult_t result;

    if (Motor_CommandGetOwner() != MOTOR_OWNER_BALANCE) {
        Balance_LatchFault(BALANCE_FAULT_MOTOR_OWNER_LOST, 1U);
        return;
    }
    if ((uint32_t)(now - g_balance_debug.state_enter_tick) >=
        BALANCE_TARGET_LEVEL_TIMEOUT_MS) {
        Balance_LatchFault(BALANCE_FAULT_RETURN_ZERO_FAILED, 1U);
        return;
    }

    if (s_target_level_phase == 0U) {
        if (s_state_request_id == 0U) {
            s_state_request_id =
                Motor_CommandRequestAbsolute(MOTOR_OWNER_BALANCE, 0);
            if (s_state_request_id != 0U) {
                g_balance_debug.final_requested_pulse = 0;
            }
            return;
        }
        result = Motor_CommandGetResult(s_state_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) {
            return;
        }
        s_state_request_id = 0U;
        if (result != MOTOR_REQUEST_SUCCESS) {
            Balance_LatchFault(Balance_MotorFaultFromResult(
                result, BALANCE_FAULT_RETURN_ZERO_FAILED),
                (uint8_t)(result != MOTOR_REQUEST_HARD_LIMIT_REJECTED));
            return;
        }
        g_balance_debug.command_position_pulse = 0.0f;
        g_balance_debug.final_requested_pulse = 0;
        s_target_level_phase = 1U;
        s_target_level_next_query_tick = now;
        return;
    }

    if (s_state_request_id == 0U) {
        if ((int32_t)(now - s_target_level_next_query_tick) < 0) {
            return;
        }
        s_state_request_id =
            Motor_CommandRequestQueryPosition(MOTOR_OWNER_BALANCE);
        return;
    }

    result = Motor_CommandGetResult(s_state_request_id);
    if ((result == MOTOR_REQUEST_PENDING) ||
        (result == MOTOR_REQUEST_UNKNOWN)) {
        return;
    }
    s_state_request_id = 0U;
    if (result != MOTOR_REQUEST_SUCCESS) {
        Balance_LatchFault(Balance_MotorFaultFromResult(
            result, BALANCE_FAULT_RETURN_ZERO_FAILED), 1U);
        return;
    }
    if (g_motor_feedback.position_valid == 0U) {
        Balance_LatchFault(BALANCE_FAULT_MOTOR_QUERY_INVALID, 1U);
        return;
    }
    if (Balance_AbsFloat(g_motor_feedback.position_degrees) <=
        BALANCE_ZERO_VERIFY_DEG) {
        s_velocity_history_valid = 0U;
        g_balance_debug.velocity_history_valid = 0U;
        g_balance_debug.velocity_raw_mm_s = 0.0f;
        g_balance_debug.velocity_filtered_mm_s = 0.0f;
        if (s_target_exit_pending != 0U) {
            s_target_exit_pending = 0U;
            Balance_SelectOriginAfterUiExit();
            Balance_SetState(BALANCE_STATE_HOLD_ORIGIN);
        } else {
            s_target_capture_baseline_valid_sequence =
                g_balance_debug.valid_sequence;
            Balance_SetState(BALANCE_STATE_TARGET_WAIT_CAPTURE);
        }
        return;
    }
    s_target_level_next_query_tick = now + BALANCE_TARGET_LEVEL_QUERY_MS;
}

static void Balance_TryCaptureTarget(const Camera_Data_t *camera, uint32_t now)
{
    if (Motor_CommandGetOwner() != MOTOR_OWNER_BALANCE) {
        Balance_LatchFault(BALANCE_FAULT_MOTOR_OWNER_LOST, 1U);
        return;
    }
    if (camera->online == 0U) {
        g_balance_debug.start_reject_reason =
            BALANCE_START_REJECT_CAMERA_OFFLINE;
        g_balance_debug.target_button_rejected_count++;
        return;
    }
    if (camera->target_found == 0U) {
        g_balance_debug.start_reject_reason =
            BALANCE_START_REJECT_TARGET_NOT_FOUND;
        g_balance_debug.target_button_rejected_count++;
        return;
    }
    if ((camera->last_valid_tick == 0U) ||
        ((uint32_t)(now - camera->last_valid_tick) >
         s_tuning.camera_fresh_ms)) {
        g_balance_debug.start_reject_reason =
            BALANCE_START_REJECT_COORDINATE_STALE;
        g_balance_debug.target_button_rejected_count++;
        return;
    }
    if (camera->valid_sequence ==
        s_target_capture_baseline_valid_sequence) {
        /* 必须至少看到一帧“轴已水平之后”的新坐标，禁止锁入回水平前的旧位置。 */
        g_balance_debug.start_reject_reason =
            BALANCE_START_REJECT_TARGET_NOT_CONFIRMED;
        g_balance_debug.target_button_rejected_count++;
        return;
    }

    /* 当前有效摄像头毫米坐标只写入运行态；掉电后仍恢复默认锁 0。 */
    g_balance_debug.target_mm = camera->x;
    g_balance_state.target_x_cm = (float)camera->x / 10.0f;
    g_balance_state.target_locked = 1U;
    g_balance_debug.start_reject_reason = BALANCE_START_REJECT_NONE;
    g_balance_debug.position_used_mm = camera->x;
    g_balance_debug.processed_valid_sequence = camera->valid_sequence;
    g_balance_debug.velocity_raw_mm_s = 0.0f;
    g_balance_debug.velocity_filtered_mm_s = 0.0f;
    g_balance_debug.velocity_history_valid = 1U;
    g_balance_debug.vision_lost_streak = 0U;
    s_last_position_mm = camera->x;
    s_last_valid_tick = camera->last_valid_tick;
    s_velocity_history_valid = 1U;
    s_last_frame_sequence = camera->frame_sequence;
    s_last_physical_reject_count = g_camera_debug.physical_reject_count;
    Balance_ClearPid();
    Balance_ResetHoldProfile();
    Balance_SetState(BALANCE_STATE_HOLD_CAPTURED_TARGET);
}

static void Balance_TryStart(const Camera_Data_t *camera,
                             uint32_t now,
                             uint8_t from_pa0)
{
    BalanceStartReject_t reject = Balance_GetStartReject(camera, now);

    g_balance_debug.start_reject_reason = (uint8_t)reject;
    if (reject != BALANCE_START_REJECT_NONE) {
        if (from_pa0 != 0U) {
            g_balance_debug.button_rejected_count++;
        }
        return;
    }
    if (Motor_CommandAcquire(MOTOR_OWNER_BALANCE) == 0U) {
        g_balance_debug.start_reject_reason =
            BALANCE_START_REJECT_MOTOR_OWNER_BUSY;
        if (from_pa0 != 0U) {
            g_balance_debug.button_rejected_count++;
        }
        return;
    }

    if (from_pa0 != 0U) {
        g_balance_debug.pa0_consumed = 1U;
    }
    Balance_ClearRunData();
    g_balance_debug.run_start_tick = now;
    g_balance_debug.target_mm = BALANCE_ORIGIN_TARGET_MM;
    g_balance_state.target_x_cm = 0.0f;
    g_balance_state.target_locked = 1U;
    g_balance_debug.start_reject_reason = BALANCE_START_REJECT_NONE;
    Balance_SetState(BALANCE_STATE_START_QUERY_MOTOR);
}

static uint8_t Balance_StartupVisionSafe(const Camera_Data_t *camera,
                                         uint32_t now)
{
    /* 清零/使能期间允许少量 N；任意位置的最近有效坐标仍新鲜即可继续。 */
    return (uint8_t)((camera->last_valid_tick != 0U) &&
                     ((uint32_t)(now - camera->last_valid_tick) <=
                       s_tuning.vision_continue_ms));
}

static void Balance_ProcessStartup(const Camera_Data_t *camera, uint32_t now)
{
    MotorRequestResult_t result;
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    if (Motor_CommandGetOwner() != MOTOR_OWNER_BALANCE) {
        Balance_LatchFault(BALANCE_FAULT_MOTOR_OWNER_LOST, 1U);
        return;
    }
    if (Balance_StartupVisionSafe(camera, now) == 0U) {
        /* 建零/使能期间视觉暂时中断时停留在当前安全阶段，收到新有效帧后继续。 */
        return;
    }

    if (state == BALANCE_STATE_START_QUERY_MOTOR) {
        if (s_state_request_id == 0U) {
            s_state_request_id =
                Motor_CommandRequestQueryPosition(MOTOR_OWNER_BALANCE);
            return;
        }
        result = Motor_CommandGetResult(s_state_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) return;
        s_state_request_id = 0U;
        if ((result != MOTOR_REQUEST_SUCCESS) ||
            (g_motor_feedback.position_valid == 0U)) {
            Balance_LatchFault(
                (result == MOTOR_REQUEST_SUCCESS) ?
                BALANCE_FAULT_MOTOR_QUERY_INVALID :
                Balance_MotorFaultFromResult(result,
                                             BALANCE_FAULT_MOTOR_QUERY_INVALID),
                1U);
            return;
        }
        s_start_zero_phase = 0U;
        Balance_SetState(BALANCE_STATE_START_SET_ZERO);
        return;
    }

    if (state == BALANCE_STATE_START_SET_ZERO) {
        /* MCU Reset 后驱动器可能仍保持使能；清零前先明确发送失能并等待 ACK。 */
        if (s_start_zero_phase == 0U) {
            if (s_state_request_id == 0U) {
                s_state_request_id =
                    Motor_CommandRequestDisable(MOTOR_OWNER_BALANCE);
                return;
            }
            result = Motor_CommandGetResult(s_state_request_id);
            if ((result == MOTOR_REQUEST_PENDING) ||
                (result == MOTOR_REQUEST_UNKNOWN)) return;
            s_state_request_id = 0U;
            if (result != MOTOR_REQUEST_SUCCESS) {
                Balance_LatchFault(Balance_MotorFaultFromResult(
                    result, BALANCE_FAULT_MOTOR_ZERO_FAILED), 1U);
                return;
            }
            s_start_zero_phase = 1U;
        }
        if (s_state_request_id == 0U) {
            s_state_request_id =
                Motor_CommandRequestResetZero(MOTOR_OWNER_BALANCE);
            return;
        }
        result = Motor_CommandGetResult(s_state_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) return;
        s_state_request_id = 0U;
        if (result != MOTOR_REQUEST_SUCCESS) {
            Balance_LatchFault(Balance_MotorFaultFromResult(
                result, BALANCE_FAULT_MOTOR_ZERO_FAILED), 1U);
            return;
        }
        Balance_SetState(BALANCE_STATE_START_VERIFY_ZERO);
        return;
    }

    if (state == BALANCE_STATE_START_VERIFY_ZERO) {
        if (s_state_request_id == 0U) {
            s_state_request_id =
                Motor_CommandRequestQueryPosition(MOTOR_OWNER_BALANCE);
            return;
        }
        result = Motor_CommandGetResult(s_state_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) return;
        s_state_request_id = 0U;
        if ((result != MOTOR_REQUEST_SUCCESS) ||
            (g_motor_feedback.position_valid == 0U) ||
            (Balance_AbsFloat(g_motor_feedback.position_degrees) >
             BALANCE_ZERO_VERIFY_DEG)) {
            Balance_LatchFault(
                (result == MOTOR_REQUEST_SUCCESS) ?
                BALANCE_FAULT_MOTOR_ZERO_VERIFY_FAILED :
                Balance_MotorFaultFromResult(
                    result, BALANCE_FAULT_MOTOR_ZERO_VERIFY_FAILED),
                1U);
            return;
        }
        g_motor_command_debug.last_transmitted_pulse = 0;
        g_motor_command_debug.last_acked_pulse = 0;
        g_balance_debug.command_position_pulse = 0.0f;
        g_balance_debug.final_requested_pulse = 0;
        /* 启动阶段累计的摄像头帧不属于运动丢帧统计，从此刻重新建立基线。 */
        s_last_frame_sequence = camera->frame_sequence;
        s_last_physical_reject_count = g_camera_debug.physical_reject_count;
        g_balance_debug.vision_lost_streak = 0U;
        Balance_SetState(BALANCE_STATE_START_ENABLE);
        return;
    }

    if (state == BALANCE_STATE_START_ENABLE) {
        if (s_state_request_id == 0U) {
            s_state_request_id =
                Motor_CommandRequestEnable(MOTOR_OWNER_BALANCE);
            return;
        }
        result = Motor_CommandGetResult(s_state_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) return;
        s_state_request_id = 0U;
        if (result != MOTOR_REQUEST_SUCCESS) {
            Balance_LatchFault(Balance_MotorFaultFromResult(
                result, BALANCE_FAULT_MOTOR_ENABLE_FAILED), 1U);
            return;
        }
        (void)Motor_CommandRequestAbsolute(MOTOR_OWNER_BALANCE, 0);
        g_balance_debug.command_position_pulse = 0.0f;
        g_balance_debug.final_requested_pulse = 0;
        /* 从现在起目标始终是 O 点；外力拨球后由同一 P/I/D 速度环主动拉回。 */
        Balance_ClearPid();
        g_balance_debug.target_mm = BALANCE_ORIGIN_TARGET_MM;
        g_balance_state.target_x_cm = 0.0f;
        g_balance_state.target_locked = 1U;
        Balance_SetState(BALANCE_STATE_HOLD_ORIGIN);
    }
}

static void Balance_TraceSample(void)
{
    BalanceTracePoint_t *point;

    if (g_balance_debug.run_start_tick == 0U) {
        return;
    }
    if (g_balance_trace.count >= BALANCE_TRACE_CAPACITY) {
        g_balance_trace.full = 1U;
        g_balance_trace.overflow = 1U;
        return;
    }
    point = (BalanceTracePoint_t *)&g_balance_trace.point[g_balance_trace.count];
    point->elapsed_ms = g_balance_debug.run_elapsed_ms;
    point->state = g_balance_debug.state;
    point->flags = s_trace_pending_flags;
    point->target_mm = g_balance_debug.target_mm;
    point->position_mm = g_balance_debug.position_used_mm;
    point->velocity_mm_s = g_balance_debug.velocity_filtered_mm_s;
    point->p_rate_pulse_s = g_balance_debug.p_rate_pulse_s;
    point->i_rate_pulse_s = g_balance_debug.i_rate_pulse_s;
    point->d_rate_pulse_s = g_balance_debug.d_rate_pulse_s;
    point->centering_rate_pulse_s =
        g_balance_debug.centering_rate_pulse_s;
    point->raw_rate_pulse_s = g_balance_debug.drive_rate_raw_pulse_s;
    point->limited_rate_pulse_s =
        g_balance_debug.drive_rate_limited_pulse_s;
    point->command_position_pulse =
        g_balance_debug.command_position_pulse;
    point->final_pulse = (int16_t)g_balance_debug.final_requested_pulse;
    g_balance_trace.count++;
    if (g_balance_trace.count >= BALANCE_TRACE_CAPACITY) {
        g_balance_trace.full = 1U;
    }
    s_trace_pending_flags = 0U;
}

static void Balance_UpdateScoringBeforeControl(int16_t position)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    if ((g_balance_debug.trajectory_started == 0U) ||
        (g_balance_debug.completion_recorded != 0U)) {
        return;
    }
    if (g_balance_debug.positive_peak_position_mm == INT16_MIN) {
        g_balance_debug.positive_peak_position_mm = position;
    }
    if (position > g_balance_debug.positive_peak_position_mm) {
        g_balance_debug.positive_peak_position_mm = position;
    }
    g_balance_debug.positive_overshoot_mm =
        (g_balance_debug.positive_peak_position_mm >
         s_tuning.positive_target_mm) ?
        (int16_t)(g_balance_debug.positive_peak_position_mm -
                  s_tuning.positive_target_mm) : 0;

    if ((state == BALANCE_STATE_MOVE_NEGATIVE) ||
        (state == BALANCE_STATE_HOLD_NEGATIVE)) {
        if (position < g_balance_debug.negative_min_position_mm) {
            g_balance_debug.negative_min_position_mm = position;
        }
        g_balance_debug.negative_overshoot_mm =
            (g_balance_debug.negative_min_position_mm <
             s_tuning.negative_target_mm) ?
            (int16_t)(s_tuning.negative_target_mm -
                      g_balance_debug.negative_min_position_mm) : 0;
    }
}

static void Balance_CheckTargets(int16_t position, uint32_t sample_tick)
{
    int16_t error;
    int16_t abs_error;
    int16_t positive_turn_tolerance_mm;
    uint8_t positive_peak_pass;
    uint8_t negative_min_pass;
    float predicted_position;
    float abs_velocity =
        Balance_AbsFloat(g_balance_debug.velocity_filtered_mm_s);

    if (((BalanceState_t)g_balance_debug.state ==
         BALANCE_STATE_HOLD_NEGATIVE) &&
        (g_balance_debug.completion_recorded != 0U)) {
        return;
    }

    if ((BalanceState_t)g_balance_debug.state ==
        BALANCE_STATE_MOVE_POSITIVE) {
        error = (int16_t)(s_tuning.positive_target_mm - position);
        abs_error = Balance_AbsInt16(error);
        positive_turn_tolerance_mm =
            s_tuning.positive_position_tolerance_mm;
        if (positive_turn_tolerance_mm > BALANCE_OFFICIAL_ERROR_MM) {
            positive_turn_tolerance_mm = BALANCE_OFFICIAL_ERROR_MM;
        }
        /*
         * 正向速度较高时，用 150 ms 前视位置提前切换负目标；但实际位置至少必须
         * 到达 +40 mm，保证折返点仍在题目允许的 +50 +/-10 mm 范围内。若球已
         * 在该官方窗口内低速停住，同样连续确认后折返，避免卡在 +40～+46 mm。
         */
        predicted_position = (float)position;
        if (g_balance_debug.velocity_filtered_mm_s > 0.0f) {
            predicted_position +=
                g_balance_debug.velocity_filtered_mm_s *
                BALANCE_POSITIVE_TURN_PREVIEW_S;
        }
        if ((position >=
             (int16_t)(s_tuning.positive_target_mm -
                       BALANCE_OFFICIAL_ERROR_MM)) &&
            (predicted_position >= (float)s_tuning.positive_target_mm)) {
            s_positive_confirm_count = s_tuning.positive_confirm_samples;
        } else if ((position >=
                    (int16_t)(s_tuning.positive_target_mm -
                              positive_turn_tolerance_mm)) &&
            (abs_velocity <= s_tuning.positive_speed_tolerance_mm_s)) {
            if (s_positive_confirm_count < 255U) {
                s_positive_confirm_count++;
            }
        } else {
            s_positive_confirm_count = 0U;
        }

        if (s_positive_confirm_count >= s_tuning.positive_confirm_samples) {
            g_balance_debug.positive_turn_position_mm = position;
            g_balance_debug.positive_turn_velocity_mm_s =
                g_balance_debug.velocity_filtered_mm_s;
            g_balance_debug.positive_turn_error_mm = error;
            g_balance_debug.positive_turn_abs_error_mm = abs_error;
            g_balance_debug.positive_overshoot_mm =
                (g_balance_debug.positive_peak_position_mm >
                 s_tuning.positive_target_mm) ?
                (int16_t)(g_balance_debug.positive_peak_position_mm -
                          s_tuning.positive_target_mm) : 0;
            Balance_ClearPid();
            s_positive_confirm_count = 0U;
            s_negative_stable_start_tick = 0U;
            g_balance_debug.target_mm = s_tuning.negative_target_mm;
            g_balance_state.target_x_cm =
                (float)s_tuning.negative_target_mm / 10.0f;
            Balance_SetState(BALANCE_STATE_MOVE_NEGATIVE);
        }
        return;
    }

    if (((BalanceState_t)g_balance_debug.state ==
         BALANCE_STATE_MOVE_NEGATIVE) ||
        ((BalanceState_t)g_balance_debug.state ==
         BALANCE_STATE_HOLD_NEGATIVE)) {
        error = (int16_t)(s_tuning.negative_target_mm - position);
        abs_error = Balance_AbsInt16(error);
        if ((abs_error <= s_tuning.negative_position_tolerance_mm) &&
            (abs_velocity <= s_tuning.negative_speed_tolerance_mm_s)) {
            if (s_negative_stable_start_tick == 0U) {
                s_negative_stable_start_tick = sample_tick;
                g_balance_debug.negative_stable_window_max_abs_error_mm =
                    abs_error;
            }
            if (abs_error >
                g_balance_debug.negative_stable_window_max_abs_error_mm) {
                g_balance_debug.negative_stable_window_max_abs_error_mm =
                    abs_error;
            }
            if ((g_balance_debug.completion_recorded == 0U) &&
                ((uint32_t)(sample_tick - s_negative_stable_start_tick) >=
                 s_tuning.negative_stable_ms)) {
                g_balance_debug.completion_time_ms =
                    (uint32_t)(sample_tick - g_balance_debug.run_start_tick);
                g_balance_debug.completion_recorded = 1U;
                g_balance_debug.within_5s =
                    (uint8_t)(g_balance_debug.completion_time_ms <=
                              BALANCE_SCORE_LIMIT_MS);
                g_balance_debug.official_time_pass =
                    g_balance_debug.within_5s;
                positive_peak_pass = (uint8_t)
                    ((g_balance_debug.positive_peak_position_mm != INT16_MIN) &&
                     (g_balance_debug.positive_peak_position_mm <=
                      (int16_t)(s_tuning.positive_target_mm +
                                BALANCE_OFFICIAL_ERROR_MM)));
                negative_min_pass = (uint8_t)
                    ((g_balance_debug.negative_min_position_mm != INT16_MAX) &&
                     (g_balance_debug.negative_min_position_mm >=
                      (int16_t)(s_tuning.negative_target_mm -
                                BALANCE_OFFICIAL_ERROR_MM)));
                g_balance_debug.official_endpoint_error_pass =
                    (uint8_t)((g_balance_debug.positive_turn_abs_error_mm <=
                               BALANCE_OFFICIAL_ERROR_MM) &&
                              (positive_peak_pass != 0U) &&
                              (negative_min_pass != 0U) &&
                               (g_balance_debug.
                                negative_stable_window_max_abs_error_mm <=
                                BALANCE_OFFICIAL_ERROR_MM));
                g_balance_debug.score_pass =
                    (uint8_t)((g_balance_debug.official_time_pass != 0U) &&
                              (g_balance_debug.
                               official_endpoint_error_pass != 0U) &&
                              (g_balance_debug.run_fault_free != 0U));
                s_trace_pending_flags |= BALANCE_TRACE_FLAG_COMPLETE;
                Balance_SetState(BALANCE_STATE_HOLD_NEGATIVE);
            }
        } else {
            s_negative_stable_start_tick = 0U;
            if (g_balance_debug.completion_recorded == 0U) {
                g_balance_debug.negative_stable_window_max_abs_error_mm = 0;
            }
        }
    }
}

/**
 * @brief 按剩余距离限制轨迹 P 绝对目标，形成可观察的小球接近减速。
 *
 * 远端最多允许 52 pulse 的 P 倾角，进入各自减速区后线性下降到 26 pulse。
 * 该包络直接作用于期望摆杆位置，而不是只限制命令变化率，因此即使电机已经跟上
 * 旧目标，也会随着小球接近端点主动减小推进倾角。自然 P 小于包络时保持原值，
 * 目标附近的积分补偿、D 制动以及 HOLD_ORIGIN 均不受影响。
 */
static float Balance_ApplyTrajectoryPTargetEnvelope(BalanceState_t state,
                                                     float error,
                                                     float p_target_pulse)
{
    float zone_mm;
    float distance_mm;
    float ratio;
    float allowed_pulse;

    if (state == BALANCE_STATE_MOVE_POSITIVE) {
        zone_mm = (float)s_tuning.positive_decel_start_mm;
    } else if ((state == BALANCE_STATE_MOVE_NEGATIVE) ||
               (state == BALANCE_STATE_HOLD_NEGATIVE)) {
        zone_mm = (float)s_tuning.negative_decel_start_mm;
    } else {
        return p_target_pulse;
    }

    if (zone_mm <= 0.0f) {
        return p_target_pulse;
    }

    distance_mm = Balance_AbsFloat(error);
    ratio = Balance_ClampFloat(distance_mm / zone_mm, 0.0f, 1.0f);
    allowed_pulse = BALANCE_TRAJECTORY_P_NEAR_LIMIT_PULSE +
        (BALANCE_TRAJECTORY_P_FAR_LIMIT_PULSE -
         BALANCE_TRAJECTORY_P_NEAR_LIMIT_PULSE) * ratio;

    return Balance_ClampFloat(p_target_pulse,
                              -allowed_pulse,
                              allowed_pulse);
}

/**
 * @brief 在目标前建立速度包络，目标反方向的制动命令保持全幅。
 *
 * 正误差需要负脉冲使球向右；负误差需要正脉冲使球向左。靠近 +5 cm 时只限制
 * 继续向右的负脉冲推进速度，靠近 -5 cm 时限制继续向左的正脉冲推进速度。
 * 若 D 项已经要求反向制动，则不削弱制动力。
 */
static float Balance_ApplyApproachDeceleration(BalanceState_t state,
                                               float error,
                                               float raw_rate,
                                               float active_max_slew_pulse_s,
                                               float trajectory_rate_scale)
{
    float distance_mm;
    float zone_mm;
    float near_rate;
    float near_response_scale;
    float ratio;
    float allowed_rate;
    float response_scale;
    float target_pulse_sign;

    g_balance_debug.approach_brake_active = 0U;
    g_balance_debug.approach_rate_limit_pulse_s =
        active_max_slew_pulse_s;

    if ((state == BALANCE_STATE_MOVE_POSITIVE) && (error > 0.0f)) {
        distance_mm = error;
        zone_mm = (float)s_tuning.positive_decel_start_mm;
        near_rate = s_tuning.positive_near_rate_pulse_s *
                    trajectory_rate_scale;
        near_response_scale = BALANCE_POSITIVE_NEAR_RESPONSE_SCALE;
        target_pulse_sign = BALANCE_BALL_DIRECTION_PER_POSITIVE_PULSE;
    } else if (((state == BALANCE_STATE_MOVE_NEGATIVE) ||
                (state == BALANCE_STATE_HOLD_NEGATIVE)) &&
               (error < 0.0f)) {
        distance_mm = -error;
        zone_mm = (float)s_tuning.negative_decel_start_mm;
        near_rate = s_tuning.negative_near_rate_pulse_s *
                    trajectory_rate_scale;
        near_response_scale = BALANCE_NEGATIVE_NEAR_RESPONSE_SCALE;
        target_pulse_sign = -BALANCE_BALL_DIRECTION_PER_POSITIVE_PULSE;
    } else {
        return raw_rate;
    }

    /* 只削弱仍在把球推向当前目标的输出，反向制动保持全幅。 */
    if ((raw_rate * target_pulse_sign) <= 0.0f) {
        return raw_rate;
    }

    if (distance_mm >= zone_mm) {
        return raw_rate;
    }

    ratio = Balance_ClampFloat(distance_mm / zone_mm, 0.0f, 1.0f);
    /*
     * 进入端点区后，即使尚未触及绝对速度上限，也主动缩小每次向目标方向的
     * 响应；反方向的 D/回中制动不经过本分支，因此仍可立即全幅刹车。
     */
    response_scale = near_response_scale +
        (1.0f - near_response_scale) * ratio;
    raw_rate *= response_scale;
    /* 二次曲线让端点附近的降速明显强于刚进入减速区时。 */
    allowed_rate = near_rate +
        (active_max_slew_pulse_s - near_rate) * ratio * ratio;
    g_balance_debug.approach_brake_active = 1U;
    g_balance_debug.approach_rate_limit_pulse_s = allowed_rate;

    if ((raw_rate * target_pulse_sign) > allowed_rate) {
        return target_pulse_sign * allowed_rate;
    }
    return raw_rate;
}

static void Balance_ComputeAndSendCascade(BalanceSampleResult_t sample)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;
    LineTrackBalanceFeedforward_t feedforward;
    float dt_s;
    float error_cm;
    float measured_position_cm;
    float measured_velocity_cm_s;
    float target_velocity_cm_s;
    float velocity_derivative_error;
    float velocity_output_degrees;
    float correction_degrees;
    float desired_pulse;
    float desired_delta;
    float maximum_delta;
    float limited_delta;
    float limited_position;
    float active_position_kp;
    float active_position_ki;
    float active_position_kd;
    float active_position_max_out_cm_s;
    float active_position_max_iout_cm_s;
    float active_position_integral_deadband_cm;
    float active_velocity_kp;
    float active_velocity_ki;
    float active_velocity_kd;
    float active_slew_pulse_s;
    float active_max_step_pulse;
    uint8_t trajectory_mode;
    uint8_t integral_enabled;
    int32_t final_pulse;
    int32_t latest_target_pulse;
    uint32_t request_id;
    MotorRequestResult_t result;

    if (Balance_StateIsMotion(state) == 0U) return;
    if (g_motor_command_debug.pending_latest_valid != 0U) {
        Balance_TraceSample();
        return;
    }
    if (sample == BALANCE_SAMPLE_TOO_FAST) {
        s_positive_confirm_count = 0U;
        s_negative_stable_start_tick = 0U;
        Balance_TraceSample();
        return;
    }

    dt_s = (sample == BALANCE_SAMPLE_NORMAL) ?
        ((float)g_balance_debug.sample_dt_ms * 0.001f) :
        BALANCE_NOMINAL_DT_S;
    if (dt_s <= 0.0f) dt_s = BALANCE_NOMINAL_DT_S;

    trajectory_mode = (uint8_t)(Balance_StateIsStrongHold(state) == 0U);
    active_position_kp = s_tuning.kp;
    active_position_ki = s_tuning.ki;
    active_position_kd = s_tuning.kd;
    active_position_max_out_cm_s = s_tuning.position_pid_max_out_cm_s;
    active_position_max_iout_cm_s = s_tuning.position_pid_max_iout_cm_s;
    active_position_integral_deadband_cm =
        s_tuning.position_integral_deadband_cm;
    active_velocity_kp = s_tuning.velocity_pid_kp;
    active_velocity_ki = s_tuning.velocity_pid_ki;
    active_velocity_kd = s_tuning.velocity_pid_kd;
    active_slew_pulse_s = s_tuning.max_slew_pulse_per_s;
    active_max_step_pulse = (float)s_tuning.max_step_pulse;
    if (trajectory_mode != 0U) {
        active_position_kp = BALANCE_SCORING_POSITION_PID_KP;
        active_position_ki = BALANCE_SCORING_POSITION_PID_KI;
        active_position_kd = BALANCE_SCORING_POSITION_PID_KD;
        active_position_max_out_cm_s =
            BALANCE_SCORING_POSITION_PID_MAX_OUT_CM_S;
        active_position_max_iout_cm_s =
            BALANCE_SCORING_POSITION_PID_MAX_IOUT_CM_S;
        active_position_integral_deadband_cm =
            BALANCE_SCORING_POSITION_INTEGRAL_DEADBAND_CM;
        active_velocity_kp = BALANCE_SCORING_VELOCITY_PID_KP;
        active_velocity_ki = BALANCE_SCORING_VELOCITY_PID_KI;
        active_velocity_kd = BALANCE_SCORING_VELOCITY_PID_KD;
        active_position_kp *= s_tuning.trajectory_kp_scale;
        active_position_ki *= s_tuning.trajectory_ki_scale;
        active_position_kd *= s_tuning.trajectory_kd_scale;
        active_slew_pulse_s *= s_tuning.trajectory_slew_scale;
        active_max_step_pulse *= s_tuning.trajectory_step_scale;
    }

    measured_position_cm = (float)g_balance_debug.position_used_mm * 0.1f;
    measured_velocity_cm_s =
        g_balance_debug.velocity_filtered_mm_s * 0.1f;
    error_cm = (float)g_balance_debug.target_mm * 0.1f -
               measured_position_cm;
    integral_enabled = (uint8_t)(Balance_AbsFloat(error_cm) >
        Balance_AbsFloat(active_position_integral_deadband_cm));

    target_velocity_cm_s = Balance_CascadePidUpdate(
        &s_position_pid,
        measured_position_cm,
        (float)g_balance_debug.target_mm * 0.1f,
        -measured_velocity_cm_s,
        integral_enabled,
        active_position_kp,
        active_position_ki,
        active_position_kd,
        active_position_max_out_cm_s,
        active_position_max_iout_cm_s);

    velocity_derivative_error =
        (target_velocity_cm_s - measured_velocity_cm_s) -
        s_velocity_pid.error0;
    velocity_output_degrees = Balance_CascadePidUpdate(
        &s_velocity_pid,
        measured_velocity_cm_s,
        target_velocity_cm_s,
        velocity_derivative_error,
        1U,
        active_velocity_kp,
        active_velocity_ki,
        active_velocity_kd,
        s_tuning.velocity_pid_max_out_deg,
        s_tuning.velocity_pid_max_iout_deg);

    /* 当前机构：正 pulse 使球向左；target<measured 时速度环为负，故取反得到正倾角。 */
    correction_degrees = -velocity_output_degrees;
    memset(&feedforward, 0, sizeof(feedforward));
    (void)LineTrack_GetBalanceFeedforwardSnapshot(&feedforward);
    if (feedforward.active != 0U) {
        correction_degrees += feedforward.feedforward_degrees;
    }

    desired_pulse = Balance_ClampFloat(
        correction_degrees * BALANCE_PULSES_PER_DEGREE,
        (float)s_tuning.soft_min_pulse,
        (float)s_tuning.soft_max_pulse);
    if ((desired_pulse <= (float)s_tuning.soft_min_pulse) ||
        (desired_pulse >= (float)s_tuning.soft_max_pulse)) {
        g_balance_debug.soft_limit_count++;
    }

    desired_delta = desired_pulse - g_balance_debug.command_position_pulse;
    maximum_delta = active_slew_pulse_s * dt_s;
    if (active_max_step_pulse < maximum_delta) {
        maximum_delta = active_max_step_pulse;
    }
    limited_delta = Balance_ClampFloat(
        desired_delta, -maximum_delta, maximum_delta);
    if (limited_delta != desired_delta) {
        g_balance_debug.slew_limit_count++;
    }
    limited_position = Balance_ClampFloat(
        g_balance_debug.command_position_pulse + limited_delta,
        (float)s_tuning.soft_min_pulse,
        (float)s_tuning.soft_max_pulse);
    g_balance_debug.command_position_pulse = limited_position;
    final_pulse = Balance_RoundToInt32(limited_position);

    s_cascade_runtime.position_output_cm_s = target_velocity_cm_s;
    s_cascade_runtime.target_velocity_cm_s = target_velocity_cm_s;
    s_cascade_runtime.velocity_output_degrees = velocity_output_degrees;
    s_cascade_runtime.feedforward_degrees =
        (feedforward.active != 0U) ? feedforward.feedforward_degrees : 0.0f;
    s_cascade_runtime.desired_correction_degrees =
        desired_pulse / BALANCE_PULSES_PER_DEGREE;
    s_cascade_runtime.feedforward_active = feedforward.active;

    g_balance_debug.error_mm = error_cm * 10.0f;
    g_balance_debug.integral_error_mm_s = s_position_pid.error_integral;
    /* 旧遥测字段继续保持 pulse/s 量纲，精确双环量见新增 Watch 字段。 */
    g_balance_debug.p_rate_pulse_s =
        -s_velocity_pid.p_output * BALANCE_PULSES_PER_DEGREE / dt_s;
    g_balance_debug.i_rate_pulse_s =
        -s_velocity_pid.i_output * BALANCE_PULSES_PER_DEGREE / dt_s;
    g_balance_debug.d_rate_pulse_s =
        -s_velocity_pid.d_output * BALANCE_PULSES_PER_DEGREE / dt_s;
    g_balance_debug.centering_rate_pulse_s =
        s_cascade_runtime.feedforward_degrees *
        BALANCE_PULSES_PER_DEGREE / dt_s;
    g_balance_debug.drive_rate_raw_pulse_s = desired_delta / dt_s;
    g_balance_debug.drive_rate_limited_pulse_s = limited_delta / dt_s;
    g_balance_debug.approach_brake_active = 0U;
    g_balance_debug.approach_rate_limit_pulse_s = active_slew_pulse_s;
    g_balance_debug.final_requested_pulse = final_pulse;

    latest_target_pulse = (g_motor_command_debug.pending_latest_valid != 0U) ?
        g_motor_command_debug.pending_latest_pulse :
        g_motor_command_debug.last_transmitted_pulse;
    if (final_pulse == latest_target_pulse) {
        Balance_TraceSample();
        return;
    }

    request_id = Motor_CommandRequestAbsolute(MOTOR_OWNER_BALANCE, final_pulse);
    if (request_id != 0U) {
        result = Motor_CommandGetResult(request_id);
        if (result == MOTOR_REQUEST_HARD_LIMIT_REJECTED) {
            Balance_LatchFault(BALANCE_FAULT_HARD_LIMIT_REQUEST, 0U);
        }
    }
    Balance_TraceSample();
}

static void Balance_ComputeAndSend(BalanceSampleResult_t sample)
{
    Balance_ComputeAndSendCascade(sample);
}

static void Balance_EnterVisionHold(uint8_t range_fault)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    if (Balance_StateIsMotion(state) == 0U) {
        return;
    }
    s_resume_state = state;
    s_vision_hold_range_fault = range_fault;
    Balance_ClearPid();
    g_balance_debug.profile_exit_confirm_count = 0U;
    g_balance_debug.drive_rate_raw_pulse_s = 0.0f;
    g_balance_debug.drive_rate_limited_pulse_s = 0.0f;
    s_negative_stable_start_tick = 0U;
    g_balance_debug.vision_hold_count++;
    s_trace_pending_flags |= BALANCE_TRACE_FLAG_VISION_HOLD;
    Balance_SetState(BALANCE_STATE_VISION_HOLD);
}

static void Balance_ProcessVision(const Camera_Data_t *camera,
                                  BalanceSampleResult_t sample,
                                  uint32_t now)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    if (camera->frame_sequence != s_last_frame_sequence) {
        uint32_t frame_delta =
            (uint32_t)(camera->frame_sequence - s_last_frame_sequence);
        uint8_t range_fault =
            (uint8_t)(g_camera_debug.physical_reject_count !=
                      s_last_physical_reject_count);
        s_last_frame_sequence = camera->frame_sequence;
        s_last_physical_reject_count =
            g_camera_debug.physical_reject_count;
        if ((state == BALANCE_STATE_VISION_HOLD) &&
            (range_fault != 0U)) {
            s_vision_hold_range_fault = 1U;
        }
        if (camera->target_found != 0U) {
            g_balance_debug.vision_lost_streak = 0U;
        } else if (frame_delta >=
                   (uint32_t)(255U - g_balance_debug.vision_lost_streak)) {
            g_balance_debug.vision_lost_streak = 255U;
        } else {
            g_balance_debug.vision_lost_streak = (uint8_t)
                (g_balance_debug.vision_lost_streak + frame_delta);
        }
        if ((g_balance_debug.vision_lost_streak >=
             s_tuning.vision_lost_confirm_frames) &&
            (Balance_StateIsMotion(state) != 0U)) {
            Balance_EnterVisionHold(range_fault);
            state = (BalanceState_t)g_balance_debug.state;
        }
    }

    if ((Balance_StateIsMotion(state) != 0U) &&
        ((camera->last_valid_tick == 0U) ||
         ((uint32_t)(now - camera->last_valid_tick) >=
          s_tuning.vision_continue_ms))) {
        /* 串口静默时也先冻结；不再从运动态直接因少量漏帧锁死。 */
        Balance_EnterVisionHold(0U);
        state = (BalanceState_t)g_balance_debug.state;
    }

    if (state == BALANCE_STATE_VISION_HOLD) {
        if ((camera->target_found != 0U) &&
            (sample != BALANCE_SAMPLE_NONE) &&
            (sample != BALANCE_SAMPLE_TOO_FAST)) {
            /* 恢复首帧只重建历史，不在该帧发送新的位置命令。 */
            s_last_position_mm = camera->x;
            s_last_valid_tick = camera->last_valid_tick;
            g_balance_debug.velocity_raw_mm_s = Balance_ClampFloat(
                (float)camera->y,
                -BALANCE_CAMERA_VELOCITY_ABS_MAX_CM_S,
                BALANCE_CAMERA_VELOCITY_ABS_MAX_CM_S) * 10.0f;
            g_balance_debug.velocity_filtered_mm_s =
                g_balance_debug.velocity_raw_mm_s;
            g_balance_debug.vision_lost_streak = 0U;
            s_vision_hold_range_fault = 0U;
            Balance_SetState(s_resume_state);
            return;
        }
        /* 不设置最终超时：保持最近一次已确认的电机目标，持续等待视觉恢复。 */
    }
}

static void Balance_ProcessReturnZero(uint32_t now)
{
    MotorRequestResult_t result;
    int32_t current;
    int32_t next;
    float delta;

    if (Motor_CommandGetOwner() != MOTOR_OWNER_SAFETY) {
        Balance_LatchFault(BALANCE_FAULT_MOTOR_OWNER_LOST, 1U);
        return;
    }

    /* 先发 best-effort STOP，建立安全屏障并排空被抢占位置命令的迟到 ACK。 */
    if (s_return_stop_done == 0U) {
        if (s_return_request_id == 0U) {
            s_return_request_id =
                Motor_CommandRequestStop(MOTOR_OWNER_SAFETY);
            return;
        }
        result = Motor_CommandGetResult(s_return_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) return;
        s_return_request_id = 0U;
        s_return_stop_done = 1U;
        s_return_last_tick = now;
        return;
    }

    if (s_return_request_id != 0U) {
        result = Motor_CommandGetResult(s_return_request_id);
        if ((result == MOTOR_REQUEST_PENDING) ||
            (result == MOTOR_REQUEST_UNKNOWN)) return;
        s_return_request_id = 0U;
        if (result != MOTOR_REQUEST_SUCCESS) {
            g_balance_debug.fault = BALANCE_FAULT_RETURN_ZERO_FAILED;
            Balance_SetState(BALANCE_STATE_FAULT_LATCHED);
            s_disable_request_id = Motor_CommandRequestStop(MOTOR_OWNER_SAFETY);
            return;
        }
        if (g_motor_command_debug.last_acked_pulse == 0) {
            Balance_SetState((s_return_is_fault != 0U) ?
                             BALANCE_STATE_FAULT_LATCHED :
                             BALANCE_STATE_ZERO_HOLD_ABORTED);
            return;
        }
    }

    if ((uint32_t)(now - s_return_last_tick) < 50U) {
        return;
    }
    s_return_last_tick = now;
    current = g_motor_command_debug.last_transmitted_pulse;
    delta = BALANCE_RETURN_ZERO_MAX_SLEW_PULSE_S * BALANCE_NOMINAL_DT_S;
    if (delta > BALANCE_RETURN_ZERO_MAX_STEP_PULSE) {
        delta = BALANCE_RETURN_ZERO_MAX_STEP_PULSE;
    }
    if (delta < 1.0f) {
        delta = 1.0f;
    }
    if (current > 0) {
        next = current - Balance_RoundToInt32(delta);
        if (next < 0) next = 0;
    } else if (current < 0) {
        next = current + Balance_RoundToInt32(delta);
        if (next > 0) next = 0;
    } else {
        next = 0;
    }
    s_return_request_id =
        Motor_CommandRequestAbsolute(MOTOR_OWNER_SAFETY, next);
    g_balance_debug.command_position_pulse = (float)next;
    g_balance_debug.final_requested_pulse = next;
    g_balance_debug.drive_rate_raw_pulse_s = 0.0f;
    g_balance_debug.drive_rate_limited_pulse_s = 0.0f;
}

static void Balance_BeginDisable(void)
{
    Balance_ClearPid();
    s_velocity_history_valid = 0U;
    g_balance_debug.velocity_history_valid = 0U;
    Motor_CommandSafetyTakeover();
    s_disable_phase = 0U;
    s_disable_request_id = 0U;
    Balance_SetState(BALANCE_STATE_DISABLING);
}

static void Balance_ProcessDisable(void)
{
    MotorRequestResult_t result;

    if (s_disable_request_id == 0U) {
        s_disable_request_id = (s_disable_phase == 0U) ?
            Motor_CommandRequestStop(MOTOR_OWNER_SAFETY) :
            Motor_CommandRequestDisable(MOTOR_OWNER_SAFETY);
        return;
    }
    result = Motor_CommandGetResult(s_disable_request_id);
    if ((result == MOTOR_REQUEST_PENDING) ||
        (result == MOTOR_REQUEST_UNKNOWN)) return;
    s_disable_request_id = 0U;
    if (s_disable_phase == 0U) {
        s_disable_phase = 1U;
        return;
    }
    Balance_SetState(BALANCE_STATE_DISABLED);
}

static void Balance_ProcessFaultStop(void)
{
    MotorRequestResult_t result;

    if (s_disable_phase != 0U) {
        return;
    }

    if (Motor_CommandGetOwner() != MOTOR_OWNER_SAFETY) {
        Motor_CommandSafetyTakeover();
    }
    if (s_disable_request_id == 0U) {
        s_disable_request_id = Motor_CommandRequestStop(MOTOR_OWNER_SAFETY);
        return;
    }
    result = Motor_CommandGetResult(s_disable_request_id);
    if ((result != MOTOR_REQUEST_PENDING) &&
        (result != MOTOR_REQUEST_UNKNOWN)) {
        s_disable_request_id = 0U;
        s_disable_phase = 1U;
    }
}

static void Balance_HandleWatchCommand(const Camera_Data_t *camera,
                                       uint32_t now)
{
    uint32_t command = g_balance_command;
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    if (command == BALANCE_COMMAND_IDLE) {
        return;
    }
    g_balance_command = BALANCE_COMMAND_IDLE;

    switch ((BalanceWatchCommand_t)command) {
    case BALANCE_COMMAND_START:
        g_balance_debug.watch_start_count++;
        if (state == BALANCE_STATE_IDLE) {
            Balance_TryStart(camera, now, 0U);
        }
        break;

    case BALANCE_COMMAND_STOP_RETURN_ZERO:
        if (Balance_StateIsRunning(state) != 0U) {
            g_balance_debug.run_fault_free = 0U;
            Balance_ClearPid();
            Motor_CommandSafetyTakeover();
            s_return_is_fault = 0U;
            s_return_stop_done = 0U;
            s_return_request_id = 0U;
            s_return_last_tick = now;
            Balance_SetState(BALANCE_STATE_RETURN_ZERO);
        }
        break;

    case BALANCE_COMMAND_RESET_RUN:
        if (((state == BALANCE_STATE_FAULT_LATCHED) ||
             (state == BALANCE_STATE_ZERO_HOLD_ABORTED) ||
             (state == BALANCE_STATE_DISABLED)) &&
            (Motor_CommandIsBusy() == 0U)) {
            (void)Motor_CommandRelease(MOTOR_OWNER_SAFETY);
            g_balance_debug.fault = BALANCE_FAULT_NONE;
            g_balance_debug.start_reject_reason = BALANCE_START_REJECT_NONE;
            g_balance_debug.run_start_tick = 0U;
            g_balance_debug.run_elapsed_ms = 0U;
            Balance_ClearScoreSummary();
            g_balance_state.target_locked = 0U;
            s_start_confirm_count = 0U;
            Balance_ClearPid();
            Balance_SetState(BALANCE_STATE_IDLE);
        }
        break;

    case BALANCE_COMMAND_CLEAR_TRACE:
        if ((Balance_StateIsRunning(state) == 0U) &&
            (state != BALANCE_STATE_DISABLING)) {
            Balance_ClearTrace();
            Balance_ClearScoreSummary();
        } else {
            g_balance_trace.clear_reject_count++;
        }
        break;

    case BALANCE_COMMAND_DISABLE:
        Balance_BeginDisable();
        break;

    default:
        break;
    }
}

static void Balance_CheckMotorCompletion(void)
{
    uint32_t id = g_motor_command_debug.last_completed_request_id;
    MotorRequestResult_t result;

    if ((id == 0U) || (id == s_seen_motor_completion_id)) {
        return;
    }
    s_seen_motor_completion_id = id;
    if (g_motor_command_debug.last_completed_owner != MOTOR_OWNER_BALANCE) {
        return;
    }
    result = (MotorRequestResult_t)g_motor_command_debug.last_completed_result;
    if ((result == MOTOR_REQUEST_TX_FAILED) ||
        (result == MOTOR_REQUEST_ACK_TIMEOUT) ||
        (result == MOTOR_REQUEST_DEVICE_REJECTED) ||
        (result == MOTOR_REQUEST_HARD_LIMIT_REJECTED)) {
        Balance_LatchFault(Balance_MotorFaultFromResult(
            result, BALANCE_FAULT_INTERNAL_STATE_ERROR),
            (uint8_t)(result != MOTOR_REQUEST_HARD_LIMIT_REJECTED));
    }
}

void Balance_ControlInit(void)
{
    memset((void *)s_public_debug, 0, sizeof(*s_public_debug));
    memset((void *)&g_balance_debug, 0, sizeof(g_balance_debug));
    memset((void *)&g_balance_state, 0, sizeof(g_balance_state));
    Balance_ResetCascadePid(&s_position_pid);
    Balance_ResetCascadePid(&s_velocity_pid);
    memset(&s_cascade_runtime, 0, sizeof(s_cascade_runtime));
    g_balance_command = BALANCE_COMMAND_IDLE;
    Balance_ClearTrace();
    Balance_LoadDefaultTuning();
    Balance_InitTargetButton();

    s_last_frame_sequence = 0U;
    s_last_valid_tick = 0U;
    s_last_position_mm = 0;
    s_velocity_history_valid = 0U;
    s_button_raw = (uint8_t)(HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port,
                                              START_BUTTON_Pin) == GPIO_PIN_SET);
    s_button_debounced = s_button_raw;
    s_button_raw_change_tick = HAL_GetTick();
    s_start_confirm_count = 0U;
    s_positive_confirm_count = 0U;
    s_negative_stable_start_tick = 0U;
    s_resume_state = BALANCE_STATE_IDLE;
    s_state_request_id = 0U;
    s_start_zero_phase = 0U;
    s_return_request_id = 0U;
    s_return_stop_done = 0U;
    s_disable_request_id = 0U;
    s_seen_motor_completion_id = 0U;
    s_vision_hold_range_fault = 0U;
    s_last_physical_reject_count = 0U;
    s_auto_start_completed = 0U;
    s_target_level_phase = 0U;
    s_target_exit_pending = 0U;
    s_target_level_next_query_tick = 0U;
    s_target_capture_baseline_valid_sequence = 0U;

    g_balance_debug.state = BALANCE_STATE_BOOT_SAFE;
    g_balance_debug.state_enter_tick = HAL_GetTick();
    g_balance_debug.button_raw_level = s_button_raw;
    g_balance_debug.button_debounced_level = s_button_debounced;
    g_balance_debug.target_button_raw_level = (uint8_t)
        (HAL_GPIO_ReadPin(TARGET_LOCK_BUTTON_GPIO_Port,
                          TARGET_LOCK_BUTTON_Pin) == GPIO_PIN_RESET);
    g_balance_debug.target_button_debounced_level =
        g_balance_debug.target_button_raw_level;
    g_balance_debug.target_mm = 0;
    g_balance_state.target_x_cm = 0.0f;
    Balance_SetState(BALANCE_STATE_WAIT_BUTTON_RELEASE);
    Balance_UpdateMotorMirror();
    Balance_UpdatePublicDebug();
}

void Balance_ControlLoop(void)
{
    Camera_Data_t camera;
    BalanceSampleResult_t sample;
    BalanceState_t state;
    uint32_t now = HAL_GetTick();
    uint8_t press_event;
    uint8_t target_press_event;
    uint8_t trajectory_press_event;
    uint8_t ui_exit_event;
    uint8_t pa0_event_handled = 0U;

    Balance_UpdateTuningSnapshot();
    (void)Camera_GetData(&camera);
    Balance_UpdateCameraMirror(&camera, now);
    press_event = Balance_UpdateButton(now);
    target_press_event = Balance_UpdateTargetButton(now);
    trajectory_press_event = UI_MenuTakeTrajectoryPress();
    ui_exit_event = UI_MenuTakeBalanceExit();
    sample = Balance_ProcessNewSample(&camera);
    state = (BalanceState_t)g_balance_debug.state;

    if (state == BALANCE_STATE_WAIT_BUTTON_RELEASE) {
        if (g_balance_debug.button_release_seen != 0U) {
            Balance_SetState(BALANCE_STATE_IDLE);
            state = BALANCE_STATE_IDLE;
        }
    }

    if (state == BALANCE_STATE_IDLE) {
        Balance_UpdateStartReady(&camera, sample, now);
        if (press_event != 0U) {
            pa0_event_handled = 1U;
            if (g_balance_debug.pa0_consumed == 0U) {
                Balance_TryStart(&camera, now, 1U);
            } else {
                g_balance_debug.button_rejected_count++;
            }
        } else if ((BALANCE_AUTO_START_AFTER_POWER_ON != 0U) &&
                   (s_auto_start_completed == 0U) &&
                   (g_balance_debug.start_ready != 0U)) {
            /* PA0 已确认松开且视觉连续识别到小球后，上电流程自动开始一次。 */
            Balance_TryStart(&camera, now, 0U);
            if ((BalanceState_t)g_balance_debug.state != BALANCE_STATE_IDLE) {
                s_auto_start_completed = 1U;
            }
        }
    }

    Balance_HandleWatchCommand(&camera, now);
    state = (BalanceState_t)g_balance_debug.state;

    if (ui_exit_event != 0U) {
        Balance_HandleUiExit();
        state = (BalanceState_t)g_balance_debug.state;
    }

    if (target_press_event != 0U) {
        if (state == BALANCE_STATE_TARGET_WAIT_CAPTURE) {
            Balance_TryCaptureTarget(&camera, now);
        } else if (Balance_CanBeginTargetLeveling(state) != 0U) {
            Balance_BeginTargetLeveling();
        } else {
            g_balance_debug.target_button_rejected_count++;
        }
        state = (BalanceState_t)g_balance_debug.state;
    }

    Balance_UpdateTrajectoryReady(&camera, sample, now);
    if (((press_event != 0U) && (pa0_event_handled == 0U)) ||
        (trajectory_press_event != 0U)) {
        if (state == BALANCE_STATE_HOLD_ORIGIN) {
            Balance_TryStartTrajectory(&camera, now);
        } else {
            g_balance_debug.start_reject_reason =
                (g_balance_debug.trajectory_started != 0U) ?
                BALANCE_START_REJECT_TRAJECTORY_ALREADY_USED :
                BALANCE_START_REJECT_NOT_HOLDING_ORIGIN;
            g_balance_debug.button_rejected_count++;
        }
        state = (BalanceState_t)g_balance_debug.state;
    }

    if (g_balance_debug.run_start_tick != 0U) {
        g_balance_debug.run_elapsed_ms =
            (uint32_t)(now - g_balance_debug.run_start_tick);
        if ((g_balance_debug.trajectory_started != 0U) &&
            (g_balance_debug.completion_recorded == 0U) &&
            (g_balance_debug.run_elapsed_ms > BALANCE_SCORE_LIMIT_MS)) {
            g_balance_debug.time_score_failed = 1U;
        }
    }

    if ((state >= BALANCE_STATE_START_QUERY_MOTOR) &&
        (state <= BALANCE_STATE_START_ENABLE)) {
        Balance_ProcessStartup(&camera, now);
    } else if (state == BALANCE_STATE_TARGET_LEVELING) {
        Balance_ProcessTargetLeveling(now);
    } else if (Balance_StateIsMotion(state) != 0U) {
        Balance_ProcessVision(&camera, sample, now);
        state = (BalanceState_t)g_balance_debug.state;
        if ((Balance_StateIsMotion(state) != 0U) &&
            (sample != BALANCE_SAMPLE_NONE)) {
            if (g_balance_debug.trajectory_started != 0U) {
                Balance_UpdateScoringBeforeControl(camera.x);
                if (sample == BALANCE_SAMPLE_NORMAL) {
                    Balance_CheckTargets(camera.x, camera.last_valid_tick);
                } else {
                    s_positive_confirm_count = 0U;
                    s_negative_stable_start_tick = 0U;
                }
            }
            Balance_ComputeAndSend(sample);
        }
    } else if (state == BALANCE_STATE_VISION_HOLD) {
        Balance_ProcessVision(&camera, sample, now);
    } else if (state == BALANCE_STATE_RETURN_ZERO) {
        Balance_ProcessReturnZero(now);
    } else if (state == BALANCE_STATE_DISABLING) {
        Balance_ProcessDisable();
    } else if (state == BALANCE_STATE_FAULT_LATCHED) {
        Balance_ProcessFaultStop();
    }

    Balance_CheckMotorCompletion();
    Balance_UpdateMotorMirror();
    Balance_UpdatePublicDebug();
}

uint8_t Balance_GetTelemetrySnapshot(BalanceTelemetrySnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return 0U;
    }

    /*
     * 当前仅由裸机主循环在 Balance_ControlLoop() 之后调用，不会与控制计算并发。
     * 保留独立快照接口，避免遥测模块直接依赖私有 BalanceRuntime_t 布局。
     */
    snapshot->state = s_balance_runtime.state;
    snapshot->fault = s_balance_runtime.fault;
    snapshot->start_reject_reason = s_balance_runtime.start_reject_reason;
    snapshot->start_ready = s_balance_runtime.start_ready;
    snapshot->trajectory_ready = s_balance_runtime.trajectory_ready;
    snapshot->trajectory_started = s_balance_runtime.trajectory_started;
    snapshot->trajectory_completed = s_balance_runtime.completion_recorded;
    snapshot->trajectory_ready_count =
        s_balance_runtime.trajectory_ready_count;
    snapshot->run_elapsed_ms = s_balance_runtime.run_elapsed_ms;
    snapshot->completion_time_ms = s_balance_runtime.completion_time_ms;
    snapshot->score_pass = s_balance_runtime.score_pass;
    snapshot->official_endpoint_error_pass =
        s_balance_runtime.official_endpoint_error_pass;
    snapshot->official_time_pass = s_balance_runtime.official_time_pass;
    snapshot->run_fault_free = s_balance_runtime.run_fault_free;
    snapshot->positive_turn_position_mm =
        s_balance_runtime.positive_turn_position_mm;
    snapshot->positive_turn_abs_error_mm =
        s_balance_runtime.positive_turn_abs_error_mm;
    snapshot->positive_peak_position_mm =
        s_balance_runtime.positive_peak_position_mm;
    snapshot->positive_overshoot_mm =
        s_balance_runtime.positive_overshoot_mm;
    snapshot->negative_min_position_mm =
        s_balance_runtime.negative_min_position_mm;
    snapshot->negative_overshoot_mm =
        s_balance_runtime.negative_overshoot_mm;
    snapshot->negative_stable_window_max_abs_error_mm =
        s_balance_runtime.negative_stable_window_max_abs_error_mm;

    snapshot->camera_online = s_balance_runtime.camera_online;
    snapshot->target_found = s_balance_runtime.target_found;
    snapshot->vision_lost_streak = s_balance_runtime.vision_lost_streak;
    snapshot->last_valid_age_ms = s_balance_runtime.last_valid_age_ms;
    snapshot->sample_dt_ms = s_balance_runtime.sample_dt_ms;
    snapshot->frame_sequence = s_balance_runtime.frame_sequence;
    snapshot->valid_sequence = s_balance_runtime.valid_sequence;

    snapshot->target_mm = s_balance_runtime.target_mm;
    snapshot->position_mm = s_balance_runtime.position_used_mm;
    snapshot->velocity_mm_s = s_balance_runtime.velocity_filtered_mm_s;
    snapshot->error_mm = s_balance_runtime.error_mm;
    snapshot->p_rate_pulse_s = s_balance_runtime.p_rate_pulse_s;
    snapshot->i_rate_pulse_s = s_balance_runtime.i_rate_pulse_s;
    snapshot->d_rate_pulse_s = s_balance_runtime.d_rate_pulse_s;
    snapshot->centering_rate_pulse_s =
        s_balance_runtime.centering_rate_pulse_s;
    snapshot->raw_rate_pulse_s = s_balance_runtime.drive_rate_raw_pulse_s;
    snapshot->limited_rate_pulse_s =
        s_balance_runtime.drive_rate_limited_pulse_s;
    snapshot->command_position_pulse =
        s_balance_runtime.command_position_pulse;
    snapshot->final_requested_pulse =
        s_balance_runtime.final_requested_pulse;
    snapshot->last_transmitted_pulse =
        s_balance_runtime.last_transmitted_pulse;
    snapshot->last_acked_pulse = s_balance_runtime.last_acked_pulse;

    snapshot->motor_owner = s_balance_runtime.motor_owner;
    snapshot->motor_state = s_balance_runtime.motor_command_state;
    snapshot->motor_retry_count = s_balance_runtime.motor_retry_count;
    snapshot->motor_ack_status = s_balance_runtime.motor_ack_status;
    snapshot->motor_last_result = g_motor_command_debug.last_completed_result;
    snapshot->motor_pending_valid =
        g_motor_command_debug.pending_latest_valid;
    snapshot->motor_tx_fail_count = s_balance_runtime.motor_tx_fail_count;
    snapshot->motor_ack_timeout_count =
        s_balance_runtime.motor_ack_timeout_count;
    snapshot->motor_ack_reject_count =
        s_balance_runtime.motor_ack_reject_count;
    snapshot->hard_limit_reject_count =
        s_balance_runtime.hard_limit_reject_count;
    snapshot->vision_hold_count = s_balance_runtime.vision_hold_count;
    snapshot->tuning_error_mask = s_balance_runtime.tuning_error_mask;
    return 1U;
}

void Balance_SetTargetCm(float target_x_cm)
{
    if (target_x_cm > 5.0f) target_x_cm = 5.0f;
    if (target_x_cm < -5.0f) target_x_cm = -5.0f;
    g_balance_state.target_x_cm = target_x_cm;
}

void Balance_LatchInitialTarget(float ball_x_cm)
{
    Balance_SetTargetCm(ball_x_cm);
    g_balance_state.target_locked = 1U;
}

void Balance_UnlockTarget(void)
{
    g_balance_state.target_locked = 0U;
}

uint8_t Balance_IsClosedLoopActive(void)
{
    BalanceState_t state = (BalanceState_t)g_balance_debug.state;

    return (uint8_t)((Balance_StateIsMotion(state) != 0U) &&
                     (g_balance_debug.fault == BALANCE_FAULT_NONE) &&
                     (g_motor_command_debug.owner == MOTOR_OWNER_BALANCE));
}
