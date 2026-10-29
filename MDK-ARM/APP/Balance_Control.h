#ifndef F407_BALANCE_CONTROL_H
#define F407_BALANCE_CONTROL_H

/**
 ******************************************************************************
 * @file    Balance_Control.h
 * @brief   水平建零、O 点/按键目标稳球及 PA0 O->+5 cm->-5 cm 轨迹状态机。
 ******************************************************************************
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BALANCE_TRACE_CAPACITY  128U
#define BALANCE_TRAJECTORY_SCORE_LIMIT_MS      5000U
#define BALANCE_TRAJECTORY_OFFICIAL_ERROR_MM     10

/*
 * F4-MIX 串级稳球默认参数。
 * 这里只决定上电初始化值；运行后请在 Keil Watch 中直接修改
 * g_balance_tuning 对应字段，控制循环不会再次用这些宏覆盖它。
 */
#define BALANCE_POSITION_PID_KP_DEFAULT                 1.05f
#define BALANCE_POSITION_PID_KI_DEFAULT                 0.008f
#define BALANCE_POSITION_PID_KD_DEFAULT                 0.08f
#define BALANCE_POSITION_PID_MAX_OUT_CM_S_DEFAULT      50.0f
#define BALANCE_POSITION_PID_MAX_IOUT_CM_S_DEFAULT     10.0f
#define BALANCE_POSITION_INTEGRAL_DEADBAND_CM_DEFAULT   0.10f
#define BALANCE_VELOCITY_PID_KP_DEFAULT                 2.25f
#define BALANCE_VELOCITY_PID_KI_DEFAULT                 0.0003f
#define BALANCE_VELOCITY_PID_KD_DEFAULT                 2.10f
#define BALANCE_VELOCITY_PID_MAX_OUT_DEG_DEFAULT      150.0f
#define BALANCE_VELOCITY_PID_MAX_IOUT_DEG_DEFAULT      50.0f
#define BALANCE_VELOCITY_OLD_WEIGHT_DEFAULT             0.30f
#define BALANCE_TRAJECTORY_KP_SCALE_DEFAULT             1.0f
#define BALANCE_TRAJECTORY_KI_SCALE_DEFAULT             1.0f
#define BALANCE_TRAJECTORY_KD_SCALE_DEFAULT             1.0f
#define BALANCE_TRAJECTORY_SLEW_SCALE_DEFAULT           1.0f
#define BALANCE_TRAJECTORY_STEP_SCALE_DEFAULT           1.0f

typedef enum
{
    BALANCE_STATE_BOOT_SAFE = 0,
    BALANCE_STATE_WAIT_BUTTON_RELEASE,
    BALANCE_STATE_IDLE,
    BALANCE_STATE_START_QUERY_MOTOR,
    BALANCE_STATE_START_SET_ZERO,
    BALANCE_STATE_START_VERIFY_ZERO,
    BALANCE_STATE_START_ENABLE,
    BALANCE_STATE_MOVE_POSITIVE,
    BALANCE_STATE_MOVE_NEGATIVE,
    BALANCE_STATE_HOLD_NEGATIVE,
    BALANCE_STATE_VISION_HOLD,
    BALANCE_STATE_RETURN_ZERO,
    BALANCE_STATE_ZERO_HOLD_ABORTED,
    BALANCE_STATE_FAULT_LATCHED,
    BALANCE_STATE_DISABLING,
    BALANCE_STATE_DISABLED,
    /* 保留原第三问状态编号不变；新的稳球状态追加在末尾。 */
    BALANCE_STATE_HOLD_ORIGIN,
    BALANCE_STATE_TARGET_LEVELING,
    BALANCE_STATE_TARGET_WAIT_CAPTURE,
    BALANCE_STATE_HOLD_CAPTURED_TARGET
} BalanceState_t;

typedef enum
{
    BALANCE_FAULT_NONE = 0,
    BALANCE_FAULT_CAMERA_LOST,                 /* 兼容旧 Watch；当前不会因视觉超时触发 */
    BALANCE_FAULT_CAMERA_OUT_OF_RANGE_PERSISTENT, /* 兼容旧 Watch；当前只等待新有效帧 */
    BALANCE_FAULT_MOTOR_OWNER_LOST,
    BALANCE_FAULT_MOTOR_TX_FAILED,
    BALANCE_FAULT_MOTOR_ACK_TIMEOUT,
    BALANCE_FAULT_MOTOR_ACK_REJECTED,
    BALANCE_FAULT_MOTOR_QUERY_INVALID,
    BALANCE_FAULT_MOTOR_ZERO_FAILED,
    BALANCE_FAULT_MOTOR_ZERO_VERIFY_FAILED,
    BALANCE_FAULT_MOTOR_ENABLE_FAILED,
    BALANCE_FAULT_HARD_LIMIT_REQUEST,
    BALANCE_FAULT_RETURN_ZERO_FAILED,
    BALANCE_FAULT_INTERNAL_STATE_ERROR
} BalanceFault_t;

typedef enum
{
    BALANCE_START_REJECT_NONE = 0,
    BALANCE_START_REJECT_BUTTON_NOT_RELEASED_AFTER_POWER_ON,
    BALANCE_START_REJECT_CAMERA_OFFLINE,
    BALANCE_START_REJECT_TARGET_NOT_FOUND,
    BALANCE_START_REJECT_COORDINATE_STALE,
    BALANCE_START_REJECT_BALL_NOT_AT_ORIGIN,
    BALANCE_START_REJECT_BALL_MOVING_TOO_FAST,
    BALANCE_START_REJECT_MOTOR_OWNER_BUSY,
    BALANCE_START_REJECT_TUNING_NOT_VALID,
    BALANCE_START_REJECT_TARGET_NOT_CONFIRMED,
    BALANCE_START_REJECT_NOT_HOLDING_ORIGIN,
    BALANCE_START_REJECT_TRAJECTORY_ALREADY_USED
} BalanceStartReject_t;

typedef enum
{
    BALANCE_COMMAND_IDLE = 0,
    BALANCE_COMMAND_START = 1,
    BALANCE_COMMAND_STOP_RETURN_ZERO = 2,
    BALANCE_COMMAND_RESET_RUN = 3,
    BALANCE_COMMAND_CLEAR_TRACE = 4,
    BALANCE_COMMAND_DISABLE = 5
} BalanceWatchCommand_t;

typedef enum
{
    BALANCE_CONTROL_PROFILE_NEAR = 0,
    BALANCE_CONTROL_PROFILE_FAR = 1
} BalanceControlProfile_t;

typedef struct
{
    /* F4-MIX 串级 PID：位置环输出目标球速，速度环输出摆杆角度。 */
    float kp;                            /* 位置环 Kp，输出 cm/s */
    float ki;                            /* 位置环 Ki，按有效视觉帧累加 */
    float kd;                            /* 位置环 Kd，直接使用 -球速 */
    float command_centering_gain_s;      /* 兼容旧 Watch，串级控制不读取 */
    float velocity_old_weight;
    float integral_window_mm;            /* 兼容旧 Watch，串级控制不读取 */
    float integral_output_limit_pulse;   /* 兼容旧 Watch，串级控制不读取 */
    float position_pid_max_out_cm_s;
    float position_pid_max_iout_cm_s;
    float position_integral_deadband_cm;
    float velocity_pid_kp;
    float velocity_pid_ki;
    float velocity_pid_kd;
    float velocity_pid_max_out_deg;
    float velocity_pid_max_iout_deg;
    int16_t positive_target_mm;          /* PA0 轨迹正向目标，默认 +50 mm */
    int16_t negative_target_mm;          /* PA0 轨迹最终目标，默认 -50 mm */
    int16_t soft_min_pulse;
    int16_t soft_max_pulse;
    float max_slew_pulse_per_s;
    int16_t max_step_pulse;
    uint16_t initial_motion_ramp_ms;      /* 首次进入正向运动时的软启动时长，0为关闭 */
    int16_t positive_decel_start_mm;      /* 距 +5 cm 还有多少毫米时开始提前减速 */
    float positive_near_rate_pulse_s;    /* +5 cm 附近仍向右推进时的最小速度上限 */
    int16_t negative_decel_start_mm;      /* 距 -5 cm 还有多少毫米时开始提前减速 */
    float negative_near_rate_pulse_s;    /* -5 cm 附近仍向左推进时的最小速度上限 */
    uint16_t camera_fresh_ms;
    int16_t start_position_tolerance_mm; /* 兼容旧 Watch；水平建零启动逻辑不读取 */
    float start_speed_tolerance_mm_s;    /* 兼容旧 Watch；水平建零启动逻辑不读取 */
    uint8_t start_confirm_samples;       /* 任意位置有效视觉连续确认帧数 */
    int16_t trajectory_start_position_tolerance_mm; /* PA0 轨迹启动时 |x| 上限 */
    float trajectory_start_speed_tolerance_mm_s;    /* PA0 轨迹启动时 |v| 上限 */
    uint8_t trajectory_start_confirm_samples;       /* 连续正常帧数 */
    int16_t positive_position_tolerance_mm;
    float positive_speed_tolerance_mm_s;
    uint8_t positive_confirm_samples;
    int16_t negative_position_tolerance_mm;
    float negative_speed_tolerance_mm_s;
    uint16_t negative_stable_ms;

    /* 只作用于 MOVE_POSITIVE/MOVE_NEGATIVE/HOLD_NEGATIVE；默认 1.0。 */
    float trajectory_kp_scale;
    float trajectory_ki_scale;
    float trajectory_kd_scale;
    float trajectory_follow_gain_scale;
    float trajectory_slew_scale;
    float trajectory_step_scale;

    uint8_t vision_lost_confirm_frames;  /* 连续多少个 N/非法帧后才冻结控制 */
    uint16_t vision_continue_ms;         /* 无新坐标时允许电机继续执行最近目标的时间 */
    uint16_t vision_hold_ms;             /* 兼容旧 Watch，当前忽略；视觉等待无超时 */

    /* 兼容旧 Watch 的远距离档字段；当前 F4-MIX 串级控制不读取。 */
    float far_kp;
    float far_ki;
    float far_kd;
    float far_command_centering_gain_s;
    float far_max_slew_pulse_per_s;
    float far_p_target_limit_pulse;
    float far_total_target_limit_pulse;
    float far_enter_speed_mm_s;
    float far_exit_speed_mm_s;
    int16_t far_max_step_pulse;
    int16_t far_enter_position_mm;
    int16_t far_exit_position_mm;
    uint8_t far_exit_confirm_samples;
} BalanceTuning_t;

typedef struct
{
    uint32_t elapsed_ms;
    uint8_t state;
    uint8_t flags;
    int16_t target_mm;
    int16_t position_mm;
    float velocity_mm_s;
    float p_rate_pulse_s;
    float i_rate_pulse_s;
    float d_rate_pulse_s;
    float centering_rate_pulse_s;
    float raw_rate_pulse_s;
    float limited_rate_pulse_s;
    float command_position_pulse;
    int16_t final_pulse;
} BalanceTracePoint_t;

typedef struct
{
    BalanceTracePoint_t point[BALANCE_TRACE_CAPACITY];
    uint16_t count;
    uint8_t full;
    uint8_t overflow;
    uint32_t clear_reject_count;
} BalanceTraceBuffer_t;

typedef struct
{
    /* 启动时优先看：state/fault/start_reject/start_ready。 */
    uint8_t state;
    uint8_t fault;
    uint8_t start_reject_reason;
    uint8_t start_ready;
    uint8_t trajectory_ready;
    uint8_t trajectory_started;
    uint8_t trajectory_completed;
    uint8_t trajectory_ready_count;
    uint8_t pa0_consumed;
    uint32_t run_elapsed_ms;
    uint32_t completion_time_ms;
    uint8_t score_pass;
    int16_t positive_turn_position_mm;
    int16_t positive_turn_abs_error_mm;
    int16_t positive_peak_position_mm;
    int16_t positive_overshoot_mm;
    int16_t negative_min_position_mm;
    int16_t negative_overshoot_mm;
    int16_t negative_stable_window_max_abs_error_mm;
    uint8_t official_endpoint_error_pass;
    uint8_t official_time_pass;
    uint8_t run_fault_free;

    /* PA0：松开 raw=0，按下 raw=1，稳定 30 ms 后 debounced 跟随。 */
    uint8_t button_raw_level;
    uint8_t button_debounced_level;
    uint8_t button_release_seen;
    /* PE2：物理低电平按下；这里仍统一为 raw/debounced 1=按下。 */
    uint8_t target_button_raw_level;
    uint8_t target_button_debounced_level;
    uint32_t target_button_press_count;
    uint32_t target_button_rejected_count;

    /* Camera：控制只使用 position_used_mm 和滤波速度。 */
    uint8_t camera_online;
    uint8_t target_found;
    uint32_t last_valid_age_ms;
    uint32_t sample_dt_ms;
    int16_t position_used_mm;
    float velocity_filtered_mm_s;
    uint8_t vision_lost_streak;
    uint32_t vision_hold_count;
    uint32_t vision_fault_count;         /* 兼容旧 Watch，当前视觉丢失不会使其增加 */

    /* 闭环调参：target_mm 随状态为 0/+50/-50，P/I/D 形成安全绝对位置目标。 */
    int16_t target_mm;
    float error_mm;
    float drive_rate_pulse_s;
    float centering_rate_pulse_s;
    uint8_t approach_brake_active;
    float approach_rate_limit_pulse_s;
    float command_position_pulse;
    int32_t final_requested_pulse;
    int32_t last_transmitted_pulse;
    int32_t last_acked_pulse;
    uint32_t tuning_error_mask;

    /* 电机通信摘要；需要原始请求 ID/ACK 时再看 g_motor_command_debug。 */
    uint8_t motor_owner;
    uint8_t motor_command_state;
    uint8_t motor_retry_count;
    uint32_t motor_tx_fail_count;
    uint32_t motor_ack_timeout_count;
    uint32_t motor_ack_reject_count;
    uint32_t hard_limit_reject_count;

    /* 双档控制观察量：0=近端档，1=远距离/高速档。 */
    uint8_t control_profile;
    uint8_t profile_exit_confirm_count;
    uint32_t profile_switch_count;

    /* F4-MIX 串级 PID 与慢速循迹前馈的只读 Watch 量。 */
    float position_pid_output_cm_s;
    float target_velocity_cm_s;
    float velocity_pid_output_deg;
    float acceleration_feedforward_deg;
    float desired_correction_deg;
    uint8_t line_track_feedforward_active;
} BalanceDebug_t;

/**
 * @brief USART3 遥测专用的完整控制快照。
 * @note 该接口只读，不改变状态机、PID、Motor_Command 所有权或电机目标。
 */
typedef struct
{
    uint8_t state;
    uint8_t fault;
    uint8_t start_reject_reason;
    uint8_t start_ready;
    uint8_t trajectory_ready;
    uint8_t trajectory_started;
    uint8_t trajectory_completed;
    uint8_t trajectory_ready_count;
    uint32_t run_elapsed_ms;
    uint32_t completion_time_ms;
    uint8_t score_pass;
    uint8_t official_endpoint_error_pass;
    uint8_t official_time_pass;
    uint8_t run_fault_free;
    int16_t positive_turn_position_mm;
    int16_t positive_turn_abs_error_mm;
    int16_t positive_peak_position_mm;
    int16_t positive_overshoot_mm;
    int16_t negative_min_position_mm;
    int16_t negative_overshoot_mm;
    int16_t negative_stable_window_max_abs_error_mm;

    uint8_t camera_online;
    uint8_t target_found;
    uint8_t vision_lost_streak;
    uint32_t last_valid_age_ms;
    uint32_t sample_dt_ms;
    uint32_t frame_sequence;
    uint32_t valid_sequence;

    int16_t target_mm;
    int16_t position_mm;
    float velocity_mm_s;
    float error_mm;
    float p_rate_pulse_s;
    float i_rate_pulse_s;
    float d_rate_pulse_s;
    float centering_rate_pulse_s;
    float raw_rate_pulse_s;
    float limited_rate_pulse_s;
    float command_position_pulse;
    int32_t final_requested_pulse;
    int32_t last_transmitted_pulse;
    int32_t last_acked_pulse;

    uint8_t motor_owner;
    uint8_t motor_state;
    uint8_t motor_retry_count;
    uint8_t motor_ack_status;
    uint8_t motor_last_result;
    uint8_t motor_pending_valid;
    uint32_t motor_tx_fail_count;
    uint32_t motor_ack_timeout_count;
    uint32_t motor_ack_reject_count;
    uint32_t hard_limit_reject_count;
    uint32_t vision_hold_count;
    uint32_t tuning_error_mask;
} BalanceTelemetrySnapshot_t;

/* 兼容旧 Watch 的简化目标镜像；实际控制目标以 g_balance_debug.target_mm 为准。 */
typedef struct
{
    float target_x_cm;
    uint8_t target_locked;
} BalanceControlState_t;

/*
 * Keil Watch 实时调参的唯一全局变量。
 * Balance_ControlInit() 只初始化一次；运行中修改后由控制循环安全接管。
 */
extern volatile BalanceTuning_t g_balance_tuning;
extern volatile BalanceDebug_t g_balance_debug;
extern volatile uint32_t g_balance_command;
extern volatile BalanceTraceBuffer_t g_balance_trace;
extern volatile BalanceControlState_t g_balance_state;

void Balance_ControlInit(void);
void Balance_ControlLoop(void);
uint8_t Balance_GetTelemetrySnapshot(BalanceTelemetrySnapshot_t *snapshot);
void Balance_SetTargetCm(float target_x_cm);
void Balance_LatchInitialTarget(float ball_x_cm);
void Balance_UnlockTarget(void);
uint8_t Balance_IsClosedLoopActive(void);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_CONTROL_H */
