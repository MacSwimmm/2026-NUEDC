#include "LineTrack_Control.h"

#include "Balance_Control.h"
#include "Track_Encoder.h"
#include "Track_Motor.h"
#include "Track_PID.h"
#include "Track_Sensor.h"

#include "stm32f4xx_hal.h"

#include <stdbool.h>
#include <string.h>

/*
 * 本文件从 F4-MIX/app/app_chassis.c 中抽取底盘循迹控制路径：
 * 八路位置加权 -> 循迹外环 PID -> 左右轮目标 -> 双轮速度 PID -> TB6612。
 * 只删除了 F4-MIX 自己的 FreeRTOS、UART、OLED、ZDT 和滚球控制实现；循迹公式、
 * 参数、终点判断、斜坡、丢线恢复、编码器故障判断和执行顺序保持原样。
 */

#define LINE_TRACK_SAMPLE_TIME_MS          (10U)
#define LINE_TRACK_C1_HIGH_Z_TIME_MS       (3000U)

typedef enum {
    CAR_MODE_STOP = 0,
    CAR_MODE_RUN,
    CAR_MODE_ZERO_SPEED_HOLD
} TrackCarMode_t;

typedef struct {
    float speed;
    float speed_set;
    TrackPID_t speed_pid;
    int32_t pwm_out;
    int32_t encoder_delta;
    uint32_t encoder_no_feedback_time_ms;
    bool encoder_fault;
} TrackMotor_t;

typedef struct {
    TrackPID_t angle_pid;
    float ramped_base_speed;
    int16_t last_error;
    int8_t last_direction;
    uint32_t lost_time_ms;
    uint32_t finish_confirm_time_ms;
    uint32_t finish_delay_time_ms;
    bool finish_pending;
    bool finish_armed;
    bool task3_start_ramp_complete;
    bool task4_start_ramp_complete;
    bool line_was_visible;
    bool line_ever_seen;
    bool normal_line_seen;
    float acceleration_source_speed;
    float acceleration_last_speed;
    float longitudinal_acceleration;
    float longitudinal_acceleration_filtered;
    float acceleration_feedforward_target_degrees;
    float acceleration_feedforward_degrees;
} TrackControl_t;

typedef struct {
    TrackCarMode_t car_mode;
    LineTrackState_t state;
    TrackMotor_t motor[2];
    TrackControl_t track;
    uint32_t start_tick_ms;
    uint32_t elapsed_time_ms;
    uint32_t final_time_ms;
    bool timer_running;
} LineTrackSystem_t;

volatile LineTrackDebug_t g_line_track_debug;

static LineTrackSystem_t s_line_track;
static uint32_t s_last_process_tick;
static uint32_t s_track_c1_enable_tick;
static uint8_t s_initialized;
static uint8_t s_start_request;
static uint8_t s_stop_request;

#define LINE_TRACK_REQUEST_NONE             0xFFU

static float line_track_absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t line_track_absi32(int32_t value)
{
    return (value < 0) ? ((uint32_t)(-(value + 1)) + 1U) :
                         (uint32_t)value;
}

static float line_track_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static float line_track_slew_toward(float current,
                                    float target,
                                    float increase_per_s,
                                    float decrease_per_s,
                                    uint32_t elapsed_ms)
{
    float maximum_delta;

    if ((elapsed_ms == 0U) || (elapsed_ms > 100U)) {
        elapsed_ms = 10U;
    }
    maximum_delta = ((target > current) ? increase_per_s : decrease_per_s) *
                    ((float)elapsed_ms * 0.001f);
    if (maximum_delta < 0.0f) maximum_delta = -maximum_delta;

    if (target > (current + maximum_delta)) return current + maximum_delta;
    if (target < (current - maximum_delta)) return current - maximum_delta;
    return target;
}

static void line_track_pid_reset(TrackPID_t *pid)
{
    if (pid == NULL) return;
    pid->Target = 0.0f;
    pid->Actual = 0.0f;
    pid->Out = 0.0f;
    pid->Error0 = 0.0f;
    pid->Error1 = 0.0f;
    pid->ErrorInt = 0.0f;
    pid->D_error = 0.0f;
}

static void line_track_stop(LineTrackSystem_t *system)
{
    uint8_t i;

    if (system == NULL) return;
    system->motor[0].speed_set = 0.0f;
    system->motor[1].speed_set = 0.0f;

    for (i = 0U; i < 2U; i++) {
        line_track_pid_reset(&system->motor[i].speed_pid);
        system->motor[i].pwm_out = 0;
    }
}

static void line_track_zero_speed_hold(LineTrackSystem_t *system)
{
    uint8_t i;

    if (system == NULL) return;
    system->motor[0].speed_set = 0.0f;
    system->motor[1].speed_set = 0.0f;
    for (i = 0U; i < 2U; i++) {
        system->motor[i].speed_pid.ErrorInt = 0.0f;
        system->motor[i].speed_pid.Error0 = 0.0f;
        system->motor[i].speed_pid.Error1 = 0.0f;
        system->motor[i].speed_pid.D_error = 0.0f;
        system->motor[i].speed_pid.Target = 0.0f;
    }
    system->car_mode = CAR_MODE_ZERO_SPEED_HOLD;
}

static void line_track_control_reset(TrackControl_t *track)
{
    if (track == NULL) return;

    line_track_pid_reset(&track->angle_pid);
    track->ramped_base_speed = 0.0f;
    track->last_error = 0;
    track->last_direction = 0;
    track->lost_time_ms = 0U;
    track->finish_confirm_time_ms = 0U;
    track->finish_delay_time_ms = 0U;
    track->finish_pending = false;
    track->finish_armed = false;
    track->task3_start_ramp_complete = false;
    track->task4_start_ramp_complete = false;
    track->line_was_visible = false;
    track->line_ever_seen = false;
    track->normal_line_seen = false;
    track->acceleration_source_speed = 0.0f;
    track->acceleration_last_speed = 0.0f;
    track->longitudinal_acceleration = 0.0f;
    track->longitudinal_acceleration_filtered = 0.0f;
    track->acceleration_feedforward_target_degrees = 0.0f;
    track->acceleration_feedforward_degrees = 0.0f;
}

static void line_track_balance_control_reset(LineTrackSystem_t *system)
{
    float current_longitudinal_speed;

    if (system == NULL) return;
    current_longitudinal_speed =
        0.5f * (system->motor[0].speed_set + system->motor[1].speed_set);
    line_track_control_reset(&system->track);
    system->track.ramped_base_speed = current_longitudinal_speed;
}

static void line_track_motor_fault_reset(LineTrackSystem_t *system)
{
    uint8_t i;

    if (system == NULL) return;
    for (i = 0U; i < 2U; i++) {
        system->motor[i].encoder_delta = 0;
        system->motor[i].encoder_no_feedback_time_ms = 0U;
        system->motor[i].encoder_fault = false;
    }
}

/*
 * 恢复 F4-MIX Task4 的“斜坡纵向速度 -> 加速度 -> 摆杆角度”前馈。
 * 该函数只为当前工程的慢速模式工作；快速和直行模式进入这里时会把前馈清零，
 * 因而不会改变它们原有的左右轮目标、PID 或终点逻辑。
 */
static void line_track_update_balance_feedforward(LineTrackSystem_t *system)
{
    TrackControl_t *track;
    float raw_acceleration;
    float feedforward_gain;
    float feedforward_target;

    if (system == NULL) return;
    track = &system->track;

    if (system->state != LINE_TRACK_STATE_TASK4_BALANCE) {
        track->acceleration_source_speed = 0.0f;
        track->acceleration_last_speed = 0.0f;
        track->longitudinal_acceleration = 0.0f;
        track->longitudinal_acceleration_filtered = 0.0f;
        track->acceleration_feedforward_target_degrees = 0.0f;
        track->acceleration_feedforward_degrees = 0.0f;
        return;
    }

    track->acceleration_source_speed = track->ramped_base_speed;
    raw_acceleration =
        (track->acceleration_source_speed - track->acceleration_last_speed) /
        ((float)LINE_TRACK_SAMPLE_TIME_MS * 0.001f);
    track->acceleration_last_speed = track->acceleration_source_speed;
    track->longitudinal_acceleration = raw_acceleration;
    track->longitudinal_acceleration_filtered =
        TRACK_TASK4_ACCEL_FILTER_OLD_WEIGHT *
            track->longitudinal_acceleration_filtered +
        (1.0f - TRACK_TASK4_ACCEL_FILTER_OLD_WEIGHT) * raw_acceleration;

    feedforward_gain = track->finish_pending ?
        TRACK_TASK4_STOP_ACCEL_FF_GAIN :
        (track->task4_start_ramp_complete ?
            TRACK_TASK4_RUNNING_ACCEL_FF_GAIN :
            TRACK_TASK4_START_ACCEL_FF_GAIN);
    feedforward_target = TRACK_TASK4_ACCEL_FF_DIRECTION_SIGN *
        feedforward_gain * track->longitudinal_acceleration_filtered;
    feedforward_target = line_track_clampf(
        feedforward_target,
        -TRACK_TASK4_ACCEL_FF_MAX_DEG,
        TRACK_TASK4_ACCEL_FF_MAX_DEG);
    track->acceleration_feedforward_target_degrees = feedforward_target;
    track->acceleration_feedforward_degrees = line_track_slew_toward(
        track->acceleration_feedforward_degrees,
        feedforward_target,
        TRACK_TASK4_ACCEL_FF_SLEW_DEG_PER_S,
        TRACK_TASK4_ACCEL_FF_SLEW_DEG_PER_S,
        LINE_TRACK_SAMPLE_TIME_MS);
}

static bool line_track_state_is_running(LineTrackState_t state)
{
    return (state == LINE_TRACK_STATE_TASK1_TRACKING) ||
           (state == LINE_TRACK_STATE_TASK3_BALANCE) ||
           (state == LINE_TRACK_STATE_TASK4_BALANCE);
}

static void line_track_start(LineTrackState_t requested_state, uint32_t now)
{
    LineTrackSystem_t *system = &s_line_track;

    line_track_stop(system);
    system->car_mode = CAR_MODE_STOP;
    if (requested_state == LINE_TRACK_STATE_TASK1_TRACKING) {
        line_track_control_reset(&system->track);
        g_line_track_debug.fast_start_count++;
    } else {
        line_track_balance_control_reset(system);
        if (requested_state == LINE_TRACK_STATE_TASK3_BALANCE) {
            g_line_track_debug.straight_start_count++;
        } else {
            g_line_track_debug.slow_start_count++;
        }
    }
    line_track_motor_fault_reset(system);
    system->state = requested_state;
    system->start_tick_ms = now;
    system->elapsed_time_ms = 0U;
    system->final_time_ms = 0U;
    system->timer_running = true;
}

static void line_track_feedback_update(LineTrackSystem_t *system)
{
    uint8_t i;

    if (system == NULL) return;
    bsp_encoder_update();

    system->motor[0].encoder_delta = g_encoderA_cnt;
    system->motor[1].encoder_delta = g_encoderB_cnt;
    for (i = 0U; i < 2U; i++) {
        int32_t delta = system->motor[i].encoder_delta;
        system->motor[i].speed = (float)delta;

        if ((system->car_mode == CAR_MODE_RUN) &&
            (line_track_absf(system->motor[i].speed_set) >=
             MOTOR_ENCODER_FAULT_MIN_TARGET_SPEED) &&
            (line_track_absi32(system->motor[i].pwm_out) >=
             (uint32_t)MOTOR_ENCODER_FAULT_MIN_PWM) &&
            (delta == 0)) {
            if (system->motor[i].encoder_no_feedback_time_ms <
                MOTOR_ENCODER_FAULT_TIMEOUT_MS) {
                system->motor[i].encoder_no_feedback_time_ms +=
                    LINE_TRACK_SAMPLE_TIME_MS;
            }
            if (system->motor[i].encoder_no_feedback_time_ms >=
                MOTOR_ENCODER_FAULT_TIMEOUT_MS) {
                system->motor[i].encoder_fault = true;
            }
        } else if (delta != 0) {
            system->motor[i].encoder_no_feedback_time_ms = 0U;
        }
    }
}

static void line_track_finish_task1(LineTrackSystem_t *system)
{
    if ((system == NULL) ||
        (system->state != LINE_TRACK_STATE_TASK1_TRACKING)) return;

    system->timer_running = false;
    system->final_time_ms = system->elapsed_time_ms;
    system->state = LINE_TRACK_STATE_TASK1_FINISHED;
    line_track_zero_speed_hold(system);
}

static void line_track_finish_task3(LineTrackSystem_t *system)
{
    if ((system == NULL) ||
        (system->state != LINE_TRACK_STATE_TASK3_BALANCE)) return;

    if (system->timer_running) {
        system->final_time_ms = system->elapsed_time_ms;
        system->timer_running = false;
    }
    system->state = LINE_TRACK_STATE_TASK3_FINISHED;
    line_track_zero_speed_hold(system);
}

static void line_track_finish_task4(LineTrackSystem_t *system)
{
    if ((system == NULL) ||
        (system->state != LINE_TRACK_STATE_TASK4_BALANCE)) return;

    if (system->timer_running) {
        system->final_time_ms = system->elapsed_time_ms;
        system->timer_running = false;
    }
    system->state = LINE_TRACK_STATE_TASK4_FINISHED;
    line_track_zero_speed_hold(system);
}

static void line_track_set_control(LineTrackSystem_t *system)
{
    TrackControl_t *track;
    bool valid = false;
    bool balance_drive_task = false;
    bool balance_stop_ramp_active = false;
    bool finish_detection_allowed;
    uint8_t track_bits;
    uint8_t finish_sensor_mask;
    uint32_t finish_stop_delay_ms;
    int16_t error;

    if (system == NULL) return;
    track = &system->track;

    if (system->motor[0].encoder_fault || system->motor[1].encoder_fault) {
        system->car_mode = CAR_MODE_STOP;
        line_track_stop(system);
        return;
    }

    switch (system->state) {
    case LINE_TRACK_STATE_TASK1_TRACKING:
        break;
    case LINE_TRACK_STATE_TASK3_BALANCE:
    case LINE_TRACK_STATE_TASK4_BALANCE:
        balance_drive_task = true;
        break;
    case LINE_TRACK_STATE_TASK1_FINISHED:
    case LINE_TRACK_STATE_TASK3_FINISHED:
    case LINE_TRACK_STATE_TASK4_FINISHED:
        system->car_mode = CAR_MODE_ZERO_SPEED_HOLD;
        system->motor[0].speed_set = 0.0f;
        system->motor[1].speed_set = 0.0f;
        return;
    case LINE_TRACK_STATE_IDLE:
    default:
        system->car_mode = CAR_MODE_STOP;
        line_track_stop(system);
        return;
    }

    /* 慢速前馈模式与滚球闭环绑定；滚球退出运动态时复用原减速停车路径。 */
    if ((system->state == LINE_TRACK_STATE_TASK4_BALANCE) &&
        (Balance_IsClosedLoopActive() == 0U)) {
        if (!track->finish_pending) {
            track->finish_pending = true;
            system->final_time_ms = system->elapsed_time_ms;
            system->timer_running = false;
        }
        track->finish_delay_time_ms = TRACK_FINISH_STOP_DELAY_MS;
    }

    track_bits = bsp_track_read();
    g_line_track_debug.track_bits = track_bits;
    if (system->state == LINE_TRACK_STATE_TASK3_BALANCE) {
        finish_sensor_mask = TRACK_TASK3_FINISH_SENSOR_MASK;
        finish_stop_delay_ms = TRACK_TASK3_FINISH_STOP_DELAY_MS;
    } else {
        finish_sensor_mask = TRACK_FINISH_SENSOR_MASK;
        finish_stop_delay_ms = TRACK_FINISH_STOP_DELAY_MS;
    }

    if (system->state == LINE_TRACK_STATE_TASK4_BALANCE) {
        /* F4-MIX Task4：从 A 点出发必须先离开 C7/C8，返回后才允许触发终点。 */
        if ((track_bits & finish_sensor_mask) == 0U) {
            track->finish_armed = true;
        }
        finish_detection_allowed = track->finish_armed;
    } else {
        finish_detection_allowed = track->normal_line_seen;
    }

    if (!track->finish_pending) {
        if (((track_bits & finish_sensor_mask) != 0U) &&
            finish_detection_allowed) {
            if (track->finish_confirm_time_ms < TRACK_FINISH_CONFIRM_MS) {
                track->finish_confirm_time_ms += LINE_TRACK_SAMPLE_TIME_MS;
            }
            if (track->finish_confirm_time_ms >= TRACK_FINISH_CONFIRM_MS) {
                track->finish_pending = true;
                track->finish_delay_time_ms = 0U;
                if ((system->state == LINE_TRACK_STATE_TASK3_BALANCE) ||
                    (system->state == LINE_TRACK_STATE_TASK4_BALANCE)) {
                    /* F4-MIX Task3/4 在终点传感器确认时冻结计时。 */
                    system->final_time_ms = system->elapsed_time_ms;
                    system->timer_running = false;
                }
            }
        } else {
            track->finish_confirm_time_ms = 0U;
        }
    }

    if (track->finish_pending) {
        if (track->finish_delay_time_ms < finish_stop_delay_ms) {
            track->finish_delay_time_ms += LINE_TRACK_SAMPLE_TIME_MS;
        }
        if (track->finish_delay_time_ms >= finish_stop_delay_ms) {
            if (balance_drive_task) {
                balance_stop_ramp_active = true;
            } else {
                line_track_finish_task1(system);
                return;
            }
        }
    }

    if (balance_stop_ramp_active) {
        track->ramped_base_speed = line_track_slew_toward(
            track->ramped_base_speed,
            0.0f,
            TRACK_BALANCE_ACCEL_SPEED_PER_S,
            (system->state == LINE_TRACK_STATE_TASK3_BALANCE) ?
                TRACK_TASK3_STOP_RAMP_SPEED_PER_S :
                TRACK_TASK4_STOP_RAMP_SPEED_PER_S,
            LINE_TRACK_SAMPLE_TIME_MS);

        if (line_track_absf(track->ramped_base_speed) <=
            TRACK_BALANCE_STOP_SPEED_EPSILON) {
            track->ramped_base_speed = 0.0f;
            system->motor[0].speed_set = 0.0f;
            system->motor[1].speed_set = 0.0f;
            if (system->state == LINE_TRACK_STATE_TASK3_BALANCE) {
                line_track_finish_task3(system);
            } else {
                line_track_finish_task4(system);
            }
            return;
        }

        system->motor[0].speed_set = track->ramped_base_speed;
        system->motor[1].speed_set = track->ramped_base_speed;
        system->car_mode = CAR_MODE_RUN;
        return;
    }

    if ((track_bits != 0U) && (track_bits != 0xFFU)) {
        track->normal_line_seen = true;
    }

    error = bsp_track_line_position_from_bits(track_bits, &valid);
    g_line_track_debug.line_error = error;

    if (valid) {
        float abs_error = (error < 0) ? (float)(-error) : (float)error;
        float task_base_speed =
            (system->state == LINE_TRACK_STATE_TASK3_BALANCE) ?
                TRACK_TASK3_BASE_SPEED :
                ((system->state == LINE_TRACK_STATE_TASK4_BALANCE) ?
                    TRACK_TASK4_BASE_SPEED : TRACK_BASE_SPEED);
        float base_speed = task_base_speed - abs_error * 0.02f;
        float correction = TrackPID_Calculate(&track->angle_pid,
                                              0.0f,
                                              (float)error);

        base_speed = line_track_clampf(base_speed,
                                       TRACK_MIN_BASE_SPEED,
                                       task_base_speed);
        correction = line_track_clampf(correction,
                                       -TRACK_MAX_CORRECTION_SPEED,
                                       TRACK_MAX_CORRECTION_SPEED);

        if (balance_drive_task) {
            float correction_scale;
            float ramp_speed =
                (system->state == LINE_TRACK_STATE_TASK3_BALANCE) ?
                    TRACK_TASK3_RAMP_SPEED_PER_S :
                    TRACK_TASK4_RAMP_SPEED_PER_S;

            track->ramped_base_speed = line_track_slew_toward(
                track->ramped_base_speed,
                base_speed,
                ramp_speed,
                ramp_speed,
                LINE_TRACK_SAMPLE_TIME_MS);

            if ((system->state == LINE_TRACK_STATE_TASK3_BALANCE) &&
                !track->task3_start_ramp_complete &&
                (line_track_absf(track->ramped_base_speed - base_speed) <=
                 TRACK_BALANCE_STOP_SPEED_EPSILON)) {
                track->task3_start_ramp_complete = true;
            }
            if ((system->state == LINE_TRACK_STATE_TASK4_BALANCE) &&
                !track->task4_start_ramp_complete &&
                (line_track_absf(track->ramped_base_speed - base_speed) <=
                 TRACK_BALANCE_STOP_SPEED_EPSILON)) {
                track->task4_start_ramp_complete = true;
            }

            correction_scale = line_track_clampf(
                track->ramped_base_speed / task_base_speed, 0.0f, 1.0f);
            base_speed = track->ramped_base_speed;
            correction *= correction_scale;
        }

        system->motor[0].speed_set = line_track_clampf(
            base_speed + correction,
            -TRACK_MAX_WHEEL_SPEED,
            TRACK_MAX_WHEEL_SPEED);
        system->motor[1].speed_set = line_track_clampf(
            base_speed - correction,
            -TRACK_MAX_WHEEL_SPEED,
            TRACK_MAX_WHEEL_SPEED);

        if (error < -TRACK_DIRECTION_DEADBAND) {
            track->last_direction = -1;
        } else if (error > TRACK_DIRECTION_DEADBAND) {
            track->last_direction = 1;
        }
        track->last_error = error;
        track->lost_time_ms = 0U;
        track->line_was_visible = true;
        track->line_ever_seen = true;
        system->car_mode = CAR_MODE_RUN;
        return;
    }

    track->line_was_visible = false;
    if (track->lost_time_ms < TRACK_LOST_PROTECT_MS) {
        track->lost_time_ms += LINE_TRACK_SAMPLE_TIME_MS;
    }

    if (track->line_ever_seen &&
        (track->lost_time_ms < TRACK_LOST_PROTECT_MS)) {
        int8_t recovery_direction = track->last_direction;
        if (recovery_direction == 0) {
            recovery_direction = (track->last_error < 0) ? -1 : 1;
        }

        g_line_track_debug.recovery_enter_count++;
        system->car_mode = CAR_MODE_RUN;
        if (recovery_direction < 0) {
            system->motor[0].speed_set = -TRACK_RECOVERY_SPEED;
            system->motor[1].speed_set = TRACK_RECOVERY_SPEED;
        } else {
            system->motor[0].speed_set = TRACK_RECOVERY_SPEED;
            system->motor[1].speed_set = -TRACK_RECOVERY_SPEED;
        }
        return;
    }

    system->car_mode = CAR_MODE_STOP;
    line_track_stop(system);
}

static void line_track_control_loop(LineTrackSystem_t *system)
{
    uint8_t i;

    if (system == NULL) return;
    if ((system->car_mode != CAR_MODE_RUN) &&
        (system->car_mode != CAR_MODE_ZERO_SPEED_HOLD)) {
        system->motor[0].speed_set = 0.0f;
        system->motor[1].speed_set = 0.0f;
        system->motor[0].pwm_out = 0;
        system->motor[1].pwm_out = 0;
        return;
    }

    if (system->car_mode == CAR_MODE_ZERO_SPEED_HOLD) {
        system->motor[0].speed_set = 0.0f;
        system->motor[1].speed_set = 0.0f;
    }

    for (i = 0U; i < 2U; i++) {
        system->motor[i].pwm_out = (int32_t)TrackPID_Calculate(
            &system->motor[i].speed_pid,
            system->motor[i].speed,
            system->motor[i].speed_set);
    }
    g_line_track_debug.control_count++;
}

static void line_track_send_command(LineTrackSystem_t *system)
{
    if (system == NULL) return;
    if ((system->car_mode != CAR_MODE_RUN) &&
        (system->car_mode != CAR_MODE_ZERO_SPEED_HOLD)) {
        bsp_motor_stop_all();
        return;
    }
    bsp_motor_set_all_pwm(system->motor[0].pwm_out,
                          system->motor[1].pwm_out);
}

static void line_track_publish_debug(void)
{
    g_line_track_debug.initialized = s_initialized;
    g_line_track_debug.state = (uint8_t)s_line_track.state;
    g_line_track_debug.car_mode = (uint8_t)s_line_track.car_mode;
    g_line_track_debug.left_speed = s_line_track.motor[0].speed;
    g_line_track_debug.right_speed = s_line_track.motor[1].speed;
    g_line_track_debug.left_speed_set = s_line_track.motor[0].speed_set;
    g_line_track_debug.right_speed_set = s_line_track.motor[1].speed_set;
    g_line_track_debug.left_pwm = s_line_track.motor[0].pwm_out;
    g_line_track_debug.right_pwm = s_line_track.motor[1].pwm_out;
    g_line_track_debug.left_encoder_fault =
        (uint8_t)s_line_track.motor[0].encoder_fault;
    g_line_track_debug.right_encoder_fault =
        (uint8_t)s_line_track.motor[1].encoder_fault;
    g_line_track_debug.finish_pending =
        (uint8_t)s_line_track.track.finish_pending;
    g_line_track_debug.finish_armed =
        (uint8_t)s_line_track.track.finish_armed;
    g_line_track_debug.elapsed_time_ms = s_line_track.elapsed_time_ms;
    g_line_track_debug.final_time_ms = s_line_track.final_time_ms;
    g_line_track_debug.balance_feedforward_active = (uint8_t)
        (s_line_track.state == LINE_TRACK_STATE_TASK4_BALANCE);
    g_line_track_debug.acceleration_source_speed =
        s_line_track.track.acceleration_source_speed;
    g_line_track_debug.longitudinal_acceleration_filtered =
        s_line_track.track.longitudinal_acceleration_filtered;
    g_line_track_debug.acceleration_feedforward_degrees =
        s_line_track.track.acceleration_feedforward_degrees;
}

void LineTrack_Init(void)
{
    static const float speed_pid_parameters[3] = {
        MOTOR_SPEED_PID_KP,
        MOTOR_SPEED_PID_KI,
        MOTOR_SPEED_PID_KD
    };
    static const float angle_pid_parameters[3] = {
        TRACK_ANGLE_PID_KP,
        TRACK_ANGLE_PID_KI,
        TRACK_ANGLE_PID_KD
    };
    uint8_t i;
    uint32_t now = HAL_GetTick();

    memset(&s_line_track, 0, sizeof(s_line_track));
    memset((void *)&g_line_track_debug, 0, sizeof(g_line_track_debug));
    s_line_track.state = LINE_TRACK_STATE_IDLE;
    s_line_track.car_mode = CAR_MODE_STOP;

    for (i = 0U; i < 2U; i++) {
        TrackPID_Init(&s_line_track.motor[i].speed_pid,
                      TRACK_PID_POSITION,
                      speed_pid_parameters,
                      MOTOR_SPEED_PID_MAX_OUT,
                      MOTOR_SPEED_PID_MAX_IOUT);
    }
    TrackPID_Init(&s_line_track.track.angle_pid,
                  TRACK_PID_POSITION,
                  angle_pid_parameters,
                  TRACK_ANGLE_PID_MAX_OUT,
                  TRACK_ANGLE_PID_MAX_IOUT);

    bsp_motor_init();
    bsp_encoder_init();
    bsp_track_init();
    line_track_motor_fault_reset(&s_line_track);
    bsp_motor_stop_all();

    s_last_process_tick = now;
    s_track_c1_enable_tick = now + LINE_TRACK_C1_HIGH_Z_TIME_MS;
    s_start_request = LINE_TRACK_REQUEST_NONE;
    s_stop_request = 0U;
    s_initialized = 1U;
    line_track_publish_debug();
}

void LineTrack_Process(void)
{
    uint32_t now;

    if (s_initialized == 0U) return;
    now = HAL_GetTick();
    if ((uint32_t)(now - s_last_process_tick) < LINE_TRACK_SAMPLE_TIME_MS) {
        return;
    }
    if ((uint32_t)(now - s_last_process_tick) > 100U) {
        s_last_process_tick = now;
    } else {
        s_last_process_tick += LINE_TRACK_SAMPLE_TIME_MS;
    }

    if (!g_track_c1_ready &&
        ((int32_t)(now - s_track_c1_enable_tick) >= 0)) {
        bsp_track_enable_c1_pullup();
    }

    if (s_stop_request != 0U) {
        s_stop_request = 0U;
        s_start_request = LINE_TRACK_REQUEST_NONE;
        if (s_line_track.timer_running) {
            s_line_track.elapsed_time_ms = now - s_line_track.start_tick_ms;
            s_line_track.final_time_ms = s_line_track.elapsed_time_ms;
            s_line_track.timer_running = false;
        }
        s_line_track.state = LINE_TRACK_STATE_IDLE;
        s_line_track.car_mode = CAR_MODE_STOP;
        line_track_control_reset(&s_line_track.track);
        line_track_stop(&s_line_track);
    }

    if (s_line_track.timer_running) {
        s_line_track.elapsed_time_ms = now - s_line_track.start_tick_ms;
    }

    if ((s_start_request != LINE_TRACK_REQUEST_NONE) &&
        !line_track_state_is_running(s_line_track.state)) {
        uint8_t request = s_start_request;
        s_start_request = LINE_TRACK_REQUEST_NONE;
        if (request == (uint8_t)LINE_TRACK_MODE_FAST) {
            line_track_start(LINE_TRACK_STATE_TASK1_TRACKING, now);
        } else if (request == (uint8_t)LINE_TRACK_MODE_STRAIGHT) {
            line_track_start(LINE_TRACK_STATE_TASK3_BALANCE, now);
        } else if (request == (uint8_t)LINE_TRACK_MODE_SLOW) {
            line_track_start(LINE_TRACK_STATE_TASK4_BALANCE, now);
        }
    }

    /* 与 F4-MIX 100 Hz 任务保持相同的五步顺序。 */
    line_track_feedback_update(&s_line_track);
    line_track_set_control(&s_line_track);
    line_track_update_balance_feedforward(&s_line_track);
    line_track_control_loop(&s_line_track);
    line_track_send_command(&s_line_track);
    line_track_publish_debug();
}

uint8_t LineTrack_RequestStart(LineTrackMode_t mode)
{
    if ((s_initialized == 0U) ||
        ((uint32_t)mode > (uint32_t)LINE_TRACK_MODE_SLOW) ||
        line_track_state_is_running(s_line_track.state)) {
        return 0U;
    }

    if ((mode == LINE_TRACK_MODE_SLOW) &&
        (Balance_IsClosedLoopActive() == 0U)) {
        g_line_track_debug.balance_not_ready_reject_count++;
        return 0U;
    }

    s_start_request = (uint8_t)mode;
    return 1U;
}

uint8_t LineTrack_RequestStop(void)
{
    if (s_initialized == 0U) {
        return 0U;
    }

    s_start_request = LINE_TRACK_REQUEST_NONE;
    s_stop_request = 1U;
    g_line_track_debug.stop_request_count++;
    return 1U;
}

uint8_t LineTrack_GetTelemetrySnapshot(LineTrackDebug_t *snapshot)
{
    if (snapshot == NULL) {
        return 0U;
    }

    snapshot->initialized = g_line_track_debug.initialized;
    snapshot->state = g_line_track_debug.state;
    snapshot->car_mode = g_line_track_debug.car_mode;
    snapshot->track_bits = g_line_track_debug.track_bits;
    snapshot->line_error = g_line_track_debug.line_error;
    snapshot->left_speed = g_line_track_debug.left_speed;
    snapshot->right_speed = g_line_track_debug.right_speed;
    snapshot->left_speed_set = g_line_track_debug.left_speed_set;
    snapshot->right_speed_set = g_line_track_debug.right_speed_set;
    snapshot->left_pwm = g_line_track_debug.left_pwm;
    snapshot->right_pwm = g_line_track_debug.right_pwm;
    snapshot->left_encoder_fault = g_line_track_debug.left_encoder_fault;
    snapshot->right_encoder_fault = g_line_track_debug.right_encoder_fault;
    snapshot->finish_pending = g_line_track_debug.finish_pending;
    snapshot->finish_armed = g_line_track_debug.finish_armed;
    snapshot->elapsed_time_ms = g_line_track_debug.elapsed_time_ms;
    snapshot->final_time_ms = g_line_track_debug.final_time_ms;
    snapshot->fast_start_count = g_line_track_debug.fast_start_count;
    snapshot->slow_start_count = g_line_track_debug.slow_start_count;
    snapshot->straight_start_count = g_line_track_debug.straight_start_count;
    snapshot->stop_request_count = g_line_track_debug.stop_request_count;
    snapshot->balance_not_ready_reject_count =
        g_line_track_debug.balance_not_ready_reject_count;
    snapshot->recovery_enter_count = g_line_track_debug.recovery_enter_count;
    snapshot->control_count = g_line_track_debug.control_count;
    return 1U;
}

uint8_t LineTrack_GetBalanceFeedforwardSnapshot(
    LineTrackBalanceFeedforward_t *snapshot)
{
    if (snapshot == NULL) {
        return 0U;
    }

    snapshot->active = g_line_track_debug.balance_feedforward_active;
    snapshot->source_speed = g_line_track_debug.acceleration_source_speed;
    snapshot->acceleration_filtered =
        g_line_track_debug.longitudinal_acceleration_filtered;
    snapshot->feedforward_degrees =
        g_line_track_debug.acceleration_feedforward_degrees;
    return 1U;
}
