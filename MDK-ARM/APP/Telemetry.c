#include "Telemetry.h"

#include "Balance_Control.h"
#include "Camera.h"
#include "Motor_Command.h"
#include "usart.h"

#include <limits.h>
#include <string.h>

#define TELEMETRY_TX_BUFFER_SIZE  768U

typedef enum
{
    TELEMETRY_FRAME_HELLO = 0,
    TELEMETRY_FRAME_CONFIG,
    TELEMETRY_FRAME_SAMPLE
} TelemetryFrameType_t;

volatile TelemetryDebug_t g_telemetry_debug;

static uint8_t s_tx_buffer[TELEMETRY_TX_BUFFER_SIZE];
static uint32_t s_last_sample_tick;
static uint32_t s_last_config_tick;
static uint8_t s_hello_pending;
static uint8_t s_config_pending;

static uint8_t Telemetry_AppendChar(uint16_t *length, char value)
{
    if (*length >= TELEMETRY_TX_BUFFER_SIZE) {
        return 0U;
    }
    s_tx_buffer[*length] = (uint8_t)value;
    (*length)++;
    return 1U;
}

static uint8_t Telemetry_AppendText(uint16_t *length, const char *text)
{
    while (*text != '\0') {
        if (Telemetry_AppendChar(length, *text) == 0U) {
            return 0U;
        }
        text++;
    }
    return 1U;
}

static uint8_t Telemetry_AppendUnsigned(uint16_t *length, uint32_t value)
{
    char digits[10];
    uint8_t count = 0U;

    do {
        digits[count++] = (char)('0' + (value % 10U));
        value /= 10U;
    } while ((value != 0U) && (count < sizeof(digits)));

    while (count != 0U) {
        if (Telemetry_AppendChar(length, digits[--count]) == 0U) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t Telemetry_AppendSigned(uint16_t *length, int32_t value)
{
    uint32_t magnitude;

    if (value < 0) {
        if (Telemetry_AppendChar(length, '-') == 0U) {
            return 0U;
        }
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    return Telemetry_AppendUnsigned(length, magnitude);
}

static int32_t Telemetry_ScaleFloat(float value, float scale)
{
    float scaled = value * scale;

    if (scaled > 2147483000.0f) {
        return INT32_MAX;
    }
    if (scaled < -2147483000.0f) {
        return INT32_MIN;
    }
    return (scaled >= 0.0f) ? (int32_t)(scaled + 0.5f) :
                              (int32_t)(scaled - 0.5f);
}

static uint8_t Telemetry_AppendUnsignedField(uint16_t *length, uint32_t value)
{
    return (uint8_t)(Telemetry_AppendChar(length, ',') &&
                     Telemetry_AppendUnsigned(length, value));
}

static uint8_t Telemetry_AppendSignedField(uint16_t *length, int32_t value)
{
    return (uint8_t)(Telemetry_AppendChar(length, ',') &&
                     Telemetry_AppendSigned(length, value));
}

static uint8_t Telemetry_AppendScaledField(uint16_t *length,
                                            float value,
                                            float scale)
{
    return Telemetry_AppendSignedField(length,
                                       Telemetry_ScaleFloat(value, scale));
}

static uint8_t Telemetry_FinishLine(uint16_t *length)
{
    return (uint8_t)(Telemetry_AppendChar(length, '\r') &&
                     Telemetry_AppendChar(length, '\n'));
}

static uint16_t Telemetry_BuildHello(void)
{
    uint16_t length = 0U;

    if ((!Telemetry_AppendText(&length, "HELLO,F407_BALANCE")) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         TELEMETRY_PROTOCOL_VERSION)) ||
        (!Telemetry_AppendUnsignedField(&length, TELEMETRY_BAUD_RATE)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         TELEMETRY_SAMPLE_PERIOD_MS)) ||
        (!Telemetry_FinishLine(&length))) {
        return 0U;
    }
    return length;
}

static uint16_t Telemetry_BuildConfig(uint32_t now)
{
    uint16_t length = 0U;

    if ((!Telemetry_AppendText(&length, "CFG")) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         TELEMETRY_PROTOCOL_VERSION)) ||
        (!Telemetry_AppendUnsignedField(&length, now)) ||
        (!Telemetry_AppendScaledField(&length, g_balance_tuning.kp, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length, g_balance_tuning.ki, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length, g_balance_tuning.kd, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.command_centering_gain_s, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.velocity_old_weight, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.integral_window_mm, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.max_slew_pulse_per_s, 100.0f)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.max_step_pulse)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.soft_min_pulse)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.soft_max_pulse)) ||
        (!Telemetry_AppendUnsignedField(&length,
            g_balance_tuning.camera_fresh_ms)) ||
        (!Telemetry_AppendUnsignedField(&length,
            g_balance_tuning.vision_lost_confirm_frames)) ||
        (!Telemetry_AppendUnsignedField(&length,
            g_balance_tuning.vision_continue_ms)) ||
        (!Telemetry_AppendSignedField(&length, MOTOR_HARD_MIN_PULSE)) ||
        (!Telemetry_AppendSignedField(&length, MOTOR_HARD_MAX_PULSE)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.positive_target_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.negative_target_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.trajectory_start_position_tolerance_mm)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_start_speed_tolerance_mm_s, 100.0f)) ||
        (!Telemetry_AppendUnsignedField(&length,
            g_balance_tuning.trajectory_start_confirm_samples)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.positive_position_tolerance_mm)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.positive_speed_tolerance_mm_s, 100.0f)) ||
        (!Telemetry_AppendUnsignedField(&length,
            g_balance_tuning.positive_confirm_samples)) ||
        (!Telemetry_AppendSignedField(&length,
            g_balance_tuning.negative_position_tolerance_mm)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.negative_speed_tolerance_mm_s, 100.0f)) ||
        (!Telemetry_AppendUnsignedField(&length,
            g_balance_tuning.negative_stable_ms)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_kp_scale, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_ki_scale, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_kd_scale, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_follow_gain_scale, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_slew_scale, 1000.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            g_balance_tuning.trajectory_step_scale, 1000.0f)) ||
        (!Telemetry_AppendUnsignedField(&length,
            BALANCE_TRAJECTORY_SCORE_LIMIT_MS)) ||
        (!Telemetry_AppendSignedField(&length,
            BALANCE_TRAJECTORY_OFFICIAL_ERROR_MM)) ||
        (!Telemetry_FinishLine(&length))) {
        return 0U;
    }
    return length;
}

static uint16_t Telemetry_BuildSample(uint32_t now)
{
    BalanceTelemetrySnapshot_t sample;
    uint16_t length = 0U;

    if (Balance_GetTelemetrySnapshot(&sample) == 0U) {
        return 0U;
    }

    if ((!Telemetry_AppendText(&length, "TEL")) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         TELEMETRY_PROTOCOL_VERSION)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         g_telemetry_debug.sequence + 1U)) ||
        (!Telemetry_AppendUnsignedField(&length, now)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.state)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.fault)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.start_reject_reason)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.start_ready)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.camera_online)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.target_found)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.last_valid_age_ms)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.sample_dt_ms)) ||
        (!Telemetry_AppendSignedField(&length, sample.target_mm)) ||
        (!Telemetry_AppendSignedField(&length, sample.position_mm)) ||
        (!Telemetry_AppendScaledField(&length, sample.velocity_mm_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length, sample.error_mm, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
                                       sample.p_rate_pulse_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
                                       sample.i_rate_pulse_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
                                       sample.d_rate_pulse_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
            sample.centering_rate_pulse_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
                                       sample.raw_rate_pulse_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
                                       sample.limited_rate_pulse_s, 100.0f)) ||
        (!Telemetry_AppendScaledField(&length,
                                       sample.command_position_pulse, 100.0f)) ||
        (!Telemetry_AppendSignedField(&length,
                                       sample.final_requested_pulse)) ||
        (!Telemetry_AppendSignedField(&length,
                                       sample.last_transmitted_pulse)) ||
        (!Telemetry_AppendSignedField(&length, sample.last_acked_pulse)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.motor_owner)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.motor_state)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.motor_retry_count)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.motor_ack_status)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.motor_last_result)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.motor_pending_valid)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.motor_tx_fail_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.motor_ack_timeout_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.motor_ack_reject_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.hard_limit_reject_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.vision_lost_streak)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.vision_hold_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.tuning_error_mask)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.frame_sequence)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.valid_sequence)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         g_camera_debug.parse_error_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         g_camera_debug.physical_reject_count)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         g_camera_debug.uart_error_count)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.run_elapsed_ms)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.trajectory_ready)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.trajectory_started)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.trajectory_completed)) ||
        (!Telemetry_AppendUnsignedField(&length,
                                         sample.trajectory_ready_count)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.completion_time_ms)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.score_pass)) ||
        (!Telemetry_AppendUnsignedField(&length,
            sample.official_endpoint_error_pass)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.official_time_pass)) ||
        (!Telemetry_AppendUnsignedField(&length, sample.run_fault_free)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.positive_turn_position_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.positive_turn_abs_error_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.positive_peak_position_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.positive_overshoot_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.negative_min_position_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.negative_overshoot_mm)) ||
        (!Telemetry_AppendSignedField(&length,
            sample.negative_stable_window_max_abs_error_mm)) ||
        (!Telemetry_FinishLine(&length))) {
        return 0U;
    }
    return length;
}

static uint8_t Telemetry_StartTransmit(uint16_t length,
                                        TelemetryFrameType_t frame_type)
{
    HAL_StatusTypeDef status;

    if (length == 0U) {
        g_telemetry_debug.build_error_count++;
        return 0U;
    }
    if (huart3.gState != HAL_UART_STATE_READY) {
        g_telemetry_debug.busy_observation_count++;
        return 0U;
    }

    status = HAL_UART_Transmit_DMA(&huart3, s_tx_buffer, length);
    g_telemetry_debug.last_tx_status = (uint8_t)status;
    if (status != HAL_OK) {
        g_telemetry_debug.tx_error_count++;
        return 0U;
    }

    g_telemetry_debug.last_tx_len = length;
    g_telemetry_debug.last_tx_tick = HAL_GetTick();
    g_telemetry_debug.tx_frame_count++;
    g_telemetry_debug.tx_byte_count += length;
    if (frame_type == TELEMETRY_FRAME_HELLO) {
        g_telemetry_debug.hello_frame_count++;
    } else if (frame_type == TELEMETRY_FRAME_CONFIG) {
        g_telemetry_debug.config_frame_count++;
    } else {
        g_telemetry_debug.sample_frame_count++;
        g_telemetry_debug.sequence++;
    }
    return 1U;
}

void Telemetry_Init(void)
{
    uint32_t now = HAL_GetTick();

    memset((void *)&g_telemetry_debug, 0, sizeof(g_telemetry_debug));
    memset(s_tx_buffer, 0, sizeof(s_tx_buffer));
    g_telemetry_debug.enabled = 1U;
    g_telemetry_debug.protocol_version = TELEMETRY_PROTOCOL_VERSION;
    g_telemetry_debug.sample_period_ms = TELEMETRY_SAMPLE_PERIOD_MS;
    s_last_sample_tick = now;
    s_last_config_tick = now;
    s_hello_pending = 1U;
    s_config_pending = 1U;
}

void Telemetry_Process(void)
{
    uint32_t now;
    uint16_t length;

    if (g_telemetry_debug.enabled == 0U) {
        g_telemetry_debug.tx_busy = 0U;
        return;
    }

    now = HAL_GetTick();
    g_telemetry_debug.tx_busy =
        (uint8_t)(huart3.gState != HAL_UART_STATE_READY);
    if (g_telemetry_debug.tx_busy != 0U) {
        return;
    }

    if (s_hello_pending != 0U) {
        length = Telemetry_BuildHello();
        if (Telemetry_StartTransmit(length, TELEMETRY_FRAME_HELLO) != 0U) {
            s_hello_pending = 0U;
        }
        return;
    }

    if ((uint32_t)(now - s_last_config_tick) >=
        TELEMETRY_CONFIG_PERIOD_MS) {
        s_config_pending = 1U;
    }
    if (s_config_pending != 0U) {
        length = Telemetry_BuildConfig(now);
        if (Telemetry_StartTransmit(length, TELEMETRY_FRAME_CONFIG) != 0U) {
            s_config_pending = 0U;
            s_last_config_tick = now;
        }
        return;
    }

    if ((uint32_t)(now - s_last_sample_tick) <
        TELEMETRY_SAMPLE_PERIOD_MS) {
        return;
    }

    length = Telemetry_BuildSample(now);
    if (Telemetry_StartTransmit(length, TELEMETRY_FRAME_SAMPLE) != 0U) {
        s_last_sample_tick = now;
    }
}
