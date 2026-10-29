#include "Motor_Calibration.h"

#include "Emm_V5.h"
#include "Motor_Command.h"
#include "Motor_Feedback.h"

#include <limits.h>
#include <string.h>

#define MOTOR_CAL_DEFAULT_ADDRESS          1U
#define MOTOR_CAL_DEFAULT_VELOCITY_RPM    10U
#define MOTOR_CAL_DEFAULT_ACCELERATION      0U
#define MOTOR_CAL_DEFAULT_JOG_PULSES       10U
#define MOTOR_CAL_DEFAULT_GUARD_PULSES \
    MOTOR_CAL_DISCOVERY_EXTRA_PULSE
#define MOTOR_CAL_DEFAULT_SAFETY_PERCENT  100U
#define MOTOR_CAL_MAX_JOG_PULSES           1000U

typedef enum
{
    MOTOR_CAL_ASYNC_NONE = 0,
    MOTOR_CAL_ASYNC_SET_ZERO,
    MOTOR_CAL_ASYNC_ENABLE,
    MOTOR_CAL_ASYNC_POSITION,
    MOTOR_CAL_ASYNC_QUERY,
    MOTOR_CAL_ASYNC_STOP,
    MOTOR_CAL_ASYNC_DISABLE
} MotorCalibrationAsyncAction_t;

volatile MotorCalibrationState_t g_motor_calibration;

static uint32_t s_request_id;
static MotorCalibrationAsyncAction_t s_async_action;
static int32_t s_requested_position;

static void Motor_CalibrationSetResult(MotorCalibrationResult_t result)
{
    g_motor_calibration.last_result = (uint8_t)result;
    g_motor_calibration.last_tx_status = g_emm_debug.last_tx_status;
    g_motor_calibration.last_tx_success_count = g_emm_debug.tx_success_count;

    if (result == MOTOR_CAL_RESULT_OK) {
        g_motor_calibration.executed_command_count++;
    } else if (result == MOTOR_CAL_RESULT_TX_FAILED) {
        g_motor_calibration.tx_failed_count++;
    } else if (result != MOTOR_CAL_RESULT_IDLE) {
        g_motor_calibration.rejected_command_count++;
    }
}

static uint8_t Motor_CalibrationParametersValid(void)
{
    return (uint8_t)((g_motor_calibration.motor_address == MOTOR_COMMAND_ADDRESS) &&
                     (g_motor_calibration.velocity_rpm ==
                      MOTOR_CALIBRATION_POSITION_VELOCITY_RPM) &&
                     (g_motor_calibration.acceleration ==
                      MOTOR_CAL_DEFAULT_ACCELERATION) &&
                     (g_motor_calibration.jog_pulses >= 1U) &&
                     (g_motor_calibration.jog_pulses <= MOTOR_CAL_MAX_JOG_PULSES) &&
                     (g_motor_calibration.discovery_guard_pulses ==
                      MOTOR_CAL_DISCOVERY_EXTRA_PULSE) &&
                     (g_motor_calibration.safety_percent >= 10U) &&
                     (g_motor_calibration.safety_percent <= 100U));
}

static void Motor_CalibrationAcceptZero(void)
{
    g_motor_calibration.position_pulses = 0;
    g_motor_calibration.captured_limit_a_pulses = MOTOR_HARD_MAX_PULSE;
    g_motor_calibration.captured_limit_b_pulses = MOTOR_HARD_MIN_PULSE;
    g_motor_calibration.safe_min_pulses = MOTOR_HARD_MIN_PULSE;
    g_motor_calibration.safe_max_pulses = MOTOR_HARD_MAX_PULSE;
    g_motor_calibration.capture_a_valid = 1U;
    g_motor_calibration.capture_b_valid = 1U;
    g_motor_calibration.limits_valid = 1U;
    g_motor_calibration.zero_valid = 1U;
    g_motor_calibration.armed = 0U;
    g_motor_calibration.startup_zero_complete = 0U;
}

static MotorCalibrationResult_t Motor_CalibrationMapResult(MotorRequestResult_t result)
{
    switch (result) {
    case MOTOR_REQUEST_TX_FAILED:
        return MOTOR_CAL_RESULT_TX_FAILED;
    case MOTOR_REQUEST_ACK_TIMEOUT:
        return MOTOR_CAL_RESULT_RESPONSE_TIMEOUT;
    case MOTOR_REQUEST_DEVICE_REJECTED:
        return MOTOR_CAL_RESULT_DEVICE_REJECTED;
    case MOTOR_REQUEST_HARD_LIMIT_REJECTED:
        return MOTOR_CAL_RESULT_REJECTED_LIMIT;
    default:
        return MOTOR_CAL_RESULT_TX_FAILED;
    }
}

static void Motor_CalibrationFinishAsync(MotorRequestResult_t result)
{
    MotorCalibrationAsyncAction_t completed_action = s_async_action;

    s_request_id = 0U;
    s_async_action = MOTOR_CAL_ASYNC_NONE;

    if (result != MOTOR_REQUEST_SUCCESS) {
        Motor_CalibrationSetResult(Motor_CalibrationMapResult(result));
        if ((completed_action == MOTOR_CAL_ASYNC_SET_ZERO) ||
            (completed_action == MOTOR_CAL_ASYNC_QUERY)) {
            (void)Motor_CommandRelease(MOTOR_OWNER_CALIBRATION);
        }
        return;
    }

    switch (completed_action) {
    case MOTOR_CAL_ASYNC_SET_ZERO:
        Motor_CalibrationAcceptZero();
        (void)Motor_CommandRelease(MOTOR_OWNER_CALIBRATION);
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        break;

    case MOTOR_CAL_ASYNC_ENABLE:
        g_motor_calibration.motor_enabled = 1U;
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        break;

    case MOTOR_CAL_ASYNC_POSITION:
        g_motor_calibration.position_pulses = s_requested_position;
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        break;

    case MOTOR_CAL_ASYNC_QUERY:
        if (g_motor_feedback.position_valid == 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_POSITION_INVALID);
        } else {
            g_motor_calibration.sweep_last_degrees =
                g_motor_feedback.position_degrees;
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        }
        (void)Motor_CommandRelease(MOTOR_OWNER_CALIBRATION);
        break;

    case MOTOR_CAL_ASYNC_STOP:
        /* STOP 为安全抢占下的 best effort；TX 完成后继续发送失能。 */
        s_request_id = Motor_CommandRequestDisable(MOTOR_OWNER_SAFETY);
        s_async_action = MOTOR_CAL_ASYNC_DISABLE;
        break;

    case MOTOR_CAL_ASYNC_DISABLE:
        g_motor_calibration.armed = 0U;
        g_motor_calibration.motor_enabled = 0U;
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        (void)Motor_CommandRelease(MOTOR_OWNER_SAFETY);
        break;

    default:
        break;
    }
}

static void Motor_CalibrationServiceAsync(void)
{
    MotorRequestResult_t result;

    if (s_async_action == MOTOR_CAL_ASYNC_NONE) {
        return;
    }

    /* 安全抢占时，上一帧 DMA 可能仍在物理发送；等串口空闲后再提交 STOP/DISABLE。 */
    if (s_request_id == 0U) {
        if (s_async_action == MOTOR_CAL_ASYNC_STOP) {
            s_request_id = Motor_CommandRequestStop(MOTOR_OWNER_SAFETY);
        } else if (s_async_action == MOTOR_CAL_ASYNC_DISABLE) {
            s_request_id = Motor_CommandRequestDisable(MOTOR_OWNER_SAFETY);
        }
        return;
    }

    result = Motor_CommandGetResult(s_request_id);
    if ((result == MOTOR_REQUEST_PENDING) ||
        (result == MOTOR_REQUEST_UNKNOWN)) {
        return;
    }
    Motor_CalibrationFinishAsync(result);
}

void Motor_CalibrationInit(bool auto_zero_on_power_on)
{
    (void)auto_zero_on_power_on;
    memset((void *)&g_motor_calibration, 0, sizeof(g_motor_calibration));
    g_motor_calibration.motor_address = MOTOR_CAL_DEFAULT_ADDRESS;
    g_motor_calibration.velocity_rpm = MOTOR_CAL_DEFAULT_VELOCITY_RPM;
    g_motor_calibration.acceleration = MOTOR_CAL_DEFAULT_ACCELERATION;
    g_motor_calibration.safety_percent = MOTOR_CAL_DEFAULT_SAFETY_PERCENT;
    g_motor_calibration.jog_pulses = MOTOR_CAL_DEFAULT_JOG_PULSES;
    g_motor_calibration.discovery_guard_pulses = MOTOR_CAL_DEFAULT_GUARD_PULSES;
    g_motor_calibration.safe_min_pulses = MOTOR_HARD_MIN_PULSE;
    g_motor_calibration.safe_max_pulses = MOTOR_HARD_MAX_PULSE;
    g_motor_calibration.last_result = MOTOR_CAL_RESULT_IDLE;
    g_motor_calibration.sweep_phase = MOTOR_CAL_SWEEP_IDLE;

    /* 无论传入值如何，生产版本均禁止冷上电自动清零、使能和端点 sweep。 */
    g_motor_calibration.startup_zero_requested = 0U;
    g_motor_calibration.startup_zero_pending = 0U;
    g_motor_calibration.sweep_requested = 0U;
    g_motor_calibration.sweep_active = 0U;
    s_request_id = 0U;
    s_async_action = MOTOR_CAL_ASYNC_NONE;
    s_requested_position = 0;
}

static bool Motor_CalibrationCommandAbsolute(int32_t target_pulses,
                                             uint8_t allow_discovery)
{
    if ((g_motor_calibration.zero_valid == 0U) ||
        (g_motor_calibration.limits_valid == 0U)) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_ZERO_UNKNOWN);
        return false;
    }
    if (g_motor_calibration.armed == 0U) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_NOT_ARMED);
        return false;
    }
    if (g_motor_calibration.motor_enabled == 0U) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_NOT_ENABLED);
        return false;
    }
    if ((g_motor_calibration.safe_min_pulses <
         MOTOR_CAL_DISCOVERY_MIN_PULSE) ||
        (g_motor_calibration.safe_max_pulses >
         MOTOR_CAL_DISCOVERY_MAX_PULSE) ||
        (g_motor_calibration.safe_min_pulses >=
         g_motor_calibration.safe_max_pulses)) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_PARAMETER);
        return false;
    }
    if (allow_discovery != 0U) {
        if ((target_pulses < MOTOR_CAL_DISCOVERY_MIN_PULSE) ||
            (target_pulses > MOTOR_CAL_DISCOVERY_MAX_PULSE)) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_LIMIT);
            return false;
        }
    } else {
        if ((target_pulses < MOTOR_HARD_MIN_PULSE) ||
            (target_pulses > MOTOR_HARD_MAX_PULSE) ||
            (target_pulses < g_motor_calibration.safe_min_pulses) ||
            (target_pulses > g_motor_calibration.safe_max_pulses)) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_LIMIT);
            return false;
        }
    }
    if ((s_async_action != MOTOR_CAL_ASYNC_NONE) ||
        (Motor_CommandGetOwner() != MOTOR_OWNER_CALIBRATION)) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
        return false;
    }

    s_request_id = Motor_CommandRequestAbsolute(MOTOR_OWNER_CALIBRATION,
                                                target_pulses);
    if (s_request_id == 0U) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
        return false;
    }
    s_requested_position = target_pulses;
    s_async_action = MOTOR_CAL_ASYNC_POSITION;
    return true;
}

bool Motor_CalibrationCommandSafeAbsolute(int32_t target_pulses)
{
    return Motor_CalibrationCommandAbsolute(target_pulses, 0U);
}

void Motor_CalibrationProcess(void)
{
    uint32_t command;
    int64_t candidate64;
    int32_t raw_min;
    int32_t raw_max;

    Motor_CalibrationServiceAsync();

    command = g_motor_calibration.command;
    if (command == MOTOR_CAL_COMMAND_IDLE) {
        return;
    }

    g_motor_calibration.command = MOTOR_CAL_COMMAND_IDLE;
    g_motor_calibration.last_command = (uint8_t)command;

    if (command == MOTOR_CAL_COMMAND_STOP_DISABLE) {
        Motor_CommandSafetyTakeover();
        g_motor_calibration.sweep_requested = 0U;
        g_motor_calibration.sweep_active = 0U;
        g_motor_calibration.sweep_phase = MOTOR_CAL_SWEEP_STOPPED;
        g_motor_calibration.armed = 0U;
        s_request_id = 0U;
        s_async_action = MOTOR_CAL_ASYNC_STOP;
        Motor_CalibrationServiceAsync();
        return;
    }

    if (s_async_action != MOTOR_CAL_ASYNC_NONE) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
        return;
    }
    if (Motor_CalibrationParametersValid() == 0U) {
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_PARAMETER);
        return;
    }

    switch ((MotorCalibrationCommand_t)command) {
    case MOTOR_CAL_COMMAND_SET_ZERO:
        if (g_motor_calibration.motor_enabled != 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_NOT_ENABLED);
            break;
        }
        if (Motor_CommandAcquire(MOTOR_OWNER_CALIBRATION) == 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
            break;
        }
        s_request_id = Motor_CommandRequestResetZero(MOTOR_OWNER_CALIBRATION);
        if (s_request_id == 0U) {
            (void)Motor_CommandRelease(MOTOR_OWNER_CALIBRATION);
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
            break;
        }
        s_async_action = MOTOR_CAL_ASYNC_SET_ZERO;
        break;

    case MOTOR_CAL_COMMAND_ARM:
        if (g_motor_calibration.zero_valid == 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_ZERO_UNKNOWN);
        } else {
            g_motor_calibration.armed = 1U;
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        }
        break;

    case MOTOR_CAL_COMMAND_ENABLE:
        if ((g_motor_calibration.zero_valid == 0U) ||
            (g_motor_calibration.armed == 0U)) {
            Motor_CalibrationSetResult(
                (g_motor_calibration.zero_valid == 0U) ?
                MOTOR_CAL_RESULT_REJECTED_ZERO_UNKNOWN :
                MOTOR_CAL_RESULT_REJECTED_NOT_ARMED);
            break;
        }
        if (Motor_CommandAcquire(MOTOR_OWNER_CALIBRATION) == 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
            break;
        }
        s_request_id = Motor_CommandRequestEnable(MOTOR_OWNER_CALIBRATION);
        if (s_request_id == 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
            break;
        }
        s_async_action = MOTOR_CAL_ASYNC_ENABLE;
        break;

    case MOTOR_CAL_COMMAND_JOG_POSITIVE:
    case MOTOR_CAL_COMMAND_JOG_NEGATIVE:
        candidate64 = (int64_t)g_motor_calibration.position_pulses;
        /* 命令 5 增加正脉冲使球向左；命令 6 增加负脉冲使球向右。 */
        candidate64 += (command == MOTOR_CAL_COMMAND_JOG_POSITIVE) ?
                       (int64_t)g_motor_calibration.jog_pulses :
                       -(int64_t)g_motor_calibration.jog_pulses;
        if ((candidate64 < INT32_MIN) || (candidate64 > INT32_MAX) ||
            (candidate64 < MOTOR_CAL_DISCOVERY_MIN_PULSE) ||
            (candidate64 > MOTOR_CAL_DISCOVERY_MAX_PULSE)) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_LIMIT);
            break;
        }
        (void)Motor_CalibrationCommandAbsolute((int32_t)candidate64, 1U);
        break;

    case MOTOR_CAL_COMMAND_CAPTURE_LIMIT_A:
        g_motor_calibration.captured_limit_a_pulses =
            g_motor_calibration.position_pulses;
        g_motor_calibration.capture_a_valid = 1U;
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        break;

    case MOTOR_CAL_COMMAND_CAPTURE_LIMIT_B:
        g_motor_calibration.captured_limit_b_pulses =
            g_motor_calibration.position_pulses;
        g_motor_calibration.capture_b_valid = 1U;
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        break;

    case MOTOR_CAL_COMMAND_BUILD_SAFE_LIMITS:
        raw_min = (g_motor_calibration.captured_limit_a_pulses <
                   g_motor_calibration.captured_limit_b_pulses) ?
                  g_motor_calibration.captured_limit_a_pulses :
                  g_motor_calibration.captured_limit_b_pulses;
        raw_max = (g_motor_calibration.captured_limit_a_pulses >
                   g_motor_calibration.captured_limit_b_pulses) ?
                  g_motor_calibration.captured_limit_a_pulses :
                  g_motor_calibration.captured_limit_b_pulses;
        if ((raw_min >= 0) || (raw_max <= 0) ||
            (raw_min < MOTOR_CAL_DISCOVERY_MIN_PULSE) ||
            (raw_max > MOTOR_CAL_DISCOVERY_MAX_PULSE)) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_CAPTURE);
            break;
        }
        /* 仅更新人工观察镜像；最终发送硬门固定为右端 -340、左端 +330。 */
        g_motor_calibration.safe_min_pulses = raw_min;
        g_motor_calibration.safe_max_pulses = raw_max;
        g_motor_calibration.limits_valid = 1U;
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_OK);
        break;

    case MOTOR_CAL_COMMAND_GOTO_ZERO:
        (void)Motor_CalibrationCommandSafeAbsolute(0);
        break;

    case MOTOR_CAL_COMMAND_QUERY_POSITION:
        if (Motor_CommandAcquire(MOTOR_OWNER_CALIBRATION) == 0U) {
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
            break;
        }
        s_request_id = Motor_CommandRequestQueryPosition(MOTOR_OWNER_CALIBRATION);
        if (s_request_id == 0U) {
            (void)Motor_CommandRelease(MOTOR_OWNER_CALIBRATION);
            Motor_CalibrationSetResult(MOTOR_CAL_RESULT_SWEEP_BUSY);
            break;
        }
        s_async_action = MOTOR_CAL_ASYNC_QUERY;
        break;

    default:
        Motor_CalibrationSetResult(MOTOR_CAL_RESULT_REJECTED_PARAMETER);
        break;
    }
}
