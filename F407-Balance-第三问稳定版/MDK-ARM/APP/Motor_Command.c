#include "Motor_Command.h"

#include "Emm_V5.h"
#include "Motor_Feedback.h"

#include <string.h>

#define MOTOR_COMMAND_ACK_TIMEOUT_MS       30U
#define MOTOR_COMMAND_TX_TIMEOUT_MS         10U
#define MOTOR_COMMAND_MAX_RETRY             1U
#define MOTOR_COMMAND_BALANCE_POSITION_VELOCITY  300U
#define MOTOR_COMMAND_POSITION_ACCEL       200U
#define MOTOR_COMMAND_HISTORY_SIZE          8U

typedef struct
{
    uint32_t request_id;
    MotorCommandType_t type;
    MotorOwner_t owner;
    int32_t target_pulse;
    uint8_t function;
    uint8_t expect_ack;
    uint8_t retry_count;
} MotorActiveRequest_t;

typedef struct
{
    uint32_t request_id;
    uint8_t result;
} MotorResultHistory_t;

volatile MotorCommandDebug_t g_motor_command_debug;

static MotorActiveRequest_t s_active;
static MotorResultHistory_t s_history[MOTOR_COMMAND_HISTORY_SIZE];
static uint8_t s_history_index;
static uint32_t s_next_request_id;

static uint32_t Motor_CommandAllocateId(void)
{
    s_next_request_id++;
    if (s_next_request_id == 0U) {
        s_next_request_id = 1U;
    }
    return s_next_request_id;
}

static void Motor_CommandRecordResult(uint32_t request_id,
                                      MotorCommandType_t type,
                                      MotorOwner_t owner,
                                      MotorRequestResult_t result)
{
    if (request_id == 0U) {
        return;
    }
    s_history[s_history_index].request_id = request_id;
    s_history[s_history_index].result = (uint8_t)result;
    s_history_index = (uint8_t)((s_history_index + 1U) % MOTOR_COMMAND_HISTORY_SIZE);

    g_motor_command_debug.last_completed_request_id = request_id;
    g_motor_command_debug.last_completed_result = (uint8_t)result;
    g_motor_command_debug.last_completed_type = (uint8_t)type;
    g_motor_command_debug.last_completed_owner = (uint8_t)owner;
}

static void Motor_CommandClearActive(void)
{
    memset(&s_active, 0, sizeof(s_active));
    g_motor_command_debug.state = MOTOR_COMMAND_STATE_IDLE;
    g_motor_command_debug.command_type = MOTOR_COMMAND_NONE;
    g_motor_command_debug.request_id = 0U;
    g_motor_command_debug.retry_count = 0U;
    g_motor_command_debug.busy = 0U;
}

static HAL_StatusTypeDef Motor_CommandStartTx(void)
{
    HAL_StatusTypeDef status = HAL_ERROR;

    if (s_active.expect_ack != 0U) {
        Motor_FeedbackBeginExpectedResponse(MOTOR_COMMAND_ADDRESS,
                                            s_active.function);
    }

    switch (s_active.type) {
    case MOTOR_COMMAND_QUERY_POSITION:
        g_motor_feedback.position_valid = 0U;
        status = Emm_V5_AsyncReadPosition(MOTOR_COMMAND_ADDRESS);
        break;
    case MOTOR_COMMAND_RESET_ZERO:
        status = Emm_V5_AsyncResetCurrentPosition(MOTOR_COMMAND_ADDRESS);
        break;
    case MOTOR_COMMAND_ENABLE:
        status = Emm_V5_AsyncEnable(MOTOR_COMMAND_ADDRESS);
        break;
    case MOTOR_COMMAND_STOP:
        status = Emm_V5_AsyncStop(MOTOR_COMMAND_ADDRESS);
        break;
    case MOTOR_COMMAND_DISABLE:
        status = Emm_V5_AsyncDisable(MOTOR_COMMAND_ADDRESS);
        break;
    case MOTOR_COMMAND_ABSOLUTE_POSITION:
        status = Emm_V5_AsyncAbsolutePosition(MOTOR_COMMAND_ADDRESS,
                                              (s_active.owner ==
                                               MOTOR_OWNER_CALIBRATION) ?
                                              MOTOR_CALIBRATION_POSITION_VELOCITY_RPM :
                                              MOTOR_COMMAND_BALANCE_POSITION_VELOCITY,
                                              MOTOR_COMMAND_POSITION_ACCEL,
                                              s_active.target_pulse);
        break;
    default:
        status = HAL_ERROR;
        break;
    }

    if (status != HAL_OK) {
        if (s_active.expect_ack != 0U) {
            Motor_FeedbackCancelExpectedResponse(0U);
        }
        return status;
    }

    g_motor_command_debug.tx_tick = HAL_GetTick();
    g_motor_command_debug.state = MOTOR_COMMAND_STATE_WAIT_TX_COMPLETE;
    g_motor_command_debug.retry_count = s_active.retry_count;
    if (s_active.type == MOTOR_COMMAND_ABSOLUTE_POSITION) {
        g_motor_command_debug.last_transmitted_pulse = s_active.target_pulse;
    }
    return HAL_OK;
}

static void Motor_CommandComplete(MotorRequestResult_t result)
{
    MotorCommandType_t completed_type = s_active.type;
    MotorOwner_t completed_owner = s_active.owner;
    uint32_t completed_id = s_active.request_id;

    if ((result == MOTOR_REQUEST_SUCCESS) &&
        (completed_type == MOTOR_COMMAND_ABSOLUTE_POSITION)) {
        g_motor_command_debug.last_acked_pulse = s_active.target_pulse;
    }
    Motor_CommandRecordResult(completed_id, completed_type, completed_owner, result);
    Motor_CommandClearActive();
}

static uint8_t Motor_CommandOwnerAllowed(MotorOwner_t owner)
{
    return (uint8_t)((owner != MOTOR_OWNER_NONE) &&
                     ((MotorOwner_t)g_motor_command_debug.owner == owner));
}

static uint32_t Motor_CommandSubmit(MotorOwner_t owner,
                                    MotorCommandType_t type,
                                    int32_t target_pulse)
{
    uint32_t request_id;
    HAL_StatusTypeDef status;
    int32_t allowed_min = MOTOR_HARD_MIN_PULSE;
    int32_t allowed_max = MOTOR_HARD_MAX_PULSE;

    if (Motor_CommandOwnerAllowed(owner) == 0U) {
        g_motor_command_debug.owner_reject_count++;
        return 0U;
    }

    request_id = Motor_CommandAllocateId();

    if ((type == MOTOR_COMMAND_ABSOLUTE_POSITION) &&
        (owner == MOTOR_OWNER_CALIBRATION)) {
        allowed_min = MOTOR_CAL_DISCOVERY_MIN_PULSE;
        allowed_max = MOTOR_CAL_DISCOVERY_MAX_PULSE;
    }
    if ((type == MOTOR_COMMAND_ABSOLUTE_POSITION) &&
        ((target_pulse < allowed_min) || (target_pulse > allowed_max))) {
        g_motor_command_debug.hard_limit_reject_count++;
        Motor_CommandRecordResult(request_id, type, owner,
                                  MOTOR_REQUEST_HARD_LIMIT_REJECTED);
        return request_id;
    }

    /* 当前 ACK 已结束但合并目标尚未启动时，新视觉目标继续覆盖旧目标。 */
    if (g_motor_command_debug.pending_latest_valid != 0U) {
        if ((type == MOTOR_COMMAND_ABSOLUTE_POSITION) &&
            ((MotorOwner_t)g_motor_command_debug.owner == owner)) {
            Motor_CommandRecordResult(
                g_motor_command_debug.pending_latest_request_id,
                MOTOR_COMMAND_ABSOLUTE_POSITION, owner,
                MOTOR_REQUEST_SUPERSEDED);
            g_motor_command_debug.pending_latest_pulse = target_pulse;
            g_motor_command_debug.pending_latest_request_id = request_id;
            return request_id;
        }
        g_motor_command_debug.busy_defer_count++;
        return 0U;
    }

    if (g_motor_command_debug.busy != 0U) {
        if ((type == MOTOR_COMMAND_ABSOLUTE_POSITION) &&
            (s_active.type == MOTOR_COMMAND_ABSOLUTE_POSITION) &&
            (s_active.owner == owner)) {
            if (g_motor_command_debug.pending_latest_valid != 0U) {
                Motor_CommandRecordResult(
                    g_motor_command_debug.pending_latest_request_id,
                    MOTOR_COMMAND_ABSOLUTE_POSITION, owner,
                    MOTOR_REQUEST_SUPERSEDED);
            }
            g_motor_command_debug.pending_latest_valid = 1U;
            g_motor_command_debug.pending_latest_pulse = target_pulse;
            g_motor_command_debug.pending_latest_request_id = request_id;
            return request_id;
        }
        g_motor_command_debug.busy_defer_count++;
        return 0U;
    }

    /* SAFETY 抢占不能中断已经在移位寄存器中的字节；等待该 DMA 真正结束再发。 */
    if (Emm_V5_AsyncTxBusy() != 0U) {
        g_motor_command_debug.busy_defer_count++;
        return 0U;
    }

    memset(&s_active, 0, sizeof(s_active));
    s_active.request_id = request_id;
    s_active.type = type;
    s_active.owner = owner;
    s_active.target_pulse = target_pulse;
    s_active.expect_ack = (uint8_t)!((owner == MOTOR_OWNER_SAFETY) &&
                                    ((type == MOTOR_COMMAND_STOP) ||
                                     (type == MOTOR_COMMAND_DISABLE)));

    switch (type) {
    case MOTOR_COMMAND_QUERY_POSITION:    s_active.function = 0x36U; break;
    case MOTOR_COMMAND_RESET_ZERO:        s_active.function = 0x0AU; break;
    case MOTOR_COMMAND_ENABLE:
    case MOTOR_COMMAND_DISABLE:           s_active.function = 0xF3U; break;
    case MOTOR_COMMAND_STOP:              s_active.function = 0xFEU; break;
    case MOTOR_COMMAND_ABSOLUTE_POSITION: s_active.function = 0xFDU; break;
    default:                              s_active.function = 0U; break;
    }

    g_motor_command_debug.busy = 1U;
    g_motor_command_debug.command_type = (uint8_t)type;
    g_motor_command_debug.request_id = request_id;

    status = Motor_CommandStartTx();
    if (status != HAL_OK) {
        g_motor_command_debug.tx_fail_count++;
        Motor_CommandComplete(MOTOR_REQUEST_TX_FAILED);
    }
    return request_id;
}

void Motor_CommandInit(void)
{
    memset((void *)&g_motor_command_debug, 0,
           sizeof(g_motor_command_debug));
    memset(&s_active, 0, sizeof(s_active));
    memset(s_history, 0, sizeof(s_history));
    s_history_index = 0U;
    s_next_request_id = 0U;
    g_motor_command_debug.owner = MOTOR_OWNER_NONE;
    g_motor_command_debug.state = MOTOR_COMMAND_STATE_IDLE;
}

uint8_t Motor_CommandAcquire(MotorOwner_t owner)
{
    MotorOwner_t current = (MotorOwner_t)g_motor_command_debug.owner;

    if ((owner == MOTOR_OWNER_NONE) || (owner == MOTOR_OWNER_SAFETY)) {
        return 0U;
    }
    if ((current == MOTOR_OWNER_NONE) || (current == owner)) {
        g_motor_command_debug.owner = (uint8_t)owner;
        return 1U;
    }
    g_motor_command_debug.owner_reject_count++;
    return 0U;
}

uint8_t Motor_CommandRelease(MotorOwner_t owner)
{
    if (((MotorOwner_t)g_motor_command_debug.owner != owner) ||
        (g_motor_command_debug.busy != 0U) ||
        (g_motor_command_debug.pending_latest_valid != 0U)) {
        return 0U;
    }
    g_motor_command_debug.owner = MOTOR_OWNER_NONE;
    return 1U;
}

void Motor_CommandSafetyTakeover(void)
{
    if ((MotorOwner_t)g_motor_command_debug.owner != MOTOR_OWNER_SAFETY) {
        g_motor_command_debug.safety_preempt_count++;
    }
    g_motor_command_debug.owner = MOTOR_OWNER_SAFETY;

    if (g_motor_command_debug.pending_latest_valid != 0U) {
        Motor_CommandRecordResult(g_motor_command_debug.pending_latest_request_id,
                                  MOTOR_COMMAND_ABSOLUTE_POSITION,
                                  s_active.owner,
                                  MOTOR_REQUEST_PREEMPTED);
        g_motor_command_debug.pending_latest_valid = 0U;
    }

    if (g_motor_command_debug.busy != 0U) {
        Motor_FeedbackCancelExpectedResponse(0U);
        Motor_CommandComplete(MOTOR_REQUEST_PREEMPTED);
    }
}

MotorOwner_t Motor_CommandGetOwner(void)
{
    return (MotorOwner_t)g_motor_command_debug.owner;
}

uint8_t Motor_CommandIsBusy(void)
{
    return (uint8_t)((g_motor_command_debug.busy != 0U) ||
                     (g_motor_command_debug.pending_latest_valid != 0U) ||
                     (Emm_V5_AsyncTxBusy() != 0U));
}

void Motor_CommandProcess(void)
{
    MotorFeedbackPollResult_t poll;
    uint32_t now = HAL_GetTick();
    HAL_StatusTypeDef status;

    if (g_motor_command_debug.busy == 0U) {
        if ((g_motor_command_debug.pending_latest_valid != 0U) &&
            ((MotorOwner_t)g_motor_command_debug.owner != MOTOR_OWNER_NONE)) {
            int32_t pulse = g_motor_command_debug.pending_latest_pulse;
            uint32_t old_id = g_motor_command_debug.pending_latest_request_id;
            MotorOwner_t owner = (MotorOwner_t)g_motor_command_debug.owner;
            g_motor_command_debug.pending_latest_valid = 0U;
            /* 保留原 pending request_id，避免合并后上层查询不到自己的请求。 */
            memset(&s_active, 0, sizeof(s_active));
            s_active.request_id = old_id;
            s_active.type = MOTOR_COMMAND_ABSOLUTE_POSITION;
            s_active.owner = owner;
            s_active.target_pulse = pulse;
            s_active.function = 0xFDU;
            s_active.expect_ack = 1U;
            g_motor_command_debug.busy = 1U;
            g_motor_command_debug.command_type = MOTOR_COMMAND_ABSOLUTE_POSITION;
            g_motor_command_debug.request_id = old_id;
            status = Motor_CommandStartTx();
            if (status != HAL_OK) {
                g_motor_command_debug.tx_fail_count++;
                Motor_CommandComplete(MOTOR_REQUEST_TX_FAILED);
            }
        }
        return;
    }

    if (g_motor_command_debug.state == MOTOR_COMMAND_STATE_WAIT_TX_COMPLETE) {
        if (Emm_V5_AsyncTxBusy() != 0U) {
            if ((uint32_t)(now - g_motor_command_debug.tx_tick) >=
                MOTOR_COMMAND_TX_TIMEOUT_MS) {
                Emm_V5_AsyncAbortTx();
                if (s_active.expect_ack != 0U) {
                    Motor_FeedbackCancelExpectedResponse(0U);
                }
                g_motor_command_debug.tx_fail_count++;
                Motor_CommandComplete(MOTOR_REQUEST_TX_FAILED);
            }
            return;
        }
        if (s_active.expect_ack == 0U) {
            Motor_CommandComplete(MOTOR_REQUEST_SUCCESS);
            return;
        }
        g_motor_command_debug.ack_deadline = now + MOTOR_COMMAND_ACK_TIMEOUT_MS;
        g_motor_command_debug.state = MOTOR_COMMAND_STATE_WAIT_ACK;
    }

    if (g_motor_command_debug.state == MOTOR_COMMAND_STATE_WAIT_ACK) {
        poll = Motor_FeedbackPollExpectedResponse();
        if (poll == MOTOR_FEEDBACK_POLL_SUCCESS) {
            g_motor_command_debug.ack_status = 1U;
            Motor_CommandComplete(MOTOR_REQUEST_SUCCESS);
            return;
        }
        if (poll == MOTOR_FEEDBACK_POLL_REJECTED) {
            g_motor_command_debug.ack_status = 2U;
            g_motor_command_debug.ack_reject_count++;
            Motor_CommandComplete(MOTOR_REQUEST_DEVICE_REJECTED);
            return;
        }
        if ((int32_t)(now - g_motor_command_debug.ack_deadline) >= 0) {
            Motor_FeedbackCancelExpectedResponse(1U);
            g_motor_command_debug.ack_timeout_count++;
            if (s_active.retry_count < MOTOR_COMMAND_MAX_RETRY) {
                s_active.retry_count++;
                g_motor_command_debug.retry_total_count++;
                g_motor_command_debug.state = MOTOR_COMMAND_STATE_RETRY_PENDING;
            } else {
                Motor_CommandComplete(MOTOR_REQUEST_ACK_TIMEOUT);
                return;
            }
        }
    }

    if (g_motor_command_debug.state == MOTOR_COMMAND_STATE_RETRY_PENDING) {
        if (Emm_V5_AsyncTxBusy() != 0U) {
            return;
        }
        status = Motor_CommandStartTx();
        if (status != HAL_OK) {
            g_motor_command_debug.tx_fail_count++;
            Motor_CommandComplete(MOTOR_REQUEST_TX_FAILED);
        }
    }
}

uint32_t Motor_CommandRequestQueryPosition(MotorOwner_t owner)
{
    return Motor_CommandSubmit(owner, MOTOR_COMMAND_QUERY_POSITION, 0);
}

uint32_t Motor_CommandRequestResetZero(MotorOwner_t owner)
{
    return Motor_CommandSubmit(owner, MOTOR_COMMAND_RESET_ZERO, 0);
}

uint32_t Motor_CommandRequestEnable(MotorOwner_t owner)
{
    return Motor_CommandSubmit(owner, MOTOR_COMMAND_ENABLE, 0);
}

uint32_t Motor_CommandRequestStop(MotorOwner_t owner)
{
    return Motor_CommandSubmit(owner, MOTOR_COMMAND_STOP, 0);
}

uint32_t Motor_CommandRequestDisable(MotorOwner_t owner)
{
    return Motor_CommandSubmit(owner, MOTOR_COMMAND_DISABLE, 0);
}

uint32_t Motor_CommandRequestAbsolute(MotorOwner_t owner, int32_t target_pulses)
{
    return Motor_CommandSubmit(owner, MOTOR_COMMAND_ABSOLUTE_POSITION,
                               target_pulses);
}

MotorRequestResult_t Motor_CommandGetResult(uint32_t request_id)
{
    uint8_t i;

    if (request_id == 0U) {
        return MOTOR_REQUEST_UNKNOWN;
    }
    if ((g_motor_command_debug.busy != 0U) &&
        (s_active.request_id == request_id)) {
        return MOTOR_REQUEST_PENDING;
    }
    if ((g_motor_command_debug.pending_latest_valid != 0U) &&
        (g_motor_command_debug.pending_latest_request_id == request_id)) {
        return MOTOR_REQUEST_PENDING;
    }
    for (i = 0U; i < MOTOR_COMMAND_HISTORY_SIZE; i++) {
        if (s_history[i].request_id == request_id) {
            return (MotorRequestResult_t)s_history[i].result;
        }
    }
    return MOTOR_REQUEST_UNKNOWN;
}
