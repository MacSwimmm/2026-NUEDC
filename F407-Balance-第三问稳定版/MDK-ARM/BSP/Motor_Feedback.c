#include "Motor_Feedback.h"

#include <string.h>

#define MOTOR_FEEDBACK_ACK_SUCCESS_STATUS  0x02U

volatile MotorFeedbackState_t g_motor_feedback;

/* USART1 回包可能被 DMA/IDLE 拆成多段，因此在中断回调之间保留当前组帧状态。 */
static uint8_t s_frame_buffer[MOTOR_FEEDBACK_MAX_FRAME_SIZE];
static uint8_t s_frame_length;
static uint8_t s_frame_target_length;

/** @brief 返回当前标定流程会使用到的张大头固定应答长度。 */
static uint8_t Motor_FeedbackLengthForFunction(uint8_t function)
{
    switch (function) {
    case 0x36U: /* 实时位置：地址、功能码、方向、4 字节位置、0x6B */
        return 8U;

    case 0x0AU: /* 当前位置清零 */
    case 0xF3U: /* 使能/失能 */
    case 0xFDU: /* 位置模式控制 */
    case 0xFEU: /* 立即停止 */
        return 4U;

    default:
        return 0U;
    }
}

static void Motor_FeedbackResetAssembler(void)
{
    s_frame_length = 0U;
    s_frame_target_length = 0U;
}

/** @brief 保存超时时已经收到但尚未成帧的数据，便于直接在 Watch 中定位。 */
static void Motor_FeedbackSavePartial(void)
{
    uint8_t copy_length = s_frame_length;

    if (copy_length > MOTOR_FEEDBACK_MAX_FRAME_SIZE) {
        copy_length = MOTOR_FEEDBACK_MAX_FRAME_SIZE;
    }
    if (copy_length != 0U) {
        memcpy((void *)g_motor_feedback.partial_data, s_frame_buffer, copy_length);
    }
    if (copy_length < MOTOR_FEEDBACK_MAX_FRAME_SIZE) {
        memset((void *)&g_motor_feedback.partial_data[copy_length], 0,
               MOTOR_FEEDBACK_MAX_FRAME_SIZE - copy_length);
    }
    g_motor_feedback.partial_length = copy_length;
}

static void Motor_FeedbackFinishFrame(void)
{
    uint8_t length = s_frame_length;
    uint8_t copy_length;
    uint32_t raw_position;

    if (length == 0U) {
        return;
    }

    copy_length = length;
    if (copy_length > MOTOR_FEEDBACK_MAX_FRAME_SIZE) {
        copy_length = MOTOR_FEEDBACK_MAX_FRAME_SIZE;
    }

    memcpy((void *)g_motor_feedback.last_frame, s_frame_buffer, copy_length);
    if (copy_length < MOTOR_FEEDBACK_MAX_FRAME_SIZE) {
        memset((void *)&g_motor_feedback.last_frame[copy_length], 0,
               MOTOR_FEEDBACK_MAX_FRAME_SIZE - copy_length);
    }

    g_motor_feedback.last_frame_length = copy_length;
    g_motor_feedback.last_frame_address = s_frame_buffer[0];
    g_motor_feedback.last_frame_function = (length >= 2U) ? s_frame_buffer[1] : 0U;
    g_motor_feedback.last_frame_status = (length >= 3U) ? s_frame_buffer[2] : 0U;
    g_motor_feedback.last_frame_tick = HAL_GetTick();
    g_motor_feedback.frame_count++;

    if (length == 4U) {
        g_motor_feedback.ack_frame_count++;
        g_motor_feedback.last_ack_status = s_frame_buffer[2];
        g_motor_feedback.last_ack_success =
            (uint8_t)(s_frame_buffer[2] == MOTOR_FEEDBACK_ACK_SUCCESS_STATUS);
        if (g_motor_feedback.last_ack_success != 0U) {
            g_motor_feedback.ack_success_count++;
        } else {
            g_motor_feedback.ack_error_count++;
        }
    }

    /* 商家“读取电机实时位置”例程明确规定 0x36 回包固定为 8 字节。 */
    if ((length == 8U) && (s_frame_buffer[1] == 0x36U)) {
        raw_position = ((uint32_t)s_frame_buffer[3] << 24) |
                       ((uint32_t)s_frame_buffer[4] << 16) |
                       ((uint32_t)s_frame_buffer[5] << 8)  |
                       ((uint32_t)s_frame_buffer[6]);

        g_motor_feedback.position_direction = s_frame_buffer[2];
        g_motor_feedback.position_raw = raw_position;
        g_motor_feedback.position_signed_units =
            (s_frame_buffer[2] != 0U) ? -(int64_t)raw_position : (int64_t)raw_position;
        g_motor_feedback.position_degrees =
            (float)g_motor_feedback.position_signed_units * 360.0f / 65536.0f;
        g_motor_feedback.position_valid = 1U;
        g_motor_feedback.position_frame_count++;
    }

    if ((g_motor_feedback.expected_active != 0U) &&
        (length >= 3U) &&
        (s_frame_buffer[0] == g_motor_feedback.expected_address) &&
        (s_frame_buffer[1] == g_motor_feedback.expected_function) &&
        ((g_motor_feedback.expected_length == 0U) ||
         (length == g_motor_feedback.expected_length))) {
        g_motor_feedback.expected_status_ok =
            (length == 4U) ? g_motor_feedback.last_ack_success : 1U;
        g_motor_feedback.expected_received = 1U;
        g_motor_feedback.expected_active = 0U;
        g_motor_feedback.matched_response_count++;
    } else {
        g_motor_feedback.unexpected_frame_count++;
    }

    Motor_FeedbackResetAssembler();
}

void Motor_FeedbackInit(void)
{
    memset((void *)&g_motor_feedback, 0, sizeof(g_motor_feedback));
    memset(s_frame_buffer, 0, sizeof(s_frame_buffer));
    Motor_FeedbackResetAssembler();
}

void Motor_FeedbackBeginExpectedResponse(uint8_t address, uint8_t function)
{
    g_motor_feedback.expected_address = address;
    g_motor_feedback.expected_function = function;
    g_motor_feedback.expected_length = Motor_FeedbackLengthForFunction(function);
    g_motor_feedback.expected_received = 0U;
    g_motor_feedback.expected_status_ok = 0U;
    g_motor_feedback.expected_active = 1U;
    g_motor_feedback.request_count++;
    g_motor_feedback.partial_length = 0U;
    memset((void *)g_motor_feedback.partial_data, 0,
           sizeof(g_motor_feedback.partial_data));

    /* 丢弃登记期望前可能遗留的半帧，避免旧数据与新应答拼接。 */
    Motor_FeedbackResetAssembler();
}

MotorFeedbackPollResult_t Motor_FeedbackPollExpectedResponse(void)
{
    if (g_motor_feedback.expected_received == 0U) {
        return MOTOR_FEEDBACK_POLL_WAITING;
    }

    return (g_motor_feedback.expected_status_ok != 0U) ?
           MOTOR_FEEDBACK_POLL_SUCCESS : MOTOR_FEEDBACK_POLL_REJECTED;
}

void Motor_FeedbackCancelExpectedResponse(uint8_t count_timeout)
{
    Motor_FeedbackSavePartial();
    Motor_FeedbackResetAssembler();
    g_motor_feedback.expected_active = 0U;
    g_motor_feedback.expected_received = 0U;
    g_motor_feedback.expected_status_ok = 0U;
    if (count_timeout != 0U) {
        g_motor_feedback.timeout_count++;
    }
}

bool Motor_FeedbackWaitExpectedResponse(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while (g_motor_feedback.expected_received == 0U) {
        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            Motor_FeedbackCancelExpectedResponse(1U);
            return false;
        }
    }

    return true;
}

bool Motor_FeedbackExpectedResponseSucceeded(void)
{
    return g_motor_feedback.expected_status_ok != 0U;
}

void Motor_FeedbackOnRxData(const uint8_t *data, uint16_t size)
{
    uint16_t i;
    uint8_t byte;

    if ((data == NULL) || (size == 0U)) {
        return;
    }

    g_motor_feedback.rx_callback_count++;
    g_motor_feedback.rx_byte_count += size;
    g_motor_feedback.last_rx_tick = HAL_GetTick();

    for (i = 0U; i < size; i++) {
        byte = data[i];

        /*
         * 有命令正在等待时先严格寻找“期望地址 + 期望功能码”帧头，避免上电杂波
         * 或迟到旧帧与本次应答拼接。位置数据是任意二进制，不能见到 0x6B 就截断。
         */
        if (g_motor_feedback.expected_active != 0U) {
            if (s_frame_length == 0U) {
                if (byte != g_motor_feedback.expected_address) {
                    g_motor_feedback.discarded_byte_count++;
                    continue;
                }
                s_frame_buffer[0] = byte;
                s_frame_length = 1U;
                s_frame_target_length = g_motor_feedback.expected_length;
                continue;
            }

            if (s_frame_length == 1U) {
                if (byte != g_motor_feedback.expected_function) {
                    g_motor_feedback.discarded_byte_count++;
                    if (byte == g_motor_feedback.expected_address) {
                        s_frame_buffer[0] = byte;
                        s_frame_length = 1U;
                    } else {
                        Motor_FeedbackResetAssembler();
                    }
                    continue;
                }
                s_frame_buffer[s_frame_length++] = byte;
                continue;
            }
        }

        if (s_frame_length >= MOTOR_FEEDBACK_MAX_FRAME_SIZE) {
            Motor_FeedbackResetAssembler();
            g_motor_feedback.overflow_count++;
        }

        s_frame_buffer[s_frame_length++] = byte;

        if (s_frame_target_length == 0U) {
            if (s_frame_length == 2U) {
                s_frame_target_length = Motor_FeedbackLengthForFunction(s_frame_buffer[1]);
            }

            /* 未登记固定长度的其他商家命令，才退回使用 0x6B 帧尾。 */
            if ((s_frame_target_length == 0U) && (byte == 0x6BU)) {
                Motor_FeedbackFinishFrame();
            }
        }

        if ((s_frame_target_length != 0U) &&
            (s_frame_length == s_frame_target_length)) {
            if (byte == 0x6BU) {
                Motor_FeedbackFinishFrame();
            } else {
                Motor_FeedbackSavePartial();
                Motor_FeedbackResetAssembler();
                g_motor_feedback.invalid_frame_count++;
            }
        }
    }
}
