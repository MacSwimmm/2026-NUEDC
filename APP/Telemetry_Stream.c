#include "Telemetry_Stream.h"

#include <stdbool.h>
#include <stdint.h>

#include "Board.h"
#include "Car_Control.h"
#include "Race_Control.h"
#include "Serial.h"
#include "app_config.h"

/* 负载 flags 位定义；电脑端使用相同位序展开为 CSV 布尔列。 */
#define TELEMETRY_FLAG_TRACK_READY       (1U << 0)
#define TELEMETRY_FLAG_ARMED             (1U << 1)
#define TELEMETRY_FLAG_MANUAL_SPEED      (1U << 2)
#define TELEMETRY_FLAG_MARKER            (1U << 3)
#define TELEMETRY_FLAG_RECOVERING        (1U << 4)
#define TELEMETRY_FLAG_LINE_FAULT        (1U << 5)
#define TELEMETRY_FLAG_MARKER_CLEARED    (1U << 6)
#define TELEMETRY_FLAG_APPROACH_ACTIVE   (1U << 7)

static uint16_t g_next_sequence = 0U;
static uint32_t g_last_frame_ms = 0U;
static uint32_t g_frames_queued = 0U;
static uint32_t g_frames_dropped = 0U;
static RaceState_t g_previous_race_state = RACE_STATE_WAITING;

/* 只供 Keil Watch 观察，真实发送状态保存在上面的 static 变量中。 */
volatile TelemetryStreamWatch_t g_telemetry_watch;

/** @brief 以小端顺序追加一个无符号 8 位量。 */
static void PutU8(uint8_t *buffer, uint16_t *index, uint8_t value)
{
    buffer[*index] = value;
    (*index)++;
}

/** @brief 以小端顺序追加一个无符号 16 位量。 */
static void PutU16(uint8_t *buffer, uint16_t *index, uint16_t value)
{
    PutU8(buffer, index, (uint8_t)(value & 0xFFU));
    PutU8(buffer, index, (uint8_t)((value >> 8) & 0xFFU));
}

/** @brief 有符号 16 位量保持补码位型后按小端顺序发送。 */
static void PutI16(uint8_t *buffer, uint16_t *index, int16_t value)
{
    PutU16(buffer, index, (uint16_t)value);
}

/** @brief 以小端顺序追加一个无符号 32 位量。 */
static void PutU32(uint8_t *buffer, uint16_t *index, uint32_t value)
{
    PutU16(buffer, index, (uint16_t)(value & 0xFFFFU));
    PutU16(buffer, index, (uint16_t)((value >> 16) & 0xFFFFU));
}

/** @brief 将 32 位诊断量饱和到协议使用的 int16_t，避免窄化回绕。 */
static int16_t ClampInt16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

/** @brief 浮点量按指定倍率四舍五入并压缩成 int16_t。 */
static int16_t ScaleFloatToInt16(float value, float scale)
{
    float scaled = value * scale;
    int32_t rounded;

    if (scaled > 32767.0f) return 32767;
    if (scaled < -32768.0f) return -32768;
    rounded = (int32_t)((scaled >= 0.0f) ?
                        (scaled + 0.5f) : (scaled - 0.5f));
    return (int16_t)rounded;
}

/** @brief CRC16-CCITT-FALSE：多项式 0x1021，初值 0xFFFF，不反射。 */
static uint16_t CalculateCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    for (i = 0U; i < length; i++) {
        uint8_t bit;

        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x8000U) != 0U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/** @brief 把分散的状态字节压成协议 flags，减少无线带宽。 */
static uint16_t BuildFlags(const CarControlWatch_t *car,
                           const RaceTelemetry_t *race)
{
    uint16_t flags = 0U;

    if (car->track_ready != 0U) flags |= TELEMETRY_FLAG_TRACK_READY;
    if (car->armed != 0U) flags |= TELEMETRY_FLAG_ARMED;
    if (car->manual_speed_mode != 0U) flags |= TELEMETRY_FLAG_MANUAL_SPEED;
    if (car->marker != 0U) flags |= TELEMETRY_FLAG_MARKER;
    if (car->recovering != 0U) flags |= TELEMETRY_FLAG_RECOVERING;
    if (car->line_fault != 0U) flags |= TELEMETRY_FLAG_LINE_FAULT;
    if (race->marker_cleared != 0U) flags |= TELEMETRY_FLAG_MARKER_CLEARED;
    if (race->approach_active != 0U) flags |= TELEMETRY_FLAG_APPROACH_ACTIVE;
    return flags;
}

/**
 * @brief 根据相邻遥测样本的比赛状态生成电脑端分段事件。
 *
 * Race_Control 的人工中止会立即回到 WAITING 并清零计时；本模块在清零后的
 * 第一帧额外发送 MANUAL_STOP，电脑端便可保留此前整圈数据并停止自动记录。
 */
static TelemetryEvent_t DetectEvent(RaceState_t current_state)
{
    TelemetryEvent_t event = TELEMETRY_EVENT_NONE;
    bool previous_active =
        (g_previous_race_state == RACE_STATE_LEAVING_A) ||
        (g_previous_race_state == RACE_STATE_RUNNING);

    if (((current_state == RACE_STATE_LEAVING_A) ||
         (current_state == RACE_STATE_RUNNING)) && !previous_active) {
        event = TELEMETRY_EVENT_RACE_STARTED;
    } else if ((current_state == RACE_STATE_FINISHED) && previous_active) {
        event = TELEMETRY_EVENT_RACE_FINISHED;
    } else if ((current_state == RACE_STATE_WAITING) && previous_active) {
        event = TELEMETRY_EVENT_MANUAL_STOP;
    }

    g_previous_race_state = current_state;
    return event;
}

/**
 * @brief 构造固定 74 字节控制遥测帧。
 *
 * 64 字节负载依次为：3 个 uint32 时间/周期量、10 个 uint8 状态量、
 * flags/丢线时间/板端丢帧共 3 个 uint16，以及 18 个 int16 诊断量。
 * 误差按 0.01 单位发送，速度按 mm/s 发送，编码器与 PWM 保持原始计数。
 */
static void BuildFrame(uint8_t *frame, uint16_t sequence, uint32_t now_ms,
                       const CarControlWatch_t *car,
                       const RaceTelemetry_t *race,
                       TelemetryEvent_t event)
{
    uint16_t index = 0U;
    uint16_t crc;

    PutU8(frame, &index, TELEMETRY_FRAME_SYNC_0);
    PutU8(frame, &index, TELEMETRY_FRAME_SYNC_1);
    PutU8(frame, &index, TELEMETRY_PROTOCOL_VERSION);
    PutU8(frame, &index, TELEMETRY_FRAME_TYPE_CONTROL);
    PutU16(frame, &index, TELEMETRY_PAYLOAD_LENGTH);
    PutU16(frame, &index, sequence);

    PutU32(frame, &index, now_ms);
    PutU32(frame, &index, car->control_tick_count);
    PutU32(frame, &index, race->elapsed_ms);

    PutU8(frame, &index, (uint8_t)race->state);
    PutU8(frame, &index, (uint8_t)race->result);
    PutU8(frame, &index, (uint8_t)event);
    PutU8(frame, &index, car->raw_track_mask);
    PutU8(frame, &index, car->track_mask);
    PutU8(frame, &index, car->active_count);
    PutU8(frame, &index, race->marker_clear_streak);
    PutU8(frame, &index, race->marker_confirm_streak);
    PutU8(frame, &index, car->center_confirm_samples);
    PutU8(frame, &index, car->line_lost_samples);

    PutU16(frame, &index, BuildFlags(car, race));
    PutU16(frame, &index, car->line_lost_ms);
    PutU16(frame, &index, (uint16_t)g_frames_dropped);

    PutI16(frame, &index, ScaleFloatToInt16(car->raw_track_error, 100.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->steering_error_used, 100.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->filtered_error, 100.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->filtered_derivative, 100.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->p_term_mps, 1000.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->d_term_mps, 1000.0f));
    PutI16(frame, &index,
           ScaleFloatToInt16(car->steering_correction_mps, 1000.0f));
    PutI16(frame, &index,
           ScaleFloatToInt16(car->requested_base_speed_mps, 1000.0f));
    PutI16(frame, &index,
           ScaleFloatToInt16(car->desired_base_speed_mps, 1000.0f));
    PutI16(frame, &index,
           ScaleFloatToInt16(car->dynamic_base_speed_mps, 1000.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->left_target_mps, 1000.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->right_target_mps, 1000.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->left_speed_mps, 1000.0f));
    PutI16(frame, &index, ScaleFloatToInt16(car->right_speed_mps, 1000.0f));
    PutI16(frame, &index, ClampInt16(car->left_encoder_count));
    PutI16(frame, &index, ClampInt16(car->right_encoder_count));
    PutI16(frame, &index, ClampInt16(car->left_pwm));
    PutI16(frame, &index, ClampInt16(car->right_pwm));

    /* CRC 不包含两个同步字节，覆盖版本、类型、长度、序号和完整负载。 */
    crc = CalculateCrc16(&frame[2],
                         (uint16_t)(TELEMETRY_FRAME_LENGTH - 4U));
    PutU16(frame, &index, crc);
    (void)index; /* 固定布局由主机测试同时验证 index == 74。 */
}

void Telemetry_StreamInit(void)
{
    RaceTelemetry_t race;

    Serial_TelemetryInit();
    Race_ControlGetTelemetry(&race);
    g_next_sequence = 0U;
    g_last_frame_ms = Board_Millis();
    g_frames_queued = 0U;
    g_frames_dropped = 0U;
    g_previous_race_state = race.state;

    g_telemetry_watch.last_sequence = 0U;
    g_telemetry_watch.last_frame_length = 0U;
    g_telemetry_watch.queued_bytes = 0U;
    g_telemetry_watch.last_event = TELEMETRY_EVENT_NONE;
    g_telemetry_watch.frames_queued = 0U;
    g_telemetry_watch.frames_dropped = 0U;
    g_telemetry_watch.last_frame_ms = g_last_frame_ms;
}

void Telemetry_StreamProcess(void)
{
    uint32_t now_ms = Board_Millis();
    CarControlWatch_t car;
    RaceTelemetry_t race;
    TelemetryEvent_t event;
    uint8_t frame[TELEMETRY_FRAME_LENGTH];
    uint16_t sequence;
    bool queued;

    if ((now_ms - g_last_frame_ms) < TELEMETRY_STREAM_PERIOD_MS) return;

    /*
     * 主循环若因 OLED 刷新晚到，不补发历史帧；直接从最新完整控制快照继续，
     * 避免瞬间突发多帧反过来挤占 UART3 环形缓冲。
     */
    g_last_frame_ms = now_ms;
    Car_ControlGetWatchSnapshot(&car);
    Race_ControlGetTelemetry(&race);
    event = DetectEvent(race.state);
    sequence = g_next_sequence++;
    BuildFrame(frame, sequence, now_ms, &car, &race, event);
    queued = Serial_TelemetryWrite(frame, TELEMETRY_FRAME_LENGTH);
    if (queued) {
        g_frames_queued++;
    } else {
        /* 整帧未入队；序号仍已递增，电脑端下一帧可直接看到缺口。 */
        g_frames_dropped++;
    }

    g_telemetry_watch.last_sequence = sequence;
    g_telemetry_watch.last_frame_length = TELEMETRY_FRAME_LENGTH;
    g_telemetry_watch.queued_bytes = Serial_TelemetryPendingBytes();
    g_telemetry_watch.last_event = (uint8_t)event;
    g_telemetry_watch.frames_queued = g_frames_queued;
    g_telemetry_watch.frames_dropped = g_frames_dropped;
    g_telemetry_watch.last_frame_ms = now_ms;
}
