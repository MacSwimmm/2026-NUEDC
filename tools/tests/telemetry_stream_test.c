#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Board.h"
#include "Car_Control.h"
#include "Race_Control.h"
#include "Serial.h"
#include "Telemetry_Stream.h"

/*
 * 该主机测试直接编译生产版 Telemetry_Stream.c，仅用桩替代时基、快照和串口。
 * 因此帧长、字段偏移、事件边沿、CRC 和整帧丢弃策略都由真实固件编码代码产生。
 */
static uint32_t g_now_ms;
static CarControlWatch_t g_car;
static RaceTelemetry_t g_race;
static uint8_t g_last_frame[TELEMETRY_FRAME_LENGTH];
static uint16_t g_last_length;
static uint16_t g_pending_bytes;
static uint32_t g_write_calls;
static uint32_t g_serial_init_calls;
static bool g_accept_write = true;

uint32_t Board_Millis(void)
{
    return g_now_ms;
}

void Car_ControlGetWatchSnapshot(CarControlWatch_t *watch)
{
    *watch = g_car;
}

void Race_ControlGetTelemetry(RaceTelemetry_t *telemetry)
{
    *telemetry = g_race;
}

void Serial_TelemetryInit(void)
{
    g_serial_init_calls++;
    g_pending_bytes = 0U;
}

bool Serial_TelemetryWrite(const uint8_t *data, uint16_t length)
{
    g_write_calls++;
    g_last_length = length;
    if (!g_accept_write) return false;
    assert(length == TELEMETRY_FRAME_LENGTH);
    memcpy(g_last_frame, data, length);
    g_pending_bytes = length;
    return true;
}

uint16_t Serial_TelemetryPendingBytes(void)
{
    return g_pending_bytes;
}

static uint16_t ReadU16(const uint8_t *data)
{
    return (uint16_t)((uint16_t)data[0] |
                      ((uint16_t)data[1] << 8));
}

static int16_t ReadI16(const uint8_t *data)
{
    return (int16_t)ReadU16(data);
}

static uint32_t ReadU32(const uint8_t *data)
{
    return (uint32_t)ReadU16(data) |
           ((uint32_t)ReadU16(data + 2) << 16);
}

static uint16_t IndependentCrc16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t i;

    for (i = 0U; i < length; i++) {
        uint8_t bit;

        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

static void SetKnownSnapshot(void)
{
    memset(&g_car, 0, sizeof(g_car));
    memset(&g_race, 0, sizeof(g_race));

    g_car.raw_track_mask = 0xE7U;
    g_car.track_mask = 0x18U;
    g_car.track_ready = 1U;
    g_car.active_count = 2U;
    g_car.armed = 1U;
    g_car.recovering = 1U;
    g_car.line_lost_samples = 4U;
    g_car.center_confirm_samples = 6U;
    g_car.line_lost_ms = 20U;
    g_car.raw_track_error = -12.34f;
    g_car.steering_error_used = -11.50f;
    g_car.filtered_error = -10.25f;
    g_car.filtered_derivative = 1.75f;
    g_car.p_term_mps = -0.023f;
    g_car.d_term_mps = 0.004f;
    g_car.steering_correction_mps = -0.019f;
    g_car.requested_base_speed_mps = 0.280f;
    g_car.desired_base_speed_mps = 0.240f;
    g_car.dynamic_base_speed_mps = 0.235f;
    g_car.left_target_mps = 0.254f;
    g_car.right_target_mps = 0.216f;
    g_car.left_speed_mps = 0.249f;
    g_car.right_speed_mps = 0.211f;
    g_car.left_encoder_count = 123;
    g_car.right_encoder_count = 118;
    g_car.left_pwm = 800;
    g_car.right_pwm = 760;
    g_car.control_tick_count = 200U;

    g_race.state = RACE_STATE_WAITING;
    g_race.result = RACE_RESULT_NONE;
    g_race.marker_cleared = 1U;
    g_race.elapsed_ms = 3456U;
}

static void VerifyBaseFrame(void)
{
    uint16_t expected_crc;

    assert(g_last_length == TELEMETRY_FRAME_LENGTH);
    assert(g_last_frame[0] == TELEMETRY_FRAME_SYNC_0);
    assert(g_last_frame[1] == TELEMETRY_FRAME_SYNC_1);
    assert(g_last_frame[2] == TELEMETRY_PROTOCOL_VERSION);
    assert(g_last_frame[3] == TELEMETRY_FRAME_TYPE_CONTROL);
    assert(ReadU16(&g_last_frame[4]) == TELEMETRY_PAYLOAD_LENGTH);
    assert(ReadU16(&g_last_frame[6]) == 0U);
    assert(ReadU32(&g_last_frame[8]) == 10U);
    assert(ReadU32(&g_last_frame[12]) == 200U);
    assert(ReadU32(&g_last_frame[16]) == 3456U);
    assert(g_last_frame[20] == RACE_STATE_WAITING);
    assert(g_last_frame[22] == TELEMETRY_EVENT_NONE);
    assert(g_last_frame[23] == 0xE7U);
    assert(g_last_frame[24] == 0x18U);
    assert(g_last_frame[25] == 2U);
    assert(g_last_frame[28] == 6U);
    assert(g_last_frame[29] == 4U);
    assert(ReadU16(&g_last_frame[30]) == 0x0053U);
    assert(ReadU16(&g_last_frame[32]) == 20U);
    assert(ReadU16(&g_last_frame[34]) == 0U);
    assert(ReadI16(&g_last_frame[36]) == -1234);
    assert(ReadI16(&g_last_frame[40]) == -1025);
    assert(ReadI16(&g_last_frame[42]) == 175);
    assert(ReadI16(&g_last_frame[44]) == -23);
    assert(ReadI16(&g_last_frame[48]) == -19);
    assert(ReadI16(&g_last_frame[50]) == 280);
    assert(ReadI16(&g_last_frame[56]) == 254);
    assert(ReadI16(&g_last_frame[60]) == 249);
    assert(ReadI16(&g_last_frame[64]) == 123);
    assert(ReadI16(&g_last_frame[68]) == 800);

    expected_crc = IndependentCrc16(&g_last_frame[2], 70U);
    assert(ReadU16(&g_last_frame[72]) == expected_crc);
}

static void StepAt(uint32_t now_ms, RaceState_t state,
                   RaceResult_t result, uint8_t expected_event,
                   uint16_t expected_sequence)
{
    g_now_ms = now_ms;
    g_race.state = state;
    g_race.result = result;
    Telemetry_StreamProcess();
    assert(ReadU16(&g_last_frame[6]) == expected_sequence);
    assert(g_last_frame[22] == expected_event);
}

int main(void)
{
    SetKnownSnapshot();
    Telemetry_StreamInit();
    assert(g_serial_init_calls == 1U);

    g_now_ms = 9U;
    Telemetry_StreamProcess();
    assert(g_write_calls == 0U);

    g_now_ms = 10U;
    Telemetry_StreamProcess();
    assert(g_write_calls == 1U);
    VerifyBaseFrame();

    StepAt(20U, RACE_STATE_LEAVING_A, RACE_RESULT_NONE,
           TELEMETRY_EVENT_RACE_STARTED, 1U);
    StepAt(30U, RACE_STATE_RUNNING, RACE_RESULT_NONE,
           TELEMETRY_EVENT_NONE, 2U);
    g_race.elapsed_ms = 12345U;
    StepAt(40U, RACE_STATE_FINISHED, RACE_RESULT_SUCCESS,
           TELEMETRY_EVENT_RACE_FINISHED, 3U);
    StepAt(50U, RACE_STATE_LEAVING_A, RACE_RESULT_NONE,
           TELEMETRY_EVENT_RACE_STARTED, 4U);
    StepAt(60U, RACE_STATE_WAITING, RACE_RESULT_NONE,
           TELEMETRY_EVENT_MANUAL_STOP, 5U);
    /* PB1 直接进入 RUNNING，也必须产生开始事件供电脑端自动记录。 */
    StepAt(70U, RACE_STATE_RUNNING, RACE_RESULT_NONE,
           TELEMETRY_EVENT_RACE_STARTED, 6U);
    StepAt(80U, RACE_STATE_WAITING, RACE_RESULT_NONE,
           TELEMETRY_EVENT_MANUAL_STOP, 7U);

    g_accept_write = false;
    g_now_ms = 90U;
    Telemetry_StreamProcess();
    assert(g_telemetry_watch.frames_dropped == 1U);

    g_accept_write = true;
    g_now_ms = 100U;
    Telemetry_StreamProcess();
    assert(ReadU16(&g_last_frame[6]) == 9U);
    assert(ReadU16(&g_last_frame[34]) == 1U);
    assert(g_telemetry_watch.frames_queued == 9U);
    assert(g_telemetry_watch.last_frame_length == TELEMETRY_FRAME_LENGTH);

    puts("[OK] UART3 telemetry frame, CRC, events and drop handling passed.");
    return 0;
}
