#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Car_Control.h"
#include "LineTrack.h"
#include "Race_Control.h"
#include "TrackSensor.h"
#include "app_config.h"
#include "bsp_config.h"
#include "ti_msp_dl_config.h"

/*
 * 本测试在电脑上直接编译生产版 LineTrack.c 和 Race_Control.c。
 * Race_Control 所依赖的 Board/Car 接口由下列最小桩函数提供，不复制其状态机公式。
 */
static CarTelemetry_t g_stub_car;
static uint32_t g_stub_millis;
static bool g_stub_arm_allowed;
static bool g_stub_armed;
static float g_stub_base_speed;
static unsigned int g_failed_checks;

/* TrackSensor.c 通过主机桩读取该字节，bit7~bit0 仍对应 CH1~CH8。 */
GPIO_Regs g_test_track_port_a;
GPIO_Regs g_test_track_port_b;
uint8_t g_test_track_raw_mask;

static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

static void Check(bool condition, const char *message)
{
    if (!condition) {
        printf("[FAIL] %s\n", message);
        g_failed_checks++;
    }
}

/* Race_Control.c 所需的 1 ms 时基桩。 */
uint32_t Board_Millis(void)
{
    return g_stub_millis;
}

/* 模拟 Car_Control 的使能门控；测试可切换 arm_allowed 验证启动拒绝。 */
bool Car_ControlSetArmed(bool armed)
{
    if (armed && !g_stub_arm_allowed) {
        g_stub_armed = false;
        return false;
    }

    g_stub_armed = armed;
    if (!armed) g_stub_car.line_fault = 0U;
    return true;
}

void Car_ControlSetBaseSpeed(float base_speed_mps)
{
    g_stub_base_speed = base_speed_mps;
}

void Car_ControlGetTelemetry(CarTelemetry_t *telemetry)
{
    *telemetry = g_stub_car;
}

/** @brief 生成一个新的 5 ms 控制样本并推进比赛主循环。 */
static void PushRaceSample(uint8_t marker, uint8_t strong_marker,
                           uint8_t line_fault)
{
    g_stub_millis += APP_CONTROL_PERIOD_MS;
    g_stub_car.control_tick_count++;
    g_stub_car.marker = marker;
    g_stub_car.junction = strong_marker;
    g_stub_car.line_fault = line_fault;
    Race_ControlProcess();
}

/** @brief 重置桩环境并进入 Race_Control 的 WAITING 状态。 */
static void ResetRaceFixture(void)
{
    g_stub_car = (CarTelemetry_t){0};
    g_stub_millis = 100U;
    g_stub_arm_allowed = true;
    g_stub_armed = false;
    g_stub_base_speed = 0.0f;
    Race_ControlInit();
}

static void TestRaceStartAndFinish(void)
{
    RaceTelemetry_t race;
    unsigned int i;

    ResetRaceFixture();
    g_stub_car.marker = 1U;              /* 探头从 A 点横线上启动。 */
    g_stub_car.junction = 1U;
    g_stub_car.control_tick_count = 10U;
    Race_ControlHandleUserKey();
    Race_ControlGetTelemetry(&race);

    Check(race.state == RACE_STATE_LEAVING_A,
          "PB21 should enter LEAVING_A");
    Check(g_race_watch.state == RACE_STATE_LEAVING_A,
          "race Watch mirror should follow start state");
    Check(g_stub_armed, "car should be armed after a valid start");
    Check(AbsFloat(g_stub_base_speed - RACE_CRUISE_SPEED_MPS) < 0.0001f,
          "start should request cruise speed");

    /* 同一个 control_tick 重复调用不能把一个样本累计成多次。 */
    for (i = 0U; i < 5U; i++) Race_ControlProcess();
    Race_ControlGetTelemetry(&race);
    Check(race.marker_clear_streak == 0U,
          "same control sample must not advance marker clear streak");

    PushRaceSample(1U, 1U, 0U);
    for (i = 0U; i < RACE_MARKER_CLEAR_SAMPLES; i++) {
        /* 强横线已离开；即使还有三路黑，也应允许进入正常跑圈。 */
        PushRaceSample(1U, 0U, 0U);
    }
    Race_ControlGetTelemetry(&race);
    Check(race.state == RACE_STATE_RUNNING,
          "three clear samples should open finish detection");
    Check(race.marker_cleared != 0U,
          "marker_cleared should latch after leaving A");

    /* 最低时间门限之前即使再次看到横线，也不能完成。 */
    for (i = 0U; i < RACE_MARKER_STRONG_CONFIRM_SAMPLES; i++) {
        PushRaceSample(1U, 0U, 0U);
    }
    Race_ControlGetTelemetry(&race);
    Check(race.state == RACE_STATE_RUNNING,
          "early marker must not finish the lap");
    Check(race.marker_confirm_streak == 0U,
          "early marker streak should be cleared");

    /* 越过最低时间门限后，三路弱标志仍不得报告正常完成。 */
    g_stub_millis = 100U + RACE_FINISH_MIN_TIME_MS;
    for (i = 0U; i < (RACE_MARKER_STRONG_CONFIRM_SAMPLES + 2U); i++) {
        PushRaceSample(1U, 0U, 0U);
    }
    Race_ControlGetTelemetry(&race);
    Check(race.state == RACE_STATE_RUNNING,
          "three-channel weak marker must not finish the lap");
    Check(race.marker_confirm_streak == 0U,
          "weak marker must not accumulate the strong finish streak");

    /* 只有连续至少四路黑的强横线持续到门限才允许完成。 */
    for (i = 0U; i < RACE_MARKER_STRONG_CONFIRM_SAMPLES; i++) {
        PushRaceSample(1U, 1U, 0U);
    }
    Race_ControlGetTelemetry(&race);
    Check(race.state == RACE_STATE_FINISHED,
          "confirmed strong second A marker should finish");
    Check(race.result == RACE_RESULT_SUCCESS,
          "normal finish should report success");
    Check((g_race_watch.result == RACE_RESULT_SUCCESS) &&
              (g_race_watch.final_time_ms == race.final_time_ms),
          "race Watch mirror should freeze successful result and time");
    Check(race.final_time_ms == race.elapsed_ms,
          "finished time should remain frozen");
    Check(!g_stub_armed, "normal finish should disarm the chassis");
}

/** @brief 验证连续相邻 4 路以上的强 A 点标志按配置的 30 ms 确认。 */
static void TestStrongMarkerFinish(void)
{
    RaceTelemetry_t race;
    unsigned int i;

    ResetRaceFixture();
    g_stub_car.marker = 1U;
    g_stub_car.junction = 1U;
    Race_ControlHandleUserKey();
    for (i = 0U; i < RACE_MARKER_CLEAR_SAMPLES; i++) {
        PushRaceSample(0U, 0U, 0U);
    }

    g_stub_millis = 100U + RACE_FINISH_MIN_TIME_MS;
    for (i = 0U; i < RACE_MARKER_STRONG_CONFIRM_SAMPLES; i++) {
        PushRaceSample(1U, 1U, 0U);
    }
    Race_ControlGetTelemetry(&race);
    Check((race.state == RACE_STATE_FINISHED) &&
              (race.result == RACE_RESULT_SUCCESS),
          "configured strong A samples should finish the lap");
}

static void TestRaceApproachFaultAndReject(void)
{
    RaceTelemetry_t race;
    unsigned int i;

    ResetRaceFixture();
    g_stub_car.marker = 1U;
    g_stub_car.junction = 1U;
    Race_ControlHandleUserKey();
    for (i = 0U; i < RACE_MARKER_CLEAR_SAMPLES; i++) {
        PushRaceSample(0U, 0U, 0U);
    }

    g_stub_millis = 100U + RACE_APPROACH_START_MS;
    Race_ControlProcess();
    Race_ControlGetTelemetry(&race);
    Check(race.approach_active != 0U,
          "configured approach time should enable approach speed");
    Check(AbsFloat(g_stub_base_speed - RACE_APPROACH_SPEED_MPS) < 0.0001f,
          "approach phase should request low speed");

    PushRaceSample(0U, 0U, 1U);
    Race_ControlGetTelemetry(&race);
    Check(race.state == RACE_STATE_FINISHED,
          "line fault should enter terminal stopped state");
    Check(race.result == RACE_RESULT_LINE_LOST,
          "line fault must not be reported as successful A arrival");
    Check(g_race_watch.result == RACE_RESULT_LINE_LOST,
          "race Watch mirror should expose line-loss result");
    Check(!g_stub_armed, "line fault should disarm the chassis");

    ResetRaceFixture();
    g_stub_arm_allowed = false;
    Race_ControlHandleUserKey();
    Race_ControlGetTelemetry(&race);
    Check(race.state == RACE_STATE_WAITING,
          "rejected start should stay in WAITING");
    Check(race.result == RACE_RESULT_START_REJECTED,
          "rejected start should expose its reason");
    Check(race.start_reject_count == 1U,
          "start reject counter should increment once");
}

static void InitLineController(LineTrack_t *controller)
{
    LineTrack_Init(controller,
                   TRACK_BASE_SPEED_MPS,
                   TRACK_BASE_SPEED_MIN_MPS,
                   TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR,
                   TRACK_STEER_KP_MPS,
                   TRACK_STEER_ENTRY_KP_MPS,
                   TRACK_STEER_CURVE_KP_MPS,
                   TRACK_STEER_ENTRY_ERROR_ABS,
                   TRACK_STEER_CURVE_ERROR_ABS,
                   TRACK_STEER_KD_MPS,
                   TRACK_CORRECTION_LIMIT_MPS,
                   TRACK_TARGET_LIMIT_MPS,
                   TRACK_RECOVERY_KICK_SPEED_MPS,
                   TRACK_RECOVERY_SPEED_MPS,
                   TRACK_RECOVERY_KICK_MS,
                   TRACK_LOST_PROTECT_MS,
                   TRACK_DIRECTION_DEADBAND);
}

/** @brief 验证只放大正常行驶速度，P/D 和恢复参数保持稳定版原值。 */
static void TestSourceTrackingFormula(void)
{
    LineTrack_t controller;
    float left;
    float right;
    float expected_base;
    float expected_correction;

    Check(AbsFloat(TRACK_FORWARD_SPEED_SCALE - 5.0f) < 0.0001f,
          "forward tracking speed scale should be exactly five");
    Check((AbsFloat(TRACK_STEER_KP_MPS -
                    0.035f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (AbsFloat(TRACK_STEER_ENTRY_KP_MPS -
                        0.100f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (AbsFloat(TRACK_STEER_CURVE_KP_MPS -
                        0.150f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (AbsFloat(TRACK_STEER_ENTRY_ERROR_ABS - 100.0f) < 0.0001f) &&
              (AbsFloat(TRACK_STEER_CURVE_ERROR_ABS - 200.0f) < 0.0001f) &&
              (AbsFloat(TRACK_STEER_KD_MPS -
                        0.020f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (AbsFloat(TRACK_CORRECTION_LIMIT_MPS -
                        20.0f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (AbsFloat(TRACK_RECOVERY_KICK_SPEED_MPS -
                        24.0f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (AbsFloat(TRACK_RECOVERY_SPEED_MPS -
                        12.0f * TRACK_RPM_TO_MPS) < 0.0001f) &&
              (TRACK_RECOVERY_KICK_MS == 60U) &&
              (TRACK_LOST_PROTECT_MS == 400U),
          "scheduled P, fixed correction limit or reverse recovery parameters mismatch");
    Check((AbsFloat(SPEED_PI_KP - 150.0f) < 0.0001f) &&
              (AbsFloat(SPEED_PI_KI - 300.0f) < 0.0001f),
          "left and right wheel speed PI gains must stay unchanged");
    InitLineController(&controller);
    LineTrack_Calculate(&controller, 0.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check((AbsFloat(left - TRACK_BASE_SPEED_MPS) < 0.0001f) &&
              (AbsFloat(right - TRACK_BASE_SPEED_MPS) < 0.0001f),
          "zero error should command the scaled 100 RPM base speed");
    Check(AbsFloat(controller.derivative) < 0.0001f,
          "the first visible sample should have zero derivative");
    Check(AbsFloat(controller.active_kp - TRACK_STEER_KP_MPS) < 0.0001f,
          "small straight error should keep the stable straight Kp");

    LineTrack_Calculate(&controller, 100.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    expected_base = TRACK_BASE_SPEED_MPS -
                    100.0f * TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR;
    expected_correction = 100.0f * TRACK_STEER_ENTRY_KP_MPS +
                          100.0f * TRACK_STEER_KD_MPS;
    Check(AbsFloat(controller.base_speed_mps - expected_base) < 0.0001f,
          "absolute error should reduce base speed by source coefficient");
    Check(AbsFloat(controller.correction_mps - expected_correction) < 0.0001f,
          "entry P and 5 ms-equivalent D terms should add directly");
    Check(AbsFloat(controller.active_kp - TRACK_STEER_ENTRY_KP_MPS) <
              0.0001f,
          "entry error should select the scheduled entry Kp");
    Check((AbsFloat(left - (expected_base + expected_correction)) < 0.0001f) &&
              (AbsFloat(right - (expected_base - expected_correction)) <
               0.0001f) &&
              (left > right) && (controller.last_direction == 1),
          "positive error should speed the left wheel and turn right");

    LineTrack_Calculate(&controller, -350.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    expected_base = TRACK_BASE_SPEED_MPS -
                    350.0f * TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR;
    Check(AbsFloat(controller.base_speed_mps - expected_base) < 0.0001f,
          "outer sensor error should retain the original scaled speed formula");
    Check(AbsFloat(controller.correction_mps + TRACK_CORRECTION_LIMIT_MPS) <
              0.0001f,
          "large curve error should retain the original 18 RPM limit");
    Check(AbsFloat(controller.active_kp - TRACK_STEER_CURVE_KP_MPS) <
              0.0001f,
          "large curve error should select the scheduled curve Kp");
    Check((left < right) && (controller.last_direction == -1),
          "negative error should speed the right wheel and turn left");

    LineTrack_Reset(&controller);
    LineTrack_Calculate(&controller, -500.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(AbsFloat(controller.base_speed_mps - TRACK_BASE_SPEED_MIN_MPS) <
              0.0001f,
          "errors beyond the physical sensor range should retain the minimum clamp");

    LineTrack_Reset(&controller);
    LineTrack_Calculate(&controller, -1000.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    LineTrack_Calculate(&controller, 0.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(AbsFloat(left - (TRACK_BASE_SPEED_MPS +
                           TRACK_CORRECTION_LIMIT_MPS)) < 0.0001f,
          "unchanged correction should be added to the scaled base speed");
    Check(AbsFloat(TRACK_TARGET_LIMIT_MPS -
                   175.0f * TRACK_RPM_TO_MPS) < 0.0001f,
          "wheel target ceiling should follow the five-times speed scale");

    LineTrack_Reset(&controller);
    LineTrack_Calculate(&controller, 25.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.last_direction == 0,
          "direction should not update at the exact 25 deadband");
    LineTrack_Calculate(&controller, 26.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.last_direction == 1,
          "direction should update only after exceeding the deadband");

    LineTrack_Reset(&controller);
    LineTrack_SetBaseSpeed(&controller, TRACK_BASE_SPEED_MIN_MPS);
    LineTrack_Calculate(&controller, 0.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(AbsFloat(controller.base_speed_mps - TRACK_BASE_SPEED_MIN_MPS) <
              0.0001f,
          "compatibility speed request should only cap the source base speed");
}

static void TestLineRecoverySearch(void)
{
    LineTrack_t controller;
    float left;
    float right;
    unsigned int lost_samples = TRACK_LOST_PROTECT_MS /
                                APP_CONTROL_PERIOD_MS;
    unsigned int kick_samples = TRACK_RECOVERY_KICK_MS /
                                APP_CONTROL_PERIOD_MS;
    unsigned int i;

    InitLineController(&controller);
    LineTrack_Calculate(&controller, -100.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    LineTrack_Calculate(&controller, 0.0f, false, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.recovering && controller.running &&
              (controller.recovery_direction == 1) &&
              (AbsFloat(left - TRACK_RECOVERY_KICK_SPEED_MPS) < 0.0001f) &&
              (AbsFloat(right + TRACK_RECOVERY_KICK_SPEED_MPS) < 0.0001f),
          "lost line after a left turn should immediately kick right");

    for (i = 1U; i <= kick_samples; i++) {
        LineTrack_Calculate(&controller, 0.0f, false,
                            APP_CONTROL_PERIOD_MS, &left, &right);
    }
    Check((controller.recovery_direction == 1) &&
              (AbsFloat(left - TRACK_RECOVERY_SPEED_MPS) < 0.0001f) &&
              (AbsFloat(right + TRACK_RECOVERY_SPEED_MPS) < 0.0001f),
          "reverse kick should continue as slow opposite-direction search");

    LineTrack_Reset(&controller);
    LineTrack_Calculate(&controller, 100.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    LineTrack_Calculate(&controller, 0.0f, false, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.recovering &&
              (controller.recovery_direction == -1) &&
              (AbsFloat(left + TRACK_RECOVERY_KICK_SPEED_MPS) < 0.0001f) &&
              (AbsFloat(right - TRACK_RECOVERY_KICK_SPEED_MPS) < 0.0001f),
          "lost line after a right turn should immediately kick left");

    /* 最后误差没有越过 25 死区时，源工程对 0 误差默认向右搜索。 */
    LineTrack_Reset(&controller);
    LineTrack_Calculate(&controller, 0.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    LineTrack_Calculate(&controller, 0.0f, false, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.recovery_direction == -1,
          "zero-error fallback should reverse the default remembered right turn");

    /* 395 ms 仍反向搜线，下一拍到 400 ms 必须立即停车并锁存故障。 */
    LineTrack_Reset(&controller);
    LineTrack_Calculate(&controller, -100.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    for (i = 1U; i < lost_samples; i++) {
        LineTrack_Calculate(&controller, 0.0f, false,
                            APP_CONTROL_PERIOD_MS, &left, &right);
    }
    Check(controller.recovering && !controller.line_fault &&
              (controller.lost_time_ms ==
               TRACK_LOST_PROTECT_MS - APP_CONTROL_PERIOD_MS),
          "opposite recovery should remain active through 395 ms");
    LineTrack_Calculate(&controller, 0.0f, false, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.line_fault && !controller.running &&
              (controller.lost_time_ms == TRACK_LOST_PROTECT_MS) &&
              (AbsFloat(left) < 0.0001f) && (AbsFloat(right) < 0.0001f),
          "400 ms loss protection should stop both wheels");
}

static void TestReacquireAlignment(void)
{
    LineTrack_t controller;
    float left;
    float right;

    InitLineController(&controller);
    LineTrack_Calculate(&controller, -100.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    LineTrack_Calculate(&controller, -200.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check(controller.derivative < 0.0f,
          "continuous visible samples should retain the source derivative");

    LineTrack_Calculate(&controller, 0.0f, false, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    LineTrack_Calculate(&controller, 100.0f, true, APP_CONTROL_PERIOD_MS,
                        &left, &right);
    Check((AbsFloat(controller.derivative) < 0.0001f) &&
              (AbsFloat(controller.d_term_mps) < 0.0001f) &&
              (controller.lost_time_ms == 0U) && !controller.recovering,
          "the first reacquired sample must force derivative to zero");
    Check(AbsFloat(controller.correction_mps -
                   100.0f * TRACK_STEER_ENTRY_KP_MPS) < 0.0001f,
          "reacquisition correction should contain P only on its first frame");
}

/** @brief 让生产版 TrackSensor.c 采样一个指定的原始 GPIO 位图。 */
static TrackSensorData_t SampleTrack(uint8_t raw_mask)
{
    TrackSensorData_t data;

    g_test_track_raw_mask = raw_mask;
    TrackSensor_Sample();
    TrackSensor_GetData(&data);
    return data;
}

static void TestTrackSensorPatterns(void)
{
    TrackSensorData_t data;
    unsigned int i;

    TrackSensor_Init();
    data = SampleTrack(0xFFU); /* 全白：原始八路均为 1。 */
    Check(data.raw_mask == 0xFFU,
          "all-white raw mask should remain 11111111");
    Check((data.line_mask == 0U) && (data.active_count == 0U),
          "all-white input should contain no black line");
    Check((data.marker == 0U) && (data.ready == 0U),
          "all-white input must not arm or mark A");

    TrackSensor_Init();
    data = SampleTrack(0xE7U); /* CH4/CH5 为黑，内部 line_mask=0x18。 */
    Check((data.line_mask == 0x18U) && (data.active_count == 2U),
          "center pair should decode as two black sensors");
    Check(AbsFloat(data.error) < 0.0001f,
          "center pair should produce zero weighted error");

    TrackSensor_Init();
    data = SampleTrack(0x7FU); /* 仅最左 CH1 黑。 */
    Check(AbsFloat(data.error + 350.0f) < 0.0001f,
          "leftmost black sensor should produce source -350 error");

    TrackSensor_Init();
    data = SampleTrack(0xFEU); /* 仅最右 CH8 黑。 */
    Check(AbsFloat(data.error - 350.0f) < 0.0001f,
          "rightmost black sensor should produce source +350 error");

    TrackSensor_Init();
    data = SampleTrack(0xF8U); /* 内部 0x07：右侧连续 3 路黑。 */
    Check((data.active_count == 3U) && (data.marker != 0U) &&
              (data.junction == 0U),
          "three adjacent black sensors should be a weak A marker");

    TrackSensor_Init();
    data = SampleTrack(0xB6U); /* 内部 0x49：3 路黑但彼此不相邻。 */
    Check((data.active_count == 3U) && (data.marker == 0U),
          "three separated black sensors must not mark A");

    TrackSensor_Init();
    data = SampleTrack(0x87U); /* 内部 0x78：连续 4 路黑。 */
    Check((data.active_count == 4U) && (data.marker != 0U) &&
              (data.junction != 0U),
          "four adjacent black sensors should be a strong A marker");

    TrackSensor_Init();
    for (i = 0U; i < TRACK_READY_SAMPLES; i++) {
        data = SampleTrack(0x00U); /* 全黑横线也是合法启动图案。 */
    }
    Check(data.ready != 0U,
          "stable A marker should satisfy start readiness");
}

int main(void)
{
    TestRaceStartAndFinish();
    TestStrongMarkerFinish();
    TestRaceApproachFaultAndReject();
    TestSourceTrackingFormula();
    TestLineRecoverySearch();
    TestReacquireAlignment();
    TestTrackSensorPatterns();

    if (g_failed_checks != 0U) {
        printf("[FAIL] %u control checks failed\n", g_failed_checks);
        return 1;
    }

    printf("[OK] source tracking, recovery, sensor and race checks passed\n");
    return 0;
}
