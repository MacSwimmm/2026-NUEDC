#include "Race_Control.h"

#include "Board.h"
#include "Car_Control.h"
#include "app_config.h"

/*
 * 三个按键共用一套主循环状态机，但速度档与结束条件彼此独立：
 *
 * PB21 CURRENT_LAP：保持原 5.08 倍参数、A 点确认和圈末停车逻辑；
 * PB0  AUX_LAP：切到 3.50 倍速度档，4 s 缓启动后按同一 A 点逻辑跑一圈；
 * PB1  AUX_STRAIGHT：切到 3.50 倍速度档并继续读取循迹误差，6 s 后用 2 s
 *                    把左右轮完整目标同步缩到 0，再统一清 PI、写零 PWM。
 *
 * 任一模式运行中再次按下任一模式键都先停车，不带着旧 PI 或转向历史切换。
 */
static RaceTelemetry_t g_race;
static uint32_t g_start_ms = 0U;
static uint32_t g_last_control_tick = 0U;

/* Watch 镜像只由主循环覆盖写入，控制状态机从不读取其中字段。 */
volatile RaceControlWatch_t g_race_watch;

/** @brief 把浮点倍率限制到 0~1。 */
static float ClampUnit(float value)
{
    if (value > 1.0f) return 1.0f;
    if (value < 0.0f) return 0.0f;
    return value;
}

/** @brief 从 0 到 1 的线性缓启动倍率。 */
static float SelectLaunchScale(uint32_t elapsed_ms, uint32_t ramp_ms)
{
    if ((ramp_ms == 0U) || (elapsed_ms >= ramp_ms)) return 1.0f;
    return ClampUnit((float)elapsed_ms / (float)ramp_ms);
}

/** @brief PB1 的缓启动、匀速循迹和 2 s 缓减速倍率。 */
static float SelectStraightScale(uint32_t elapsed_ms, uint8_t *ramping)
{
    *ramping = 0U;
    if (elapsed_ms < PB1_STRAIGHT_LAUNCH_RAMP_MS) {
        *ramping = 1U;
        return SelectLaunchScale(elapsed_ms,
                                 PB1_STRAIGHT_LAUNCH_RAMP_MS);
    }
    if (elapsed_ms < PB1_STRAIGHT_RUN_MS) {
        return 1.0f;
    }
    if (elapsed_ms < PB1_STRAIGHT_TOTAL_MS) {
        *ramping = 1U;
        return ClampUnit(
            (float)(PB1_STRAIGHT_TOTAL_MS - elapsed_ms) /
            (float)PB1_STRAIGHT_DECEL_MS);
    }
    return 0.0f;
}

/** @brief 返回当前是否仍有模式占用底盘。 */
static bool IsRunning(void)
{
    return (g_race.state == RACE_STATE_LEAVING_A) ||
           (g_race.state == RACE_STATE_RUNNING);
}

/** @brief 将真实比赛状态复制到 Watch；最后写控制周期号作为刷新标志。 */
static void UpdateRaceWatch(void)
{
    g_race_watch.mode = g_race.mode;
    g_race_watch.state = g_race.state;
    g_race_watch.result = g_race.result;
    g_race_watch.marker_cleared = g_race.marker_cleared;
    g_race_watch.approach_active = g_race.approach_active;
    g_race_watch.marker_clear_streak = g_race.marker_clear_streak;
    g_race_watch.marker_confirm_streak = g_race.marker_confirm_streak;
    g_race_watch.start_ms = g_start_ms;
    g_race_watch.elapsed_ms = g_race.elapsed_ms;
    g_race_watch.final_time_ms = g_race.final_time_ms;
    g_race_watch.start_reject_count = g_race.start_reject_count;
    g_race_watch.last_control_tick = g_last_control_tick;
}

/** @brief 无条件停车并恢复 PB21 当前速度档，等待下一次模式选择。 */
static void EnterWaiting(void)
{
    Car_ControlSetArmed(false);
    Car_ControlSetTrackingSpeedProfile(CAR_TRACK_SPEED_PROFILE_CURRENT);
    Car_ControlSetTrackingOutputScale(1.0f);
    Car_ControlSetBaseSpeed(RACE_CRUISE_SPEED_MPS);

    g_race.mode = RACE_MODE_NONE;
    g_race.state = RACE_STATE_WAITING;
    g_race.result = RACE_RESULT_NONE;
    g_race.marker_cleared = 0U;
    g_race.approach_active = 0U;
    g_race.marker_clear_streak = 0U;
    g_race.marker_confirm_streak = 0U;
    g_race.elapsed_ms = 0U;
    g_race.final_time_ms = 0U;
    g_start_ms = 0U;
    UpdateRaceWatch();
}

/** @brief 冻结循迹时间并通过统一整车接口立即停车。 */
static void FinishRace(RaceResult_t result, uint32_t now_ms)
{
    g_race.elapsed_ms = now_ms - g_start_ms;
    g_race.final_time_ms = g_race.elapsed_ms;
    g_race.result = result;
    g_race.state = RACE_STATE_FINISHED;
    g_race.approach_active = 0U;
    Car_ControlSetArmed(false);
    UpdateRaceWatch();
}

/** @brief 启动 PB21/PB0 的一圈循迹，或 PB1 的定时直线循迹。 */
static void StartMode(RaceMode_t mode)
{
    CarTelemetry_t car;
    CarTrackSpeedProfile_t profile;
    float base_speed_mps;
    float initial_output_scale;
    uint32_t now_ms = Board_Millis();

    if (IsRunning()) {
        EnterWaiting();
        return;
    }

    if ((mode == RACE_MODE_AUX_LAP) ||
        (mode == RACE_MODE_AUX_STRAIGHT)) {
        profile = CAR_TRACK_SPEED_PROFILE_AUX_3_50;
        base_speed_mps = AUX_TRACK_BASE_SPEED_MPS;
        initial_output_scale = 0.0f;
    } else {
        mode = RACE_MODE_CURRENT_LAP;
        profile = CAR_TRACK_SPEED_PROFILE_CURRENT;
        base_speed_mps = RACE_CRUISE_SPEED_MPS;
        initial_output_scale = 1.0f;
    }

    /* 先锁存当前控制周期，再停车切档；新运行从下一份真实 5 ms 样本开始计数。 */
    Car_ControlGetTelemetry(&car);
    Car_ControlSetTrackingSpeedProfile(profile);
    Car_ControlSetBaseSpeed(base_speed_mps);
    Car_ControlSetTrackingOutputScale(initial_output_scale);
    if (!Car_ControlSetArmed(true)) {
        EnterWaiting();
        g_race.mode = mode;
        g_race.result = RACE_RESULT_START_REJECTED;
        g_race.start_reject_count++;
        UpdateRaceWatch();
        return;
    }

    g_start_ms = now_ms;
    g_last_control_tick = car.control_tick_count;
    g_race.mode = mode;
    g_race.state = (mode == RACE_MODE_AUX_STRAIGHT) ?
                   RACE_STATE_RUNNING : RACE_STATE_LEAVING_A;
    g_race.result = RACE_RESULT_NONE;
    g_race.marker_cleared =
        (mode == RACE_MODE_AUX_STRAIGHT) ? 1U : 0U;
    g_race.approach_active =
        (initial_output_scale < 1.0f) ? 1U : 0U;
    g_race.marker_clear_streak = 0U;
    g_race.marker_confirm_streak = 0U;
    g_race.elapsed_ms = 0U;
    g_race.final_time_ms = 0U;
    UpdateRaceWatch();
}

void Race_ControlInit(void)
{
    g_race.start_reject_count = 0U;
    g_last_control_tick = 0U;
    EnterWaiting();
}

void Race_ControlHandleUserKey(void)
{
    StartMode(RACE_MODE_CURRENT_LAP);
}

void Race_ControlHandleLapKey(void)
{
    StartMode(RACE_MODE_AUX_LAP);
}

void Race_ControlHandleStraightKey(void)
{
    StartMode(RACE_MODE_AUX_STRAIGHT);
}

void Race_ControlStop(void)
{
    EnterWaiting();
}

/** @brief 推进 PB1 的 6 s 直线循迹和 2 s 缓停。 */
static void ProcessStraightMode(uint32_t now_ms)
{
    CarTelemetry_t car;
    float output_scale;

    g_race.elapsed_ms = now_ms - g_start_ms;
    Car_ControlGetTelemetry(&car);
    if (car.line_fault != 0U) {
        FinishRace(RACE_RESULT_LINE_LOST, now_ms);
        return;
    }

    if (g_race.elapsed_ms >= PB1_STRAIGHT_TOTAL_MS) {
        /* 即使主循环略晚，定时流程的显示成绩仍固定为 6000+2000 ms。 */
        g_race.elapsed_ms = PB1_STRAIGHT_TOTAL_MS;
        g_race.final_time_ms = PB1_STRAIGHT_TOTAL_MS;
        g_race.result = RACE_RESULT_SUCCESS;
        g_race.state = RACE_STATE_FINISHED;
        g_race.approach_active = 0U;
        Car_ControlSetTrackingOutputScale(0.0f);
        Car_ControlSetArmed(false);
        UpdateRaceWatch();
        return;
    }

    output_scale = SelectStraightScale(g_race.elapsed_ms,
                                       &g_race.approach_active);
    Car_ControlSetTrackingOutputScale(output_scale);
    UpdateRaceWatch();
}

/** @brief 推进 PB21/PB0 共用的 A 点离开与第二次到达确认。 */
static void ProcessLapMode(uint32_t now_ms)
{
    CarTelemetry_t car;

    g_race.elapsed_ms = now_ms - g_start_ms;

    if (g_race.mode == RACE_MODE_AUX_LAP) {
        g_race.approach_active =
            (g_race.elapsed_ms < PB0_LAP_LAUNCH_RAMP_MS) ? 1U : 0U;
        Car_ControlSetTrackingOutputScale(
            SelectLaunchScale(g_race.elapsed_ms,
                              PB0_LAP_LAUNCH_RAMP_MS));
    } else if ((g_race.approach_active == 0U) &&
               (g_race.elapsed_ms >= RACE_APPROACH_START_MS)) {
        /* PB21 的原有固定时刻请求与参数保持不变。 */
        g_race.approach_active = 1U;
        Car_ControlSetBaseSpeed(RACE_APPROACH_SPEED_MPS);
    }

    Car_ControlGetTelemetry(&car);
    if (car.line_fault != 0U) {
        FinishRace(RACE_RESULT_LINE_LOST, now_ms);
        return;
    }

    /* 同一个 5 ms 样本只能推进一次横线连续计数。 */
    if (car.control_tick_count == g_last_control_tick) {
        UpdateRaceWatch();
        return;
    }
    g_last_control_tick = car.control_tick_count;

    if (g_race.state == RACE_STATE_LEAVING_A) {
        if (car.junction == 0U) {
            if (g_race.marker_clear_streak < RACE_MARKER_CLEAR_SAMPLES) {
                g_race.marker_clear_streak++;
            }
        } else {
            g_race.marker_clear_streak = 0U;
        }

        if (g_race.marker_clear_streak >= RACE_MARKER_CLEAR_SAMPLES) {
            g_race.marker_cleared = 1U;
            g_race.marker_confirm_streak = 0U;
            g_race.state = RACE_STATE_RUNNING;
        }
        UpdateRaceWatch();
        return;
    }

    /* PB21/PB0 均要求驶离起点、越过最低时间，并连续 30 ms 看到强 A 点。 */
    if (g_race.elapsed_ms < RACE_FINISH_MIN_TIME_MS) {
        g_race.marker_confirm_streak = 0U;
        UpdateRaceWatch();
        return;
    }

    if (car.junction != 0U) {
        if (g_race.marker_confirm_streak <
            RACE_MARKER_STRONG_CONFIRM_SAMPLES) {
            g_race.marker_confirm_streak++;
        }
    } else {
        g_race.marker_confirm_streak = 0U;
    }

    if (g_race.marker_confirm_streak >=
        RACE_MARKER_STRONG_CONFIRM_SAMPLES) {
        FinishRace(RACE_RESULT_SUCCESS, now_ms);
    } else {
        UpdateRaceWatch();
    }
}

void Race_ControlProcess(void)
{
    uint32_t now_ms;

    if (!IsRunning()) return;

    now_ms = Board_Millis();
    if (g_race.mode == RACE_MODE_AUX_STRAIGHT) {
        ProcessStraightMode(now_ms);
    } else {
        ProcessLapMode(now_ms);
    }
}

void Race_ControlGetTelemetry(RaceTelemetry_t *telemetry)
{
    telemetry->mode = g_race.mode;
    telemetry->state = g_race.state;
    telemetry->result = g_race.result;
    telemetry->marker_cleared = g_race.marker_cleared;
    telemetry->approach_active = g_race.approach_active;
    telemetry->marker_clear_streak = g_race.marker_clear_streak;
    telemetry->marker_confirm_streak = g_race.marker_confirm_streak;
    telemetry->elapsed_ms = g_race.elapsed_ms;
    telemetry->final_time_ms = g_race.final_time_ms;
    telemetry->start_reject_count = g_race.start_reject_count;
}
