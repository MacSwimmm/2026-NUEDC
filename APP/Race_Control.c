#include "Race_Control.h"

#include "Board.h"
#include "Car_Control.h"
#include "app_config.h"

/*
 * 第 2 项流程保持为四个状态：
 *
 * WAITING --PB21--> LEAVING_A --横线连续消失--> RUNNING
 * RUNNING --第二次 A 点强标志连续确认--> FINISHED
 *
 * 启动时探头允许正压在 A 点加宽黑线上。只有候选标志连续消失 3 个真实控制
 * 样本后 marker_cleared 才置位；终点还必须跨过最低时间门限，因此起点附近
 * 从普通两路黑线逐渐进入加宽区域也不会被当成完成。
 * 三路黑可能只是圆弧或赛道边缘，不再作为完成条件。
 */
static RaceTelemetry_t g_race;
static uint32_t g_start_ms = 0U;
static uint32_t g_last_control_tick = 0U;

/*
 * 对外可见的 Keil Watch 只读镜像。状态机只读写上面的 static 真实状态，
 * 因此在调试器里修改 g_race_watch 不会跳转状态或触发停车。
 */
volatile RaceControlWatch_t g_race_watch;

/** @brief 将真实比赛状态复制到 Watch；last_control_tick 最后写入作为刷新标志。 */
static void UpdateRaceWatch(void)
{
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

/** @brief 停车并回到等待状态，用于上电初始化和运行中人工中止。 */
static void EnterWaiting(void)
{
    Car_ControlSetArmed(false);
    Car_ControlSetBaseSpeed(RACE_CRUISE_SPEED_MPS);

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

/**
 * @brief 冻结时间并进入终止状态。
 * @param result  正常完成或持续丢线
 * @param now_ms  本次判定使用的 1 ms 时基快照
 *
 * 先冻结时间，再通过统一的 Car_Control 停车接口清 PI 和写零 PWM。正常终点
 * 确认发生在第三个 5 ms 横线样本之后，因而显示时间对应实际停车判定时刻。
 */
static void FinishRace(RaceResult_t result, uint32_t now_ms)
{
    g_race.elapsed_ms = now_ms - g_start_ms;
    g_race.final_time_ms = g_race.elapsed_ms;
    g_race.result = result;
    g_race.state = RACE_STATE_FINISHED;
    Car_ControlSetArmed(false);
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
    CarTelemetry_t car;
    uint32_t now_ms = Board_Millis();

    if ((g_race.state == RACE_STATE_LEAVING_A) ||
        (g_race.state == RACE_STATE_RUNNING)) {
        /*
         * 实车调试时再次按 PB21 可立即人工中止。该路径不算完成，不保留旧时间，
         * 下一次按键仍从一圈全新计时开始。
         */
        EnterWaiting();
        return;
    }

    /*
     * WAITING 或 FINISHED 均允许重新开始。先读取当前控制周期编号，启动后必须
     * 等待下一个真实 5 ms 样本，不能把按键所在周期的 A 点状态重复计数。
     */
    Car_ControlGetTelemetry(&car);
    Car_ControlSetBaseSpeed(RACE_CRUISE_SPEED_MPS);
    if (!Car_ControlSetArmed(true)) {
        /*
         * 失败启动也清掉上一圈冻结时间，避免 OLED 在 NO LINE 状态下继续显示
         * 旧成绩；累计拒绝次数保留，便于判断是否为探头未就绪。
         */
        EnterWaiting();
        g_race.result = RACE_RESULT_START_REJECTED;
        g_race.start_reject_count++;
        UpdateRaceWatch();
        return;
    }

    g_start_ms = now_ms;
    g_last_control_tick = car.control_tick_count;
    g_race.state = RACE_STATE_LEAVING_A;
    g_race.result = RACE_RESULT_NONE;
    g_race.marker_cleared = 0U;
    g_race.approach_active = 0U;
    g_race.marker_clear_streak = 0U;
    g_race.marker_confirm_streak = 0U;
    g_race.elapsed_ms = 0U;
    g_race.final_time_ms = 0U;
    UpdateRaceWatch();
}

void Race_ControlProcess(void)
{
    CarTelemetry_t car;
    uint32_t now_ms;

    if ((g_race.state != RACE_STATE_LEAVING_A) &&
        (g_race.state != RACE_STATE_RUNNING)) {
        return;
    }

    now_ms = Board_Millis();
    g_race.elapsed_ms = now_ms - g_start_ms;

    /*
     * 第一版使用固定时间门限预减速。只在跨过门限时设置一次请求速度，
     * 后续 5 ms 控制层保持原有线速度公式，不增加直线或弯道速度状态。
     */
    if ((g_race.approach_active == 0U) &&
        (g_race.elapsed_ms >= RACE_APPROACH_START_MS)) {
        g_race.approach_active = 1U;
        Car_ControlSetBaseSpeed(RACE_APPROACH_SPEED_MPS);
    }

    Car_ControlGetTelemetry(&car);
    if (car.line_fault != 0U) {
        /* 持续丢线属于故障停车，不伪装成到达 A 点。 */
        FinishRace(RACE_RESULT_LINE_LOST, now_ms);
        return;
    }

    /*
     * 主循环通常远快于 5 ms。若仍是同一个控制样本，只更新时间而不推进
     * 横线连续计数；否则一次 while 循环就可能把一个样本误累加到 3 次。
     */
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

    /*
     * 第二次 A 点必须同时满足：
     * 1. 起点横线已经离开；
     * 2. 运行时间超过最低门限；
     * 3. 连续相邻至少 4 路黑的强标志持续 30 ms。
     *
     * 三路黑不再容错为 A 点：它在宽圆弧、车身偏出和边缘阴影下都可能出现，
     * 上一份跑飞日志正是由边缘三路黑误报正常完成。
     */
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

void Race_ControlGetTelemetry(RaceTelemetry_t *telemetry)
{
    /*
     * Race_ControlProcess、Display_Update 和本函数都在主循环调用，不存在 ISR
     * 并发写入；逐项复制仅用于保持接口清晰并方便后续增加赛题项目。
     */
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
