#include "Display.h"

#include "Car_Control.h"
#include "OLED.h"
#include "Race_Control.h"

#include <stdint.h>

/**
 * @brief 显示固定宽度带符号整数。
 * @param x/y    左上角像素坐标
 * @param value  待显示数值
 * @param digits 绝对值十进制位数，不含符号
 */
static void ShowSignedInteger(uint8_t x, uint8_t y,
                              int32_t value, uint8_t digits)
{
    uint32_t magnitude;

    if (value < 0) {
        OLED_ShowString(x, y, "-");
        magnitude = (uint32_t)(-value);
    } else {
        OLED_ShowString(x, y, "+");
        magnitude = (uint32_t)value;
    }
    OLED_ShowUnsigned((uint8_t)(x + 8U), y, magnitude, digits);
}

/**
 * @brief 显示 SSS.mmm 格式的比赛时间。
 *
 * 秒数最多显示 999；该限幅只保护屏幕排版，不改变 Race_Control 中保存的
 * uint32_t 原始毫秒时间。
 */
static void ShowRaceTime(uint8_t x, uint8_t y, uint32_t time_ms)
{
    uint32_t seconds = time_ms / 1000U;
    uint32_t milliseconds = time_ms % 1000U;

    if (seconds > 999U) seconds = 999U;
    OLED_ShowUnsigned(x, y, seconds, 3U);
    OLED_ShowString((uint8_t)(x + 24U), y, ".");
    OLED_ShowUnsigned((uint8_t)(x + 32U), y, milliseconds, 3U);
}

/** @brief 把八路位图按 CH1 到 CH8 的物理顺序显示为 8 个字符。 */
static void ShowTrackBits(uint8_t x, uint8_t y, uint8_t mask)
{
    char bits[9];
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        bits[i] = ((mask & (uint8_t)(0x80U >> i)) != 0U) ? '1' : '0';
    }
    bits[8] = '\0';
    OLED_ShowString(x, y, bits);
}

/** @brief 将 m/s 转为 OLED 使用的 mm/s，并限制到三位绝对值。 */
static int32_t SpeedToMillimeters(float speed_mps)
{
    int32_t speed_mmps = (int32_t)(speed_mps * 1000.0f);

    if (speed_mmps > 999) speed_mmps = 999;
    if (speed_mmps < -999) speed_mmps = -999;
    return speed_mmps;
}

/** @brief 返回 OLED 顶行使用的按键模式与独立速度档。 */
static const char *GetModeText(RaceMode_t mode)
{
    if (mode == RACE_MODE_CURRENT_LAP) return "PB21 LAP 5.08";
    if (mode == RACE_MODE_AUX_LAP) return "PB0 LAP 3.50";
    if (mode == RACE_MODE_AUX_STRAIGHT) return "PB1 LINE 3.50";
    return "READY:21/0/1";
}

/** @brief 根据比赛状态和终止原因返回状态文本。 */
static const char *GetStateText(const RaceTelemetry_t *race)
{
    if (race->state == RACE_STATE_LEAVING_A) return "LEAVE";
    if (race->state == RACE_STATE_RUNNING) return "RUN";
    if (race->state == RACE_STATE_FINISHED) {
        return (race->result == RACE_RESULT_SUCCESS) ? "DONE" : "LOST";
    }
    if (race->result == RACE_RESULT_START_REJECTED) return "NO LINE";
    return "WAIT";
}

/** @brief 绘制模式、计时、循迹位图以及左右轮目标/反馈。 */
static void DrawRacePage(const CarTelemetry_t *car,
                         const RaceTelemetry_t *race)
{
    uint32_t shown_time = (race->state == RACE_STATE_FINISHED) ?
                          race->final_time_ms : race->elapsed_ms;
    uint32_t output_percent =
        (uint32_t)(car->tracking_output_scale * 100.0f + 0.5f);

    if (output_percent > 100U) output_percent = 100U;

    /* 第 1 行明确显示由哪个按键启动，以及该模式使用的速度倍率。 */
    OLED_ShowString(0U, 0U, GetModeText(race->mode));

    /* 第 2 行：流程状态与实时/冻结时间。 */
    OLED_ShowString(0U, 13U, GetStateText(race));
    ShowRaceTime(64U, 13U, shown_time);

    /* 第 3 行：算法实际使用的 CH1~CH8 位图，以及缓启停输出百分比。 */
    OLED_ShowString(0U, 26U, "TR:");
    ShowTrackBits(24U, 26U, car->track_mask);
    OLED_ShowString(96U, 26U, "X");
    OLED_ShowUnsigned(104U, 26U, output_percent, 3U);

    /* 第 4、5 行：左右轮目标 T 与编码器反馈 V，均为带符号 mm/s。 */
    OLED_ShowString(0U, 39U, "LT");
    ShowSignedInteger(16U, 39U,
                      SpeedToMillimeters(car->left_target_mps), 3U);
    OLED_ShowString(56U, 39U, "LV");
    ShowSignedInteger(72U, 39U,
                      SpeedToMillimeters(car->left_speed_mps), 3U);

    OLED_ShowString(0U, 52U, "RT");
    ShowSignedInteger(16U, 52U,
                      SpeedToMillimeters(car->right_target_mps), 3U);
    OLED_ShowString(56U, 52U, "RV");
    ShowSignedInteger(72U, 52U,
                      SpeedToMillimeters(car->right_speed_mps), 3U);
}

void Display_Init(void)
{
    OLED_Init();
    OLED_Clear();
}

void Display_Update(void)
{
    CarTelemetry_t car;
    RaceTelemetry_t race;

    /*
     * Car 快照在临界区内复制，Race 快照只由主循环更新。先取得两份完整数据，
     * 再清显存和绘制，避免一帧中混入不同控制周期的字段。
     */
    Car_ControlGetTelemetry(&car);
    Race_ControlGetTelemetry(&race);
    OLED_ClearBuffer();
    DrawRacePage(&car, &race);
    OLED_Refresh();
}
