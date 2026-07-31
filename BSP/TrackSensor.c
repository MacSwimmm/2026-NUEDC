#include "TrackSensor.h"

#include "bsp_config.h"
#include "ti_msp_dl_config.h"

/* 采样结果由 5 ms 控制路径更新，也可能被主循环使能检查读取。 */
static volatile TrackSensorData_t g_track_data;
/* 连续正常图案计数用于抑制上电瞬间和插拔接线时的误使能。 */
static uint8_t g_ready_streak = 0U;

/** @brief 按固定空间顺序读取八个 GPIO 原始电平并打包为 8 位位图。 */
static uint8_t ReadRawMask(void)
{
    uint8_t raw = 0U;

    /*
     * 固定软件位序为“bit7 最左、bit0 最右”，不依赖两个 GPIO 端口的物理排列。
     * 实车模块从左到右安装为 CH1~CH8，因此 CH1 放入最高位、CH8 放入最低位。
     */
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH1_PIN) != 0U) raw |= 0x80U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH2_PIN) != 0U) raw |= 0x40U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH3_PIN) != 0U) raw |= 0x20U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH4_PIN) != 0U) raw |= 0x10U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH5_PIN) != 0U) raw |= 0x08U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH6_PIN) != 0U) raw |= 0x04U;
    if (DL_GPIO_readPins(TRACK_B_PORT, TRACK_B_CH7_PIN) != 0U) raw |= 0x02U;
    if (DL_GPIO_readPins(TRACK_B_PORT, TRACK_B_CH8_PIN) != 0U) raw |= 0x01U;
    return raw;
}

/**
 * @brief 返回八位位图中最长的连续 1 数量。
 *
 * A 点是一段加宽黑线，车辆居中通过时通常能覆盖中间约 4 路探头；高速回线时
 * 若车身存在横向偏差，实测可能只在探头一侧连续覆盖 3 路。这里只接受连续相邻
 * 通道，避免三个彼此分散的异常低电平被误认为横线。
 */
static uint8_t GetLongestActiveRun(uint8_t mask)
{
    uint8_t longest = 0U;
    uint8_t current = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        if ((mask & (uint8_t)(0x80U >> i)) != 0U) {
            current++;
            if (current > longest) longest = current;
        } else {
            current = 0U;
        }
    }
    return longest;
}

void TrackSensor_Init(void)
{
    /* 初始化为“未就绪且无有效线”，只有后续真实采样才能开放整车使能。 */
    g_track_data.raw_mask = 0U;
    g_track_data.line_mask = 0U;
    g_track_data.active_count = 0U;
    g_track_data.ready = 0U;
    g_track_data.marker = 0U;
    g_track_data.junction = 0U;
    g_track_data.error = 0.0f;
    g_ready_streak = 0U;
}

/**
 * @brief 采样八路数字循迹模块并计算横向误差。
 *
 * 权重完整沿用源工程：从左到右为 -350、-250、-150、-50、
 * +50、+150、+250、+350。
 * 多路同时压线时取平均值，因此中心两路同时有效时误差接近 0。
 * 这里只做一次采样和加权，没有弯道、路口或丢线状态机。
 *
 * raw 是模块电气原始值；与 TRACK_ACTIVE_LOW_MASK 异或后得到 mask。
 * 当前 MASK=0xFF，因此原始低电平会变成 line_mask 中的 1。
 */
void TrackSensor_Sample(void)
{
    static const int16_t weights[8] = {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    uint8_t raw = ReadRawMask();
    uint8_t mask = (uint8_t)(raw ^ TRACK_ACTIVE_LOW_MASK);
    uint8_t count = 0U;
    uint8_t longest_run;
    uint8_t i;
    int32_t sum = 0;

    /* 从 bit7/最左开始遍历，权重数组与物理空间顺序严格一一对应。 */
    for (i = 0U; i < 8U; i++) {
        if ((mask & (uint8_t)(0x80U >> i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }

    /* 一次计算完成后再写入共享快照，减少读取侧看到中间状态的时间窗口。 */
    longest_run = GetLongestActiveRun(mask);

    g_track_data.raw_mask = raw;
    g_track_data.line_mask = mask;
    g_track_data.active_count = count;
    g_track_data.marker =
        (longest_run >= TRACK_MARKER_WEAK_MIN_CONTIGUOUS) ? 1U : 0U;
    g_track_data.junction =
        (longest_run >= TRACK_MARKER_STRONG_MIN_CONTIGUOUS) ? 1U : 0U;
    g_track_data.error = (count == 0U) ? 0.0f : ((float)sum / (float)count);

    /*
     * 全白表示没有检测到黑线，不能允许启动；A 点横线也是合法启动图案。
     * 横线的强弱分级、连续确认和“先离开再返回”由比赛层完成。
     */
    if (mask != 0U) {
        if (g_ready_streak < TRACK_READY_SAMPLES) g_ready_streak++;
    } else {
        g_ready_streak = 0U;
    }
    g_track_data.ready = (g_ready_streak >= TRACK_READY_SAMPLES) ? 1U : 0U;
}

void TrackSensor_GetData(TrackSensorData_t *data)
{
    uint32_t primask = __get_PRIMASK();

    /* error 为 32 位 float，但整份结构跨多个字段，必须整体保护以保证同周期一致性。 */
    __disable_irq();
    data->raw_mask = g_track_data.raw_mask;
    data->line_mask = g_track_data.line_mask;
    data->active_count = g_track_data.active_count;
    data->ready = g_track_data.ready;
    data->marker = g_track_data.marker;
    data->junction = g_track_data.junction;
    data->error = g_track_data.error;
    if (primask == 0U) {
        __enable_irq();
    }
}
