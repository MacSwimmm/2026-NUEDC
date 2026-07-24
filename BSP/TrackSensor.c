#include "TrackSensor.h"

#include "bsp_config.h"
#include "ti_msp_dl_config.h"

static volatile TrackSensorData_t g_track_data;
static uint8_t g_ready_streak = 0U;

static uint8_t ReadRawMask(void)
{
    uint8_t raw = 0U;

    /* 固定软件位序，不依赖 GPIO 在两个端口中的物理排列。 */
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH8_PIN) != 0U) raw |= 0x80U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH7_PIN) != 0U) raw |= 0x40U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH6_PIN) != 0U) raw |= 0x20U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH5_PIN) != 0U) raw |= 0x10U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH4_PIN) != 0U) raw |= 0x08U;
    if (DL_GPIO_readPins(TRACK_A_PORT, TRACK_A_CH3_PIN) != 0U) raw |= 0x04U;
    if (DL_GPIO_readPins(TRACK_B_PORT, TRACK_B_CH2_PIN) != 0U) raw |= 0x02U;
    if (DL_GPIO_readPins(TRACK_B_PORT, TRACK_B_CH1_PIN) != 0U) raw |= 0x01U;
    return raw;
}

void TrackSensor_Init(void)
{
    g_track_data.raw_mask = 0U;
    g_track_data.line_mask = 0U;
    g_track_data.active_count = 0U;
    g_track_data.ready = 0U;
    g_track_data.junction = 0U;
    g_track_data.error = 0.0f;
    g_ready_streak = 0U;
}

/**
 * @brief 采样八路数字循迹模块并计算横向误差。
 *
 * 权重从左到右为 +70、+50、+30、+10、-10、-30、-50、-70。
 * 多路同时压线时取平均值，因此中心两路同时有效时误差接近 0。
 * 这里只做一次采样和加权，没有弯道、路口或丢线状态机。
 */
void TrackSensor_Sample(void)
{
    static const int8_t weights[8] = {70, 50, 30, 10, -10, -30, -50, -70};
    uint8_t raw = ReadRawMask();
    uint8_t mask = (uint8_t)(raw ^ TRACK_ACTIVE_LOW_MASK);
    uint8_t count = 0U;
    uint8_t i;
    int16_t sum = 0;

    for (i = 0U; i < 8U; i++) {
        if ((mask & (uint8_t)(0x80U >> i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }

    g_track_data.raw_mask = raw;
    g_track_data.line_mask = mask;
    g_track_data.active_count = count;
    g_track_data.junction = (mask == 0xFFU) ? 1U : 0U;
    g_track_data.error = (count == 0U) ? 0.0f : ((float)sum / (float)count);

    /* 全白与全黑都不作为“可安全启动”的正常图案。 */
    if ((mask != 0U) && (mask != 0xFFU)) {
        if (g_ready_streak < TRACK_READY_SAMPLES) g_ready_streak++;
    } else {
        g_ready_streak = 0U;
    }
    g_track_data.ready = (g_ready_streak >= TRACK_READY_SAMPLES) ? 1U : 0U;
}

void TrackSensor_GetData(TrackSensorData_t *data)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    data->raw_mask = g_track_data.raw_mask;
    data->line_mask = g_track_data.line_mask;
    data->active_count = g_track_data.active_count;
    data->ready = g_track_data.ready;
    data->junction = g_track_data.junction;
    data->error = g_track_data.error;
    if (primask == 0U) {
        __enable_irq();
    }
}
