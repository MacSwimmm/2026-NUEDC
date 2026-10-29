#ifndef F407_BALANCE_OLED_DISPLAY_H
#define F407_BALANCE_OLED_DISPLAY_H

/**
 ******************************************************************************
 * @file    OLED_Display.h
 * @brief   将两键菜单和只读控制快照绘制到 0.96 英寸 SSD1306 OLED。
 ******************************************************************************
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_DISPLAY_REFRESH_MS          100U
#define OLED_DISPLAY_BLINK_HALF_MS       250U

typedef enum
{
    OLED_DISPLAY_CAMERA_OFF = 0,
    OLED_DISPLAY_LOST,
    OLED_DISPLAY_LOCK
} OLED_DisplayMode_t;

/** @brief Watch 镜像；显示层只消费控制快照，不参与任何闭环决策。 */
typedef struct
{
    uint8_t mode;
    uint8_t page;
    uint8_t home_detail;
    uint8_t track_run_detail;
    uint8_t balance_state;
    uint8_t track_state;
    int16_t position_mm;
    int32_t velocity_mm_s;
    uint32_t last_valid_age_ms;
    uint32_t render_count;
    uint32_t submit_count;
    uint32_t submit_drop_count;
    uint32_t last_render_tick;
} OLED_DisplayDebug_t;

extern volatile OLED_DisplayDebug_t g_oled_display_debug;

/** @brief 清空显示缓存并启动非阻塞 OLED 上电状态机，不发送运动命令。 */
void OLED_DisplayInit(void);

/** @brief 放在主循环最后调用；每 100 ms 最多提交一帧，忙时直接丢帧。 */
void OLED_DisplayProcess(void);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_OLED_DISPLAY_H */
