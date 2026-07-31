#ifndef G3507_BASE_DISPLAY_H
#define G3507_BASE_DISPLAY_H

/**
 * @file Display.h
 * @brief 三按键模式、速度档、计时和循迹遥测 OLED 显示接口。
 *
 * 固定保留一页运行信息，避免页面切换或舵机/IMU显示在运行中占用主循环。
 * JY901S 驱动和 UART2 仍保留，但不参与本页显示与循迹控制。
 */

/** 初始化 SSD1306，并清空屏幕。 */
void Display_Init(void);

/** 读取比赛与底盘快照，重绘完整 128×64 显存并一次性刷新。 */
void Display_Update(void);

#endif
