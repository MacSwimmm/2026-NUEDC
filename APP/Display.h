#ifndef G3507_BASE_DISPLAY_H
#define G3507_BASE_DISPLAY_H

/**
 * @file Display.h
 * @brief OLED 应用层页面管理接口。
 *
 * Display 只读取整车和 IMU 快照并组织页面，不直接操作 GPIO 或软件 SPI 时序；
 * 底层字节发送、显存和 SSD1306 初始化均由 BSP/OLED 负责。
 */

/** 初始化 OLED 并把当前页设为底盘页。 */
void Display_Init(void);

/** 切换到下一页，超过最后一页后回到第一页。 */
void Display_NextPage(void);

/** 根据当前页重绘 1 KiB 显存并一次性刷新到 OLED。 */
void Display_Update(void);

#endif
