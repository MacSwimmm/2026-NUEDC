#ifndef G3507_BASE_OLED_H
#define G3507_BASE_OLED_H

#include <stdint.h>

/**
 * @file OLED.h
 * @brief SSD1306 128×64 OLED 的 1 KiB 显存与软件四线 SPI 接口。
 *
 * 所有绘图函数只修改 RAM 显存，OLED_Refresh() 才把整帧发送到屏幕。
 * 这样应用可以先完成整页绘制再统一刷新，减少字段逐个更新造成的闪烁。
 */

#define OLED_WIDTH   128U
#define OLED_HEIGHT  64U

/* 完整 1 KiB 显存：128 列 × 8 页，每页字节对应垂直 8 个像素。 */
extern uint8_t OLED_GRAM[OLED_WIDTH][OLED_HEIGHT / 8U];

/** 复位并按 SSD1306 128×64 页寻址模式完成初始化。 */
void OLED_Init(void);

/** 仅清空 RAM 显存，不立即操作屏幕。 */
void OLED_ClearBuffer(void);

/** 清空 RAM 显存并立即刷新一帧黑屏。 */
void OLED_Clear(void);

/** 将完整 1 KiB 显存按 8 页发送到 OLED。 */
void OLED_Refresh(void);

/** 在显存中绘制一个 6×12 ASCII 字符，超出可打印范围时显示问号。 */
void OLED_ShowChar(uint8_t x, uint8_t y, char character);

/** 从指定坐标连续绘制字符串，超过屏幕右边界后停止。 */
void OLED_ShowString(uint8_t x, uint8_t y, const char *text);

/** 以固定十进制位数绘制无符号整数，高位不足时补 0。 */
void OLED_ShowUnsigned(uint8_t x, uint8_t y, uint32_t value, uint8_t digits);

#endif
