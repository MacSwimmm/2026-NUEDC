#ifndef G3507_BASE_OLED_H
#define G3507_BASE_OLED_H

#include <stdint.h>

#define OLED_WIDTH   128U
#define OLED_HEIGHT  64U

/* 完整 1 KiB 显存；APP 先绘制缓冲区，再由 OLED_Refresh 一次发送整帧。 */
extern uint8_t OLED_GRAM[OLED_WIDTH][OLED_HEIGHT / 8U];

void OLED_Init(void);
void OLED_ClearBuffer(void);
void OLED_Clear(void);
void OLED_Refresh(void);
void OLED_ShowChar(uint8_t x, uint8_t y, char character);
void OLED_ShowString(uint8_t x, uint8_t y, const char *text);
void OLED_ShowUnsigned(uint8_t x, uint8_t y, uint32_t value, uint8_t digits);

#endif
