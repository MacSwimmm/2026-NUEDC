#ifndef G3507_BASE_BOARD_H
#define G3507_BASE_BOARD_H

#include <stdbool.h>
#include <stdint.h>

/* 每 5 ms 回调一次 APP 控制函数，使 Board BSP 不反向包含应用层头文件。 */
typedef void (*BoardControlCallback_t)(void);

/* Board_Init 只开放 NVIC；调用前必须先完成 SysConfig 和各 BSP 初始化。 */
void Board_Init(BoardControlCallback_t control_callback, uint8_t control_period_ms);
uint32_t Board_Millis(void);
void Board_DelayUs(uint32_t us);
void Board_DelayMs(uint32_t ms);
void Board_LEDSet(bool enabled);
void Board_LEDToggle(void);
uint32_t Board_EnterCritical(void);
void Board_ExitCritical(uint32_t previous_primask);

#endif
