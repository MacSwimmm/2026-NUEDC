#ifndef G3507_BASE_KEY_H
#define G3507_BASE_KEY_H

#include <stdint.h>

#define KEY_EVENT_USER   (1U << 0) /* 板载 PB21：整车使能/停车 */
#define KEY_EVENT_KEY1   (1U << 1) /* 扩展 PB0：切换显示页面 */
#define KEY_EVENT_KEY2   (1U << 2) /* 扩展 PB1：循环调整基础速度 */

void Key_Init(void);
void Key_Scan5ms(void);
uint8_t Key_TakeEvents(void);

#endif
