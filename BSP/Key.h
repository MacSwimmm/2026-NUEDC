#ifndef G3507_BASE_KEY_H
#define G3507_BASE_KEY_H

#include <stdint.h>

/**
 * @file Key.h
 * @brief 三个低有效按键的 5 ms 消抖与一次性事件接口。
 *
 * 按键扫描在控制中断中执行，应用主循环通过 Key_TakeEvents() 原子取走事件。
 * 长按只产生一次按下事件，不实现连发。
 */

#define KEY_EVENT_USER   (1U << 0) /* PB21：保持当前参数完成一圈；运行中再按中止 */
#define KEY_EVENT_KEY1   (1U << 1) /* PB0：3.50 倍速度档缓启动完成一圈 */
#define KEY_EVENT_KEY2   (1U << 2) /* PB1：3.50 倍直线循迹 6 s + 2 s 缓停 */

/** 清空三个按键的稳定状态、候选状态和待处理事件。 */
void Key_Init(void);

/** 每 5 ms 调用一次，执行低有效采样、20 ms 消抖和按下沿事件生成。 */
void Key_Scan5ms(void);

/** 原子返回当前事件位图并清零，保证每个事件只被应用处理一次。 */
uint8_t Key_TakeEvents(void);

#endif
