#ifndef G3507_BASE_BUZZER_H
#define G3507_BASE_BUZZER_H

#include <stdbool.h>

/**
 * @file Buzzer.h
 * @brief PB23 有源蜂鸣器的独立 GPIO 输出接口。
 *
 * 当前硬件约定为高电平开启、低电平关闭。本模块只负责稳定电平控制，
 * 不包含提示音节拍或阻塞延时；提示序列应由 APP 层组织。
 */

/** 将蜂鸣器初始化为关闭状态。 */
void Buzzer_Init(void);

/** 设置蜂鸣器开关状态，true 为开启。 */
void Buzzer_Set(bool enabled);

/** 开启蜂鸣器。 */
void Buzzer_On(void);

/** 关闭蜂鸣器。 */
void Buzzer_Off(void);

/** 返回最近一次通过本模块设置的开关状态。 */
bool Buzzer_IsOn(void);

#endif
