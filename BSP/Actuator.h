#ifndef G3507_BASE_ACTUATOR_H
#define G3507_BASE_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file Actuator.h
 * @brief 竞赛扩展执行器的纯底层输出接口。
 *
 * 本模块不包含动作序列或状态机，只负责电磁铁 PWM、两路机构 PWM、蜂鸣器和
 * 通用使能引脚。所有输出上电均为关闭，后续由具体 APP 按题目需要调用。
 */

/** 将所有扩展执行器恢复为关闭/零占空比。 */
void Actuator_Init(void);

/** 设置 PA2 功率控制 PWM，占空比单位为千分比 0~1000。 */
void Actuator_SetMagnetPermille(uint16_t duty_permille);

/** 设置 PB26/PB27 两路机构 PWM 原始比较计数，并分别限幅到周期值。 */
void Actuator_SetMechanismPWM(uint16_t channel0, uint16_t channel1);

/** 控制 PB23 有源蜂鸣器，true 为开启。 */
void Actuator_SetBuzzer(bool enabled);

/** 控制 PA1 通用使能输出，true 为高电平。 */
void Actuator_SetGeneralEnable(bool enabled);

#endif
