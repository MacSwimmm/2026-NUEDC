#ifndef G3507_BASE_SERVO_H
#define G3507_BASE_SERVO_H

#include <stdint.h>

/**
 * @file Servo.h
 * @brief 两路 50 Hz 舵机 PWM 输出接口。
 *
 * 通道 1/2 分别对应 TIMG8_C0/PB10 与 TIMG8_C1/PB11。定时器计数频率为
 * 1 MHz，因此比较值数值与高电平微秒数相同；接口会限制到 BSP 安全范围。
 */

/** 将两路舵机置于 1500 us 中位。 */
void Servo_Init(void);

/** 设置指定通道的高电平脉宽，单位 us；非法通道不执行写入。 */
void Servo_SetPulseUS(uint8_t channel, uint16_t pulse_us);

/** 将 0~180° 线性映射到安全脉宽范围后输出。 */
void Servo_SetAngle(uint8_t channel, uint8_t angle_deg);

/** 返回最近一次写入的脉宽，非法通道返回 0。 */
uint16_t Servo_GetPulseUS(uint8_t channel);

#endif
