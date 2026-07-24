#ifndef G3507_BASE_SERVO_H
#define G3507_BASE_SERVO_H

#include <stdint.h>

/* 通道号固定为 1/2；脉宽接口会限幅到 app_config.h 规定的安全范围。 */
void Servo_Init(void);
void Servo_SetPulseUS(uint8_t channel, uint16_t pulse_us);
void Servo_SetAngle(uint8_t channel, uint8_t angle_deg);
uint16_t Servo_GetPulseUS(uint8_t channel);

#endif
