#ifndef G3507_BASE_ACTUATOR_H
#define G3507_BASE_ACTUATOR_H

#include <stdbool.h>
#include <stdint.h>

/* 扩展执行器仅提供底层输出接口，上电初始化全部为关闭状态。 */
void Actuator_Init(void);
void Actuator_SetMagnetPermille(uint16_t duty_permille);
void Actuator_SetMechanismPWM(uint16_t channel0, uint16_t channel1);
void Actuator_SetBuzzer(bool enabled);
void Actuator_SetGeneralEnable(bool enabled);

#endif
