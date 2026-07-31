#ifndef G3507_BASE_PID_TUNER_H
#define G3507_BASE_PID_TUNER_H

/**
 * @file PID_Tuner.h
 * @brief 通过板载 CH340/UART0 进行轮速 PI 遥测和受限在线调参。
 *
 * 本模块只编排已有 Serial、Car_Control 和 PID Watch 接口，不拥有编码器、
 * PWM 或控制器状态。所有运动命令均受 Car_Control 的目标速度与 3200 PWM
 * 硬限幅保护，且上位机超时后自动停车。
 */

/** 初始化协议状态并保持停车；应在 Car_ControlInit 和 Board_Init 后调用。 */
void PIDTuner_Init(void);

/** 在主循环中高频调用：轮询 UART0 命令、执行超时停车并按需发送遥测。 */
void PIDTuner_Process(void);

#endif
