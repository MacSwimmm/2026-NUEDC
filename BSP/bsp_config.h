#ifndef G3507_BASE_BSP_CONFIG_H
#define G3507_BASE_BSP_CONFIG_H

/**
 * @file bsp_config.h
 * @brief 不由 SysConfig 自动表达的板级常量集中配置。
 *
 * 这里保存与定时器、传感器电平和执行器脉宽直接相关的参数；
 * 车辆速度、机械尺寸和控制器系数仍放在 APP/app_config.h。
 * 修改本文件前应先确认对应 SysConfig 周期/时钟没有变化。
 */

/* TIMA1 双路电机 PWM：80 MHz / 8000 = 10 kHz，并保留 200 计数余量。 */
#define MOTOR_PWM_PERIOD_COUNTS            8000
#define MOTOR_PWM_LIMIT_COUNTS             7800

/* 辅助板数字输出为白色=1、黑色=0，八路均按低电平检测到黑线。 */
#define TRACK_ACTIVE_LOW_MASK              0xFFU
#define TRACK_READY_SAMPLES                4U

/* TIMG8 为 1 us/计数，舵机安全脉宽范围 1000~2000 us。 */
#define SERVO_MIN_PULSE_US                 1000U
#define SERVO_NEUTRAL_PULSE_US             1500U
#define SERVO_MAX_PULSE_US                 2000U

/* 超过该时间没有收到有效 JY901S 帧，就把本地快照标记为离线。 */
#define JY901S_OFFLINE_TIMEOUT_MS           500U

#endif
