#ifndef G3507_BASE_BSP_CONFIG_H
#define G3507_BASE_BSP_CONFIG_H

/*
 * BSP 硬件参数集中配置。
 *
 * 这里保存与定时器、传感器电平、分压网络和执行器脉宽直接相关的参数；
 * 车辆速度、机械尺寸和控制器系数仍放在 APP/app_config.h。
 */

/* TIMA1 双路电机 PWM：80 MHz / 8000 = 10 kHz，并保留 200 计数余量。 */
#define MOTOR_PWM_PERIOD_COUNTS            8000
#define MOTOR_PWM_LIMIT_COUNTS             7800

/* 八路循迹模块电平极性，以及允许整车使能前所需的连续正常采样次数。 */
#define TRACK_ACTIVE_LOW_MASK              0x00U
#define TRACK_READY_SAMPLES                4U

/* PA15 电池采样：3.3 V 参考、12 位 ADC、10:1 外部分压的总恢复倍率为 11。 */
#define BATTERY_ADC_REFERENCE_V            3.3f
#define BATTERY_ADC_FULL_SCALE             4096.0f
#define BATTERY_DIVIDER_RATIO              11.0f

/* TIMG8 为 1 us/计数，舵机安全脉宽范围 1000~2000 us。 */
#define SERVO_MIN_PULSE_US                 1000U
#define SERVO_NEUTRAL_PULSE_US             1500U
#define SERVO_MAX_PULSE_US                 2000U

/* 超过该时间没有收到有效 JY901S 帧，就把本地快照标记为离线。 */
#define JY901S_OFFLINE_TIMEOUT_MS           500U

#endif
