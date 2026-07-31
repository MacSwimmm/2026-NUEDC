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
/*
 * A 点横线宽度判定使用“连续相邻黑线数”，不能只统计总黑色通道数：
 * - 连续 4 路及以上是强标志，对应车辆较居中地通过 A 点；
 * - 连续 3 路是弱标志，用于容忍高速回到 A 点时车辆存在横向偏差。
 * 两类标志使用不同的连续确认时间，最终停车仍由比赛层结合最低运行时间判定。
 */
#define TRACK_MARKER_WEAK_MIN_CONTIGUOUS   3U
#define TRACK_MARKER_STRONG_MIN_CONTIGUOUS 4U

/* 超过该时间没有收到有效 JY901S 帧，就把本地快照标记为离线。 */
#define JY901S_OFFLINE_TIMEOUT_MS           500U

#endif
