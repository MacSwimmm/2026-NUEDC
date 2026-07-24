#ifndef G3507_BASE_APP_CONFIG_H
#define G3507_BASE_APP_CONFIG_H

/**
 * @file app_config.h
 * @brief 与整车机械结构和控制效果相关的应用层参数。
 *
 * 本文件只放“换车或调车时可能修改”的参数；GPIO、定时器周期和传感器
 * 有效电平等硬件事实放在 BSP/bsp_config.h 或 SysConfig 中。
 */

/*
 * 裸机调度与底盘闭环周期。
 * TIMG0 每 1 ms 进入一次中断，Board 层每累计 5 次调用一次 Car_ControlStep5ms。
 * 频率宏参与编码器计数到 m/s 的换算，两者必须保持严格互为倒数。
 */
#define APP_CONTROL_PERIOD_MS              5U
#define APP_CONTROL_FREQUENCY_HZ           200.0f

/*
 * 车体安装方向补偿。
 * 这里只改变“正目标速度对应的电机输出符号”，不改变 TB6612 接线和 PWM 周期。
 * 当前符号沿用已跑通底盘；若实车前进方向错误，应先核对接线再修改这里。
 */
#define LEFT_MOTOR_OUTPUT_SIGN             (-1)
#define RIGHT_MOTOR_OUTPUT_SIGN            (1)

/*
 * 编码器与机械参数。
 * 当前值沿用参考车：电机轴 13 线、28:1 减速、A/B 两相上升沿 2 倍频。
 * 更换电机或轮胎后，必须按铭牌和实测值修改，不能只调 PID 掩盖比例错误。
 */
#define ENCODER_COUNT_MULTIPLIER           2.0f
#define ENCODER_MOTOR_SHAFT_PPR            13.0f
#define MOTOR_GEAR_RATIO                   28.0f
#define WHEEL_CIRCUMFERENCE_M              0.204203519f
#define LEFT_ENCODER_SIGN                  (1)
#define RIGHT_ENCODER_SIGN                 (-1)

/*
 * 左右轮各自使用一套增量式速度 PI，参数沿用已跑通 Arc_Track 基线。
 * PI 输入为 m/s、输出为 PWM 计数，轮径或编码器比例未校准前不要盲目调大参数。
 */
#define SPEED_PI_KP                        400.0f
#define SPEED_PI_KI                        400.0f

/*
 * 八路循迹参数。
 * 统一约定：正误差表示黑线位于车体左侧，控制器应让车辆向左修正。
 * 基础速度按 PB1 在最小值到最大值之间循环；单轮目标还会受 LIMIT 二次限制。
 */
#define TRACK_BASE_SPEED_MPS               0.18f
#define TRACK_BASE_SPEED_STEP_MPS          0.02f
#define TRACK_BASE_SPEED_MIN_MPS           0.08f
#define TRACK_BASE_SPEED_MAX_MPS           0.30f
#define TRACK_STEER_KP_MPS                 0.0022f
#define TRACK_STEER_KD_MPS                 0.0008f
#define TRACK_TARGET_LIMIT_MPS             0.35f

#endif
