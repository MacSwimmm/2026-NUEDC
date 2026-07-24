#ifndef G3507_BASE_APP_CONFIG_H
#define G3507_BASE_APP_CONFIG_H

/* 裸机调度与底盘闭环周期。 */
#define APP_CONTROL_PERIOD_MS              5U
#define APP_CONTROL_FREQUENCY_HZ           200.0f

/* 车体安装方向；只改变软件正方向，不改变 BSP 的硬件 PWM 周期。 */
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

/* 左右轮各自使用一套增量式速度 PI。 */
#define SPEED_PI_KP                        400.0f
#define SPEED_PI_KI                        400.0f

/*
 * 八路循迹参数。
 * 统一约定：正误差表示黑线位于车体左侧，控制器应让车辆向左修正。
 */
#define TRACK_BASE_SPEED_MPS               0.18f
#define TRACK_BASE_SPEED_STEP_MPS          0.02f
#define TRACK_BASE_SPEED_MIN_MPS           0.08f
#define TRACK_BASE_SPEED_MAX_MPS           0.30f
#define TRACK_STEER_KP_MPS                 0.0022f
#define TRACK_STEER_KD_MPS                 0.0008f
#define TRACK_TARGET_LIMIT_MPS             0.35f

#endif
