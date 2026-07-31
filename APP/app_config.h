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
#define RIGHT_MOTOR_OUTPUT_SIGN            (-1)

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
#define SPEED_PI_KP                        150.0f
#define SPEED_PI_KI                        300.0f
/*
 * 30 s 三圈遥测中左轮约 32% 时间顶在 2000，说明原上限已限制实际提速。
 * 首版冲击 20 s 将安全上限放宽到 3200（TIMA1 周期 8000 的 40%）；Motor BSP
 * 仍保留 7800 最终保护，Watch/PID 调参协议只能把限幅调低，不能突破这里。
 */
#define SPEED_PI_OUTPUT_LIMIT_COUNTS       3200.0f
/* SetSpeed_Left/Right 的目标速度绝对值上限。 */
#define WHEEL_SPEED_TARGET_LIMIT_MPS       0.45f

/*
 * GitHub 源工程 ti_mspm0g3507 的八路循迹参数，已从 RPM 换算为本工程的 m/s。
 * TRACK_FORWARD_SPEED_SCALE 只放大正常行驶的基础速度曲线和目标速度上限。
 * 直线小误差 P 和 D 保持稳定版原值；入弯、弯中按误差分段提高 P。
 * 本版本在不改变 PID 和有线速度公式的前提下，略微增强弯道修正与丢线恢复。
 * 源工程权重从左到右为 -350~-50、+50~+350，因此：
 * - 负误差表示黑线位于车体左侧，左轮应减速；
 * - 正误差表示黑线位于车体右侧，右轮应减速。
 *
 * 源控制周期为 10 ms，本工程为 5 ms。P、速度和限幅直接按轮周长换算；
 * 离散 D 项系数乘 2，使同一物理误差变化速率在 5 ms 周期下保持等效修正量。
 */
#define TRACK_RPM_TO_MPS                   (WHEEL_CIRCUMFERENCE_M / 60.0f)
/* PB21 保持当前 5.08 倍参数；PB0/PB1 使用互不影响的 3.50 倍速度档。 */
#define TRACK_FORWARD_SPEED_SCALE          5.08f
#define AUX_TRACK_FORWARD_SPEED_SCALE      3.50f
#define TRACK_BASE_SPEED_FOR_SCALE_MPS(scale) \
    (20.0f * (scale) * TRACK_RPM_TO_MPS)
#define TRACK_BASE_SPEED_MIN_FOR_SCALE_MPS(scale) \
    (12.0f * (scale) * TRACK_RPM_TO_MPS)
#define TRACK_BASE_SPEED_STEP_FOR_SCALE_MPS(scale) \
    (2.0f * (scale) * TRACK_RPM_TO_MPS)
#define TRACK_ERROR_SLOWDOWN_FOR_SCALE_MPS(scale) \
    (0.02f * (scale) * TRACK_RPM_TO_MPS)
#define TRACK_TARGET_LIMIT_FOR_SCALE_MPS(scale) \
    (35.0f * (scale) * TRACK_RPM_TO_MPS)
#define TRACK_BASE_SPEED_MPS \
    TRACK_BASE_SPEED_FOR_SCALE_MPS(TRACK_FORWARD_SPEED_SCALE)
#define TRACK_BASE_SPEED_MIN_MPS \
    TRACK_BASE_SPEED_MIN_FOR_SCALE_MPS(TRACK_FORWARD_SPEED_SCALE)
#define TRACK_BASE_SPEED_MAX_MPS           TRACK_BASE_SPEED_MPS
#define TRACK_BASE_SPEED_STEP_MPS \
    TRACK_BASE_SPEED_STEP_FOR_SCALE_MPS(TRACK_FORWARD_SPEED_SCALE)
#define TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR \
    TRACK_ERROR_SLOWDOWN_FOR_SCALE_MPS(TRACK_FORWARD_SPEED_SCALE)
#define TRACK_STEER_KP_MPS                 (0.035f * TRACK_RPM_TO_MPS)
#define TRACK_STEER_ENTRY_KP_MPS           (0.100f * TRACK_RPM_TO_MPS)
#define TRACK_STEER_CURVE_KP_MPS           (0.150f * TRACK_RPM_TO_MPS)
#define TRACK_STEER_ENTRY_ERROR_ABS        100.0f
#define TRACK_STEER_CURVE_ERROR_ABS        200.0f
#define TRACK_STEER_KD_MPS                 (0.020f * TRACK_RPM_TO_MPS)
#define TRACK_CORRECTION_LIMIT_MPS         (20.0f * TRACK_RPM_TO_MPS)
#define TRACK_TARGET_LIMIT_MPS \
    TRACK_TARGET_LIMIT_FOR_SCALE_MPS(TRACK_FORWARD_SPEED_SCALE)
#define AUX_TRACK_BASE_SPEED_MPS \
    TRACK_BASE_SPEED_FOR_SCALE_MPS(AUX_TRACK_FORWARD_SPEED_SCALE)
#define AUX_TRACK_BASE_SPEED_MIN_MPS \
    TRACK_BASE_SPEED_MIN_FOR_SCALE_MPS(AUX_TRACK_FORWARD_SPEED_SCALE)
#define AUX_TRACK_BASE_SPEED_STEP_MPS \
    TRACK_BASE_SPEED_STEP_FOR_SCALE_MPS(AUX_TRACK_FORWARD_SPEED_SCALE)
#define AUX_TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR \
    TRACK_ERROR_SLOWDOWN_FOR_SCALE_MPS(AUX_TRACK_FORWARD_SPEED_SCALE)
#define AUX_TRACK_TARGET_LIMIT_MPS \
    TRACK_TARGET_LIMIT_FOR_SCALE_MPS(AUX_TRACK_FORWARD_SPEED_SCALE)
#define TRACK_RECOVERY_KICK_SPEED_MPS      (24.0f * TRACK_RPM_TO_MPS)
#define TRACK_RECOVERY_SPEED_MPS           (12.0f * TRACK_RPM_TO_MPS)
#define TRACK_RECOVERY_KICK_MS              60U
#define TRACK_LOST_PROTECT_MS              400U
#define TRACK_DIRECTION_DEADBAND           25.0f

/*
 * 赛题第 2 项流程仍负责 PB21 启动、完整一圈、A 点停车和时间冻结。
 * 车速按移植后的源公式在 60~100 RPM 内连续计算；巡航和接近请求均设为
 * 源算法上限。本版本不增加额外直线/弯道速度状态。
 */
#define RACE_CRUISE_SPEED_MPS              TRACK_BASE_SPEED_MPS
#define RACE_APPROACH_SPEED_MPS            TRACK_BASE_SPEED_MPS
#define RACE_APPROACH_START_MS             16500U
#define RACE_FINISH_MIN_TIME_MS            8000U
/* A 点只接受连续至少四路黑的强标志，30 ms 确认用于过滤边缘短毛刺。 */
#define RACE_MARKER_STRONG_CONFIRM_SAMPLES 6U
#define RACE_MARKER_CLEAR_SAMPLES          3U

/*
 * 扩展按键运行表。PB0 用 4 s 将整组循迹轮目标从 0 平滑放大到 3.50 倍档，
 * 随后仍按与 PB21 相同的“离开 A、再次到达 A”条件完成一圈。PB1 也使用
 * 3.50 倍循迹档：前 1.5 s 缓启动，第 6 s 开始减速，2 s 后目标归零并停车。
 */
#define PB0_LAP_LAUNCH_RAMP_MS             4000U
#define PB1_STRAIGHT_LAUNCH_RAMP_MS        1500U
#define PB1_STRAIGHT_RUN_MS                6000U
#define PB1_STRAIGHT_DECEL_MS              2000U
#define PB1_STRAIGHT_TOTAL_MS \
    (PB1_STRAIGHT_RUN_MS + PB1_STRAIGHT_DECEL_MS)

/*
 * UART3/PB12 无线 DAP 遥测周期。10 ms 对应 100 Hz，足以观察 5 ms 控制环中
 * 相邻两拍的总体趋势；固定 74 字节帧在 115200 8N1 下约占 64% 线速，
 * 并由最低优先级 TX 中断后台发送，不在控制 ISR 内执行串口等待。
 */
#define TELEMETRY_STREAM_PERIOD_MS          10U

#endif
