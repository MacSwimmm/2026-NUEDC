#include "Car_Control.h"

#include "Actuator.h"
#include "Board.h"
#include "Encoder.h"
#include "JY901S.h"
#include "Key.h"
#include "LineTrack.h"
#include "Motor.h"
#include "PID.h"
#include "Servo.h"
#include "TrackSensor.h"
#include "app_config.h"
#include "bsp_config.h"

/*
 * 本文件是 APP 层的控制编排中心：把循迹采样、编码器测速、差速算法、
 * 双轮 PI 和 TB6612 输出按固定顺序串联。Car_ControlStep5ms() 运行在
 * TIMG0 派生的 5 ms 中断上下文，其余公开接口主要由主循环调用。
 */

/* 左右轮必须各自保存独立的 PI 累计量，不能共用一个控制器实例。 */
static IncrementalPI_t g_left_speed_pi;
static IncrementalPI_t g_right_speed_pi;
static LineTrack_t g_line_controller;

/* 下列状态会被中断与主循环共同访问，读写时必须使用 Board 临界区接口。 */
static volatile bool g_armed = false;
static volatile CarTelemetry_t g_telemetry;

/**
 * @brief 将一个 5 ms 周期内的编码器增量换算为轮缘线速度。
 * @param count          本周期 A/B 两相上升沿累计计数，可正可负
 * @param direction_sign 安装方向补偿，只允许传入 +1 或 -1
 * @return 车轮线速度，单位 m/s
 *
 * 每个车轮输出轴一圈的计数为：倍频数 × 电机轴线数 × 减速比。
 * count×控制频率得到每秒计数，再除以每圈计数并乘轮周长即可得到 m/s。
 */
static float CountToSpeed(int32_t count, int32_t direction_sign)
{
    float pulses_per_wheel_turn = ENCODER_COUNT_MULTIPLIER *
                                  ENCODER_MOTOR_SHAFT_PPR *
                                  MOTOR_GEAR_RATIO;

    return (float)(count * direction_sign) * APP_CONTROL_FREQUENCY_HZ *
           WHEEL_CIRCUMFERENCE_M / pulses_per_wheel_turn;
}

/**
 * @brief 清除所有会跨周期累积的闭环状态并立即写零 PWM。
 * @note 必须在停车、丢线和重新使能时调用，防止旧 PI 输出导致突然起步。
 */
static void ResetClosedLoop(void)
{
    PID_IncrementalReset(&g_left_speed_pi);
    PID_IncrementalReset(&g_right_speed_pi);
    LineTrack_Reset(&g_line_controller);
    Motor_Stop();
}

static void UpdateStatusLED(uint8_t track_ready)
{
    static uint16_t blink_ticks = 0U;

    /*
     * LED 状态约定：
     * - 常亮：整车已使能；
     * - 500 ms 翻转：未使能，但循迹输入已经满足启动条件；
     * - 100 ms 翻转：循迹输入未就绪，优先检查接线、极性或传感器位置。
     * 本函数每 5 ms 调用，100/20 个 tick 分别对应 500/100 ms。
     */
    if (g_armed) {
        blink_ticks = 0U;
        Board_LEDSet(true);
        return;
    }

    blink_ticks++;
    if (track_ready != 0U) {
        /* 未使能但传感器正常：500 ms 翻转一次。 */
        if (blink_ticks >= 100U) {
            blink_ticks = 0U;
            Board_LEDToggle();
        }
    } else if (blink_ticks >= 20U) {
        /* 传感器尚未就绪：100 ms 翻转一次，提示优先检查接线和极性。 */
        blink_ticks = 0U;
        Board_LEDToggle();
    }
}

void Car_ControlInit(void)
{
    /*
     * 先初始化所有 BSP，使电机和扩展执行器进入确定的关闭状态；
     * 再初始化算法参数，最后统一保持未使能。此时 NVIC 尚未开放，不存在并发写入。
     */
    Motor_Init();
    Encoder_Init();
    TrackSensor_Init();
    Key_Init();
    JY901S_Init();
    Servo_Init();
    Actuator_Init();

    PID_IncrementalInit(&g_left_speed_pi, SPEED_PI_KP, SPEED_PI_KI,
                        (float)MOTOR_PWM_LIMIT_COUNTS);
    PID_IncrementalInit(&g_right_speed_pi, SPEED_PI_KP, SPEED_PI_KI,
                        (float)MOTOR_PWM_LIMIT_COUNTS);
    LineTrack_Init(&g_line_controller, TRACK_BASE_SPEED_MPS,
                   TRACK_STEER_KP_MPS, TRACK_STEER_KD_MPS,
                   TRACK_TARGET_LIMIT_MPS);

    g_armed = false;
    g_telemetry.arm_reject_count = 0U;
    g_telemetry.control_tick_count = 0U;
    ResetClosedLoop();
}

bool Car_ControlSetArmed(bool armed)
{
    TrackSensorData_t track;
    uint32_t primask;

    /*
     * 该函数运行在主循环，而速度 PI 和电机输出运行在 5 ms 中断中。
     * 使能状态、PI 历史和 PWM 必须作为一个整体切换，避免中断恰好插入后
     * 出现“状态已停车但旧 PI 又写回一次 PWM”的竞态。
     */
    primask = Board_EnterCritical();
    if (armed) {
        /* 只有连续获得正常循迹图案后才允许起步，避免上电悬空时误动作。 */
        TrackSensor_GetData(&track);
        if (track.ready == 0U) {
            g_telemetry.arm_reject_count++;
            Board_ExitCritical(primask);
            return false;
        }
        /* 从零积分、零 PWM 状态开始闭环，使第一次输出只由当前误差决定。 */
        ResetClosedLoop();
        g_armed = true;
        Board_ExitCritical(primask);
        return true;
    }

    /* 停车路径不依赖传感器状态，任何时刻都必须能够立即执行。 */
    g_armed = false;
    ResetClosedLoop();
    Board_ExitCritical(primask);
    return true;
}

bool Car_ControlIsArmed(void)
{
    /* bool 在 Cortex-M0+ 上可原子读取；写入仍由 SetArmed 的临界区统一完成。 */
    return g_armed;
}

void Car_ControlCycleBaseSpeed(void)
{
    uint32_t primask;
    float speed;

    /* base_speed_mps 同时会被 5 ms 控制中断读取，更新时短暂进入临界区。 */
    primask = Board_EnterCritical();
    speed = g_line_controller.base_speed_mps + TRACK_BASE_SPEED_STEP_MPS;
    /* 最大档之后回到最小档，按键每次只产生一个事件，不会因长按快速连跳。 */
    if (speed > TRACK_BASE_SPEED_MAX_MPS) speed = TRACK_BASE_SPEED_MIN_MPS;
    LineTrack_SetBaseSpeed(&g_line_controller, speed);
    Board_ExitCritical(primask);
}

/**
 * @brief 固定 5 ms 执行一次的整车控制步骤。
 *
 * 顺序保持简单且固定：采循迹 → 取编码器 → 换算轮速 → 计算目标 → 两轮 PI → PWM。
 * 工程没有弯道/路口状态机；未使能或完全丢线时立即清 PI 并写零 PWM。
 *
 * 本函数位于中断上下文，不能进行 printf、OLED 整帧刷新或任何无界阻塞操作。
 * 遥测只写入 RAM 快照，显示和日志由主循环以较低频率读取。
 */
void Car_ControlStep5ms(void)
{
    TrackSensorData_t track;
    int32_t left_count;
    int32_t right_count;
    float left_speed;
    float right_speed;
    float left_target = 0.0f;
    float right_target = 0.0f;
    int32_t left_pwm = 0;
    int32_t right_pwm = 0;
    bool line_valid;

    /* 先采集本周期输入，保证后续算法使用同一时间窗口的数据。 */
    Key_Scan5ms();
    TrackSensor_Sample();
    TrackSensor_GetData(&track);
    Encoder_ReadAndReset(&left_count, &right_count);

    /* 编码器方向补偿在测速阶段完成，之后统一约定正速度表示车辆前进。 */
    left_speed = CountToSpeed(left_count, LEFT_ENCODER_SIGN);
    right_speed = CountToSpeed(right_count, RIGHT_ENCODER_SIGN);
    line_valid = (track.active_count != 0U);

    if (g_armed && line_valid) {
        /* 循迹 PD 只产生速度目标，最终执行量仍由左右轮各自的速度 PI 计算。 */
        LineTrack_Calculate(&g_line_controller, track.error, true,
                            &left_target, &right_target);
        left_pwm = (int32_t)PID_IncrementalCalculate(
            &g_left_speed_pi, left_target, left_speed);
        right_pwm = (int32_t)PID_IncrementalCalculate(
            &g_right_speed_pi, right_target, right_speed);
        Motor_SetPWM(left_pwm * LEFT_MOTOR_OUTPUT_SIGN,
                     right_pwm * RIGHT_MOTOR_OUTPUT_SIGN);
    } else {
        /* 未使能或丢线均走同一安全停车路径，不继续沿用上一周期目标。 */
        ResetClosedLoop();
    }

    /*
     * 控制计算结束后一次性更新遥测字段。读取侧通过临界区复制，因而不会看到
     * “一半是旧周期、一半是新周期”的混合快照。
     */
    g_telemetry.raw_track_mask = track.raw_mask;
    g_telemetry.track_mask = track.line_mask;
    g_telemetry.track_ready = track.ready;
    g_telemetry.junction = track.junction;
    g_telemetry.armed = g_armed ? 1U : 0U;
    g_telemetry.track_error = track.error;
    g_telemetry.base_speed_mps = g_line_controller.base_speed_mps;
    g_telemetry.left_target_mps = left_target;
    g_telemetry.right_target_mps = right_target;
    g_telemetry.left_speed_mps = left_speed;
    g_telemetry.right_speed_mps = right_speed;
    g_telemetry.left_pwm = Motor_GetLeftPWM();
    g_telemetry.right_pwm = Motor_GetRightPWM();
    g_telemetry.control_tick_count++;

    UpdateStatusLED(track.ready);
}

void Car_ControlGetTelemetry(CarTelemetry_t *telemetry)
{
    uint32_t primask = Board_EnterCritical();

    /* 字段逐项复制便于接口演进，也避免把 volatile 结构直接强制转换复制。 */
    telemetry->raw_track_mask = g_telemetry.raw_track_mask;
    telemetry->track_mask = g_telemetry.track_mask;
    telemetry->track_ready = g_telemetry.track_ready;
    telemetry->junction = g_telemetry.junction;
    telemetry->armed = g_telemetry.armed;
    telemetry->track_error = g_telemetry.track_error;
    telemetry->base_speed_mps = g_telemetry.base_speed_mps;
    telemetry->left_target_mps = g_telemetry.left_target_mps;
    telemetry->right_target_mps = g_telemetry.right_target_mps;
    telemetry->left_speed_mps = g_telemetry.left_speed_mps;
    telemetry->right_speed_mps = g_telemetry.right_speed_mps;
    telemetry->left_pwm = g_telemetry.left_pwm;
    telemetry->right_pwm = g_telemetry.right_pwm;
    telemetry->arm_reject_count = g_telemetry.arm_reject_count;
    telemetry->control_tick_count = g_telemetry.control_tick_count;
    Board_ExitCritical(primask);
}
