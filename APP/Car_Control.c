#include "Car_Control.h"

#include "Board.h"
#include "Encoder.h"
#include "JY901S.h"
#include "Key.h"
#include "LineTrack.h"
#include "Motor.h"
#include "PID.h"
#include "TrackSensor.h"
#include "app_config.h"

/*
 * 本文件是 APP 层的底盘控制编排中心。固定 5 ms 控制链如下：
 *
 * 八路循迹采样 -> 源工程动态基础速度/差速 PD/丢线原地搜线
 *              -> 两套独立速度 PI -> TB6612 PWM
 *
 * Car_ControlStep5ms() 运行在 TIMG0 派生的中断上下文；主循环只能通过
 * Car_ControlSetArmed()、Car_ControlSetBaseSpeed() 和遥测快照与其交互。
 * 这样比赛流程不直接操作编码器、PI 累计量或 PWM 寄存器，停车时序只有一条。
 */

/* 左右轮必须各自保存独立的 PI 累计量，不能共用一个控制器实例。 */
static IncrementalPI_t g_left_speed_pi;
static IncrementalPI_t g_right_speed_pi;
static LineTrack_t g_line_controller;

/*
 * 以下变量会被 5 ms 中断和主循环共同访问。主循环写入时必须使用 Board
 * 临界区；ISR 内部无需再次关中断。
 */
static volatile bool g_armed = false;
static volatile bool g_manual_speed_mode = false;
static volatile float g_manual_left_target_mps = 0.0f;
static volatile float g_manual_right_target_mps = 0.0f;
static volatile float g_requested_base_speed_mps = TRACK_BASE_SPEED_MPS;
static volatile float g_tracking_output_scale = 1.0f;
static float g_base_speed_step_mps = TRACK_BASE_SPEED_STEP_MPS;
static volatile CarTelemetry_t g_telemetry;

/*
 * 对外可见的 Keil Watch 只读镜像。真实控制状态仍保存在本文件的 static 变量中，
 * 调试器即使改写该结构，也会在下一次 5 ms 控制周期被真实值覆盖。
 */
volatile CarControlWatch_t g_car_watch;

/*
 * 下列状态只由 5 ms 控制路径推进；停车/重新使能时会在临界区统一复位。
 * dynamic_base 是源算法本周期按误差得到的基础速度，requested_base 只作为上限。
 */
static float g_dynamic_base_speed_mps = 0.0f;
static bool g_line_fault = false;

/** @brief 返回浮点数绝对值，避免为一个简单运算引入数学库依赖。 */
static float AbsFloat(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** @brief 把浮点值限制在闭区间内。 */
static float ClampFloat(float value, float lower, float upper)
{
    if (value > upper) return upper;
    if (value < lower) return lower;
    return value;
}

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

/** @brief 限制 Watch 可写的 PI 输出上限，绝不允许突破比赛固件的安全边界。 */
static float ClampPIDLimit(float limit)
{
    limit = AbsFloat(limit);
    if (limit > SPEED_PI_OUTPUT_LIMIT_COUNTS) {
        limit = SPEED_PI_OUTPUT_LIMIT_COUNTS;
    }
    return limit;
}

/** @brief 限制独立轮速调试接口的目标速度。 */
static float ClampWheelTarget(float target_mps)
{
    return ClampFloat(target_mps,
                      -WHEEL_SPEED_TARGET_LIMIT_MPS,
                      WHEEL_SPEED_TARGET_LIMIT_MPS);
}

/** @brief 限制比赛层请求的基础速度，防止参数或调试命令越过整车安全范围。 */
static float ClampBaseSpeed(float speed_mps)
{
    return ClampFloat(speed_mps,
                      g_line_controller.minimum_base_speed_mps,
                      g_line_controller.maximum_base_speed_mps);
}

/** @brief 在停车状态把源循迹的四个速度相关参数切换到指定档位。 */
static void ApplyTrackingSpeedProfile(CarTrackSpeedProfile_t profile)
{
    float maximum_base_speed_mps;
    float minimum_base_speed_mps;
    float error_slowdown_mps_per_error;
    float target_limit_mps;

    if (profile == CAR_TRACK_SPEED_PROFILE_AUX_3_50) {
        maximum_base_speed_mps = AUX_TRACK_BASE_SPEED_MPS;
        minimum_base_speed_mps = AUX_TRACK_BASE_SPEED_MIN_MPS;
        error_slowdown_mps_per_error =
            AUX_TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR;
        target_limit_mps = AUX_TRACK_TARGET_LIMIT_MPS;
        g_base_speed_step_mps = AUX_TRACK_BASE_SPEED_STEP_MPS;
    } else {
        maximum_base_speed_mps = TRACK_BASE_SPEED_MPS;
        minimum_base_speed_mps = TRACK_BASE_SPEED_MIN_MPS;
        error_slowdown_mps_per_error =
            TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR;
        target_limit_mps = TRACK_TARGET_LIMIT_MPS;
        g_base_speed_step_mps = TRACK_BASE_SPEED_STEP_MPS;
    }

    LineTrack_SetSpeedProfile(&g_line_controller,
                              maximum_base_speed_mps,
                              minimum_base_speed_mps,
                              error_slowdown_mps_per_error,
                              target_limit_mps);
    g_requested_base_speed_mps = maximum_base_speed_mps;
    g_telemetry.requested_base_speed_mps = maximum_base_speed_mps;
}

/**
 * @brief 清除两轮速度 PI 并立即写零 PWM。
 *
 * 本函数只清速度 PI 和电机输出，不清 LineTrack 的丢线计时与故障现场。
 * 因而 ISR 检测到 400 ms 丢线并停车后，主循环仍能读取完整故障快照。
 */
static void ResetMotionControllers(void)
{
    PID_IncrementalReset(&g_left_speed_pi);
    PID_IncrementalReset(&g_right_speed_pi);
    Motor_Stop();

    g_dynamic_base_speed_mps = 0.0f;

    g_pid_watch.left.error = 0.0f;
    g_pid_watch.left.output = 0.0f;
    g_pid_watch.right.error = 0.0f;
    g_pid_watch.right.output = 0.0f;
}

/** @brief 开始一次新运行前，清除上一圈的丢线和误差记忆。 */
static void ResetTrackingMemory(void)
{
    LineTrack_Reset(&g_line_controller);
    g_line_fault = false;
}

/** @brief 停车、切换控制来源或重新使能时使用的完整复位。 */
static void ResetClosedLoop(void)
{
    ResetMotionControllers();
    ResetTrackingMemory();
}

/**
 * @brief 把本周期最关键的循迹内部量复制到 Keil Watch 镜像。
 *
 * control_tick_count 最后写入：在 Watch 中看到该值变化时，前面的字段已经完成
 * 本周期刷新。该镜像只用于观察，Race_Control 和电机闭环都不会读取它。
 */
static void UpdateCarWatch(const TrackSensorData_t *track,
                           float steering_error_used,
                           float left_target_mps,
                           float right_target_mps,
                           float left_speed_mps,
                           float right_speed_mps,
                           int32_t left_count,
                           int32_t right_count)
{
    uint32_t line_lost_samples = g_line_controller.lost_time_ms /
                                 APP_CONTROL_PERIOD_MS;

    g_car_watch.raw_track_mask = track->raw_mask;
    g_car_watch.track_mask = track->line_mask;
    g_car_watch.track_ready = track->ready;
    g_car_watch.active_count = track->active_count;
    g_car_watch.marker = track->marker;
    g_car_watch.armed = g_armed ? 1U : 0U;
    g_car_watch.manual_speed_mode = g_manual_speed_mode ? 1U : 0U;
    g_car_watch.recovering = g_line_controller.recovering ? 1U : 0U;
    /* 旧三阶段字段保留帧布局兼容；移植后的源算法只有单阶段原地搜线。 */
    g_car_watch.recovery_phase = 0U;
    g_car_watch.line_fault = g_line_fault ? 1U : 0U;
    g_car_watch.line_lost_samples = (line_lost_samples > 0xFFU) ?
                                    0xFFU : (uint8_t)line_lost_samples;
    g_car_watch.center_confirm_samples = 0U;
    g_car_watch.track_phase = 0U;
    g_car_watch.curve_direction = g_line_controller.recovering ?
        g_line_controller.recovery_direction :
        g_line_controller.last_direction;
    g_car_watch.line_lost_ms = (uint16_t)g_line_controller.lost_time_ms;
    g_car_watch.raw_track_error = track->error;
    g_car_watch.steering_error_used = steering_error_used;
    /* 字段名为旧协议兼容名，实际值已是源算法未滤波误差和原始离散微分。 */
    g_car_watch.filtered_error = g_line_controller.last_error;
    g_car_watch.filtered_derivative = g_line_controller.derivative;
    g_car_watch.p_term_mps = g_line_controller.p_term_mps;
    g_car_watch.d_term_mps = g_line_controller.d_term_mps;
    g_car_watch.steering_correction_mps =
        g_line_controller.correction_mps;
    g_car_watch.phase_bias_mps = 0.0f;
    g_car_watch.requested_base_speed_mps = g_requested_base_speed_mps;
    g_car_watch.tracking_output_scale = g_tracking_output_scale;
    g_car_watch.desired_base_speed_mps =
        g_line_controller.base_speed_mps * g_tracking_output_scale;
    g_car_watch.dynamic_base_speed_mps = g_dynamic_base_speed_mps;
    g_car_watch.left_target_mps = left_target_mps;
    g_car_watch.right_target_mps = right_target_mps;
    g_car_watch.left_speed_mps = left_speed_mps;
    g_car_watch.right_speed_mps = right_speed_mps;
    g_car_watch.left_encoder_count = left_count;
    g_car_watch.right_encoder_count = right_count;
    g_car_watch.left_pwm = Motor_GetLeftPWM();
    g_car_watch.right_pwm = Motor_GetRightPWM();
    g_car_watch.control_tick_count = g_telemetry.control_tick_count;
}

/** @brief 根据当前使能和传感器就绪状态驱动板载 LED。 */
static void UpdateStatusLED(uint8_t track_ready)
{
    static uint16_t blink_ticks = 0U;

    /*
     * LED 状态约定：
     * - 常亮：底盘正在输出循迹或独立轮速闭环；
     * - 500 ms 翻转：停车，但循迹输入已经满足启动条件；
     * - 100 ms 翻转：循迹输入未就绪，优先检查接线、极性或探头位置。
     * 本函数每 5 ms 调用，100/20 个 tick 分别对应 500/100 ms。
     */
    if (g_armed || (g_manual_speed_mode &&
                    ((g_manual_left_target_mps != 0.0f) ||
                     (g_manual_right_target_mps != 0.0f)))) {
        blink_ticks = 0U;
        Board_LEDSet(true);
        return;
    }

    blink_ticks++;
    if (track_ready != 0U) {
        if (blink_ticks >= 100U) {
            blink_ticks = 0U;
            Board_LEDToggle();
        }
    } else if (blink_ticks >= 20U) {
        blink_ticks = 0U;
        Board_LEDToggle();
    }
}

void Car_ControlInit(void)
{
    /*
     * 先初始化底盘和传感器 BSP，使电机处于确定的零 PWM；再初始化算法对象。
     * JY901S/UART2 按用户要求保留，但本次比赛循迹控制不读取其姿态数据。
     */
    Motor_Init();
    Encoder_Init();
    TrackSensor_Init();
    Key_Init();
    JY901S_Init();

    PID_WatchInit(SPEED_PI_KP, SPEED_PI_KI,
                  SPEED_PI_OUTPUT_LIMIT_COUNTS);
    PID_IncrementalInit(&g_left_speed_pi, SPEED_PI_KP, SPEED_PI_KI,
                        SPEED_PI_OUTPUT_LIMIT_COUNTS);
    PID_IncrementalInit(&g_right_speed_pi, SPEED_PI_KP, SPEED_PI_KI,
                        SPEED_PI_OUTPUT_LIMIT_COUNTS);
    LineTrack_Init(&g_line_controller,
                   TRACK_BASE_SPEED_MPS,
                   TRACK_BASE_SPEED_MIN_MPS,
                   TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR,
                   TRACK_STEER_KP_MPS,
                   TRACK_STEER_ENTRY_KP_MPS,
                   TRACK_STEER_CURVE_KP_MPS,
                   TRACK_STEER_ENTRY_ERROR_ABS,
                   TRACK_STEER_CURVE_ERROR_ABS,
                   TRACK_STEER_KD_MPS,
                   TRACK_CORRECTION_LIMIT_MPS,
                   TRACK_TARGET_LIMIT_MPS,
                   TRACK_RECOVERY_KICK_SPEED_MPS,
                   TRACK_RECOVERY_SPEED_MPS,
                   TRACK_RECOVERY_KICK_MS,
                   TRACK_LOST_PROTECT_MS,
                   TRACK_DIRECTION_DEADBAND);

    g_armed = false;
    g_manual_speed_mode = false;
    g_manual_left_target_mps = 0.0f;
    g_manual_right_target_mps = 0.0f;
    g_requested_base_speed_mps = TRACK_BASE_SPEED_MPS;
    g_tracking_output_scale = 1.0f;
    g_base_speed_step_mps = TRACK_BASE_SPEED_STEP_MPS;

    /* 静态变量虽会在上电时清零，这里仍逐项初始化，便于软件复位路径审查。 */
    g_telemetry.raw_track_mask = 0U;
    g_telemetry.track_mask = 0U;
    g_telemetry.track_ready = 0U;
    g_telemetry.active_count = 0U;
    g_telemetry.marker = 0U;
    g_telemetry.junction = 0U;
    g_telemetry.armed = 0U;
    g_telemetry.manual_speed_mode = 0U;
    g_telemetry.line_fault = 0U;
    g_telemetry.line_lost_ms = 0U;
    g_telemetry.track_error = 0.0f;
    g_telemetry.requested_base_speed_mps = TRACK_BASE_SPEED_MPS;
    g_telemetry.tracking_output_scale = 1.0f;
    g_telemetry.base_speed_mps = 0.0f;
    g_telemetry.left_target_mps = 0.0f;
    g_telemetry.right_target_mps = 0.0f;
    g_telemetry.left_speed_mps = 0.0f;
    g_telemetry.right_speed_mps = 0.0f;
    g_telemetry.left_encoder_count = 0;
    g_telemetry.right_encoder_count = 0;
    g_telemetry.left_pwm = 0;
    g_telemetry.right_pwm = 0;
    g_telemetry.arm_reject_count = 0U;
    g_telemetry.control_tick_count = 0U;

    ResetClosedLoop();
}

void SetSpeed_Left(float target_speed_mps)
{
    uint32_t primask = Board_EnterCritical();

    if (!g_manual_speed_mode) {
        g_armed = false;
        g_manual_speed_mode = true;
        ResetClosedLoop();
    }
    g_manual_left_target_mps = ClampWheelTarget(target_speed_mps);
    PID_IncrementalReset(&g_left_speed_pi);
    Board_ExitCritical(primask);
}

void SetSpeed_Right(float target_speed_mps)
{
    uint32_t primask = Board_EnterCritical();

    if (!g_manual_speed_mode) {
        g_armed = false;
        g_manual_speed_mode = true;
        ResetClosedLoop();
    }
    g_manual_right_target_mps = ClampWheelTarget(target_speed_mps);
    PID_IncrementalReset(&g_right_speed_pi);
    Board_ExitCritical(primask);
}

void SetSpeed_Stop(void)
{
    uint32_t primask = Board_EnterCritical();

    g_armed = false;
    g_manual_speed_mode = false;
    g_manual_left_target_mps = 0.0f;
    g_manual_right_target_mps = 0.0f;
    ResetClosedLoop();
    Board_ExitCritical(primask);
}

bool Car_ControlSetArmed(bool armed)
{
    TrackSensorData_t track;
    uint32_t primask;

    /*
     * 该函数运行在主循环，而速度 PI 和电机输出运行在 5 ms 中断中。
     * 使能状态、PI 历史和 PWM 必须作为一个整体切换，避免中断插入后出现
     * “状态已经停车，但旧 PI 又写回一次 PWM”的竞态。
     */
    primask = Board_EnterCritical();
    g_armed = false;
    g_manual_speed_mode = false;
    g_manual_left_target_mps = 0.0f;
    g_manual_right_target_mps = 0.0f;
    ResetClosedLoop();

    if (armed) {
        /*
         * A 点横线属于合法启动图案，因此 ready 只拒绝全白/未稳定输入。
         * 比赛层负责“先离开 A，再次返回才完成”的横线时序。
         */
        TrackSensor_GetData(&track);
        if (track.ready == 0U) {
            g_telemetry.arm_reject_count++;
            Board_ExitCritical(primask);
            return false;
        }
        g_armed = true;
    }

    Board_ExitCritical(primask);
    return true;
}

bool Car_ControlIsArmed(void)
{
    /* bool 在 Cortex-M0+ 上可原子读取；写入仍由临界区或 ISR 统一完成。 */
    return g_armed;
}

void Car_ControlSetTrackingSpeedProfile(CarTrackSpeedProfile_t profile)
{
    uint32_t primask = Board_EnterCritical();

    /* 切档前先统一停车，避免旧 PI 或旧循迹历史跨速度档继续输出。 */
    g_armed = false;
    g_manual_speed_mode = false;
    g_manual_left_target_mps = 0.0f;
    g_manual_right_target_mps = 0.0f;
    ResetClosedLoop();
    ApplyTrackingSpeedProfile(profile);
    g_tracking_output_scale = 1.0f;
    g_telemetry.tracking_output_scale = 1.0f;
    Board_ExitCritical(primask);
}

void Car_ControlSetTrackingOutputScale(float scale)
{
    uint32_t primask = Board_EnterCritical();

    g_tracking_output_scale = ClampFloat(scale, 0.0f, 1.0f);
    g_telemetry.tracking_output_scale = g_tracking_output_scale;
    Board_ExitCritical(primask);
}

void Car_ControlSetBaseSpeed(float base_speed_mps)
{
    uint32_t primask = Board_EnterCritical();
    float limited_speed = ClampBaseSpeed(base_speed_mps);

    /*
     * 这里只修改源算法允许的基础速度上限；5 ms 控制中断仍按当前误差
     * 有线时仍按原公式在 60~100 RPM 内连续计算，不增加速度状态。
     */
    g_requested_base_speed_mps = limited_speed;
    LineTrack_SetBaseSpeed(&g_line_controller, limited_speed);
    g_telemetry.requested_base_speed_mps = limited_speed;
    Board_ExitCritical(primask);
}

void Car_ControlCycleBaseSpeed(void)
{
    uint32_t primask = Board_EnterCritical();
    float speed = g_requested_base_speed_mps + g_base_speed_step_mps;

    if (speed > g_line_controller.maximum_base_speed_mps) {
        speed = g_line_controller.minimum_base_speed_mps;
    }
    g_requested_base_speed_mps = speed;
    LineTrack_SetBaseSpeed(&g_line_controller, speed);
    g_telemetry.requested_base_speed_mps = speed;
    Board_ExitCritical(primask);
}

/**
 * @brief 固定 5 ms 执行一次的整车控制步骤。
 *
 * 所有非全白图案都按源工程的八路平均权重参与控制，A 点宽横线不再有独立
 * 转向分支；其对称图案自然得到 0 误差。全白后先向最后真实转向的反方向
 * 快速打一把，再沿反方向慢速搜线；400 ms 未重捕则清 PI、写零 PWM 并锁存
 * line_fault。重捕获首拍 D 项为 0。
 *
 * 本函数位于中断上下文，禁止 printf、OLED 刷新或任何无界阻塞操作。
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
    float steering_error_used = 0.0f;
    float tracking_output_scale = g_tracking_output_scale;
    int32_t left_pwm = 0;
    int32_t right_pwm = 0;

    /* 先采集本周期输入，保证后续算法和遥测使用同一时间窗口的数据。 */
    Key_Scan5ms();
    TrackSensor_Sample();
    TrackSensor_GetData(&track);
    Encoder_ReadAndReset(&left_count, &right_count);

    /* 方向补偿后，计数和速度统一约定为车辆前进方向为正。 */
    left_count *= LEFT_ENCODER_SIGN;
    right_count *= RIGHT_ENCODER_SIGN;
    left_speed = CountToSpeed(left_count, 1);
    right_speed = CountToSpeed(right_count, 1);

    /* Watch 中修改的 PI 参数每周期生效，但输出上限只能调低、不能突破 3200。 */
    g_left_speed_pi.kp = g_pid_watch.left.kp;
    g_left_speed_pi.ki = g_pid_watch.left.ki;
    g_left_speed_pi.output_limit = ClampPIDLimit(g_pid_watch.left.output_limit);
    g_right_speed_pi.kp = g_pid_watch.right.kp;
    g_right_speed_pi.ki = g_pid_watch.right.ki;
    g_right_speed_pi.output_limit = ClampPIDLimit(g_pid_watch.right.output_limit);
    g_pid_watch.left.output_limit = g_left_speed_pi.output_limit;
    g_pid_watch.right.output_limit = g_right_speed_pi.output_limit;

    if (g_pid_watch.reset_request != 0U) {
        PID_IncrementalReset(&g_left_speed_pi);
        PID_IncrementalReset(&g_right_speed_pi);
        g_pid_watch.reset_request = 0U;
    }

    if (g_manual_speed_mode) {
        /*
         * 独立轮速模式仅供保留的 PID_Tuner 文件使用。比赛 main.c 不初始化该协议，
         * 因而 PB21 比赛流程不会与串口调速争用控制权。
         */
        left_target = g_manual_left_target_mps;
        right_target = g_manual_right_target_mps;

        if (left_target != 0.0f) {
            left_pwm = (int32_t)PID_IncrementalCalculate(
                &g_left_speed_pi, left_target, left_speed);
        } else {
            PID_IncrementalReset(&g_left_speed_pi);
        }
        if (right_target != 0.0f) {
            right_pwm = (int32_t)PID_IncrementalCalculate(
                &g_right_speed_pi, right_target, right_speed);
        } else {
            PID_IncrementalReset(&g_right_speed_pi);
        }
        Motor_SetPWM(left_pwm * LEFT_MOTOR_OUTPUT_SIGN,
                     right_pwm * RIGHT_MOTOR_OUTPUT_SIGN);
    } else if (g_armed) {
        bool line_valid = (track.active_count != 0U);

        steering_error_used = line_valid ? track.error : 0.0f;
        LineTrack_Calculate(&g_line_controller,
                            track.error,
                            line_valid,
                            APP_CONTROL_PERIOD_MS,
                            &left_target,
                            &right_target);
        left_target *= tracking_output_scale;
        right_target *= tracking_output_scale;
        g_dynamic_base_speed_mps =
            g_line_controller.base_speed_mps * tracking_output_scale;

        if (g_line_controller.line_fault) {
            /* 400 ms 丢线保护到时：锁存比赛故障并保留现场供遥测读取。 */
            g_line_fault = true;
            g_armed = false;
            left_target = 0.0f;
            right_target = 0.0f;
            ResetMotionControllers();
        } else if (g_line_controller.running) {
            left_pwm = (int32_t)PID_IncrementalCalculate(
                &g_left_speed_pi, left_target, left_speed);
            right_pwm = (int32_t)PID_IncrementalCalculate(
                &g_right_speed_pi, right_target, right_speed);
            Motor_SetPWM(left_pwm * LEFT_MOTOR_OUTPUT_SIGN,
                         right_pwm * RIGHT_MOTOR_OUTPUT_SIGN);
        } else {
            /* 尚未见过有效线时沿用源工程停车语义，不让旧 PI 输出残留。 */
            ResetMotionControllers();
        }
    } else {
        /*
         * 停车状态每周期保持控制器和 PWM 为零，但不清 ISR 锁存的 line_fault；
         * 比赛层读到原因后会显式调用 SetArmed(false) 完成整次运行复位。
         */
        ResetMotionControllers();
    }

    /*
     * 控制计算结束后一次性更新遥测字段。读取侧通过临界区复制，不会看到
     * “一半旧周期、一半新周期”的混合快照。
     */
    g_telemetry.raw_track_mask = track.raw_mask;
    g_telemetry.track_mask = track.line_mask;
    g_telemetry.track_ready = track.ready;
    g_telemetry.active_count = track.active_count;
    g_telemetry.marker = track.marker;
    g_telemetry.junction = track.junction;
    g_telemetry.armed = g_armed ? 1U : 0U;
    g_telemetry.manual_speed_mode = g_manual_speed_mode ? 1U : 0U;
    g_telemetry.line_fault = g_line_fault ? 1U : 0U;
    g_telemetry.line_lost_ms = (uint16_t)g_line_controller.lost_time_ms;
    g_telemetry.track_error = track.error;
    g_telemetry.requested_base_speed_mps = g_requested_base_speed_mps;
    g_telemetry.tracking_output_scale = tracking_output_scale;
    g_telemetry.base_speed_mps = g_dynamic_base_speed_mps;
    g_telemetry.left_target_mps = left_target;
    g_telemetry.right_target_mps = right_target;
    g_telemetry.left_speed_mps = left_speed;
    g_telemetry.right_speed_mps = right_speed;
    g_telemetry.left_encoder_count = left_count;
    g_telemetry.right_encoder_count = right_count;
    g_telemetry.left_pwm = Motor_GetLeftPWM();
    g_telemetry.right_pwm = Motor_GetRightPWM();
    g_telemetry.control_tick_count++;

    g_pid_watch.left.target = left_target;
    g_pid_watch.left.feedback = left_speed;
    g_pid_watch.left.error = left_target - left_speed;
    g_pid_watch.left.output = g_left_speed_pi.output;
    g_pid_watch.right.target = right_target;
    g_pid_watch.right.feedback = right_speed;
    g_pid_watch.right.error = right_target - right_speed;
    g_pid_watch.right.output = g_right_speed_pi.output;

    UpdateCarWatch(&track,
                   steering_error_used,
                   left_target,
                   right_target,
                   left_speed,
                   right_speed,
                   left_count,
                   right_count);
    UpdateStatusLED(track.ready);
}

void Car_ControlGetTelemetry(CarTelemetry_t *telemetry)
{
    uint32_t primask = Board_EnterCritical();

    /* 字段逐项复制便于接口演进，也避免把 volatile 结构强制转换后整体复制。 */
    telemetry->raw_track_mask = g_telemetry.raw_track_mask;
    telemetry->track_mask = g_telemetry.track_mask;
    telemetry->track_ready = g_telemetry.track_ready;
    telemetry->active_count = g_telemetry.active_count;
    telemetry->marker = g_telemetry.marker;
    telemetry->junction = g_telemetry.junction;
    telemetry->armed = g_telemetry.armed;
    telemetry->manual_speed_mode = g_telemetry.manual_speed_mode;
    telemetry->line_fault = g_telemetry.line_fault;
    telemetry->line_lost_ms = g_telemetry.line_lost_ms;
    telemetry->track_error = g_telemetry.track_error;
    telemetry->requested_base_speed_mps =
        g_telemetry.requested_base_speed_mps;
    telemetry->tracking_output_scale = g_telemetry.tracking_output_scale;
    telemetry->base_speed_mps = g_telemetry.base_speed_mps;
    telemetry->left_target_mps = g_telemetry.left_target_mps;
    telemetry->right_target_mps = g_telemetry.right_target_mps;
    telemetry->left_speed_mps = g_telemetry.left_speed_mps;
    telemetry->right_speed_mps = g_telemetry.right_speed_mps;
    telemetry->left_encoder_count = g_telemetry.left_encoder_count;
    telemetry->right_encoder_count = g_telemetry.right_encoder_count;
    telemetry->left_pwm = g_telemetry.left_pwm;
    telemetry->right_pwm = g_telemetry.right_pwm;
    telemetry->arm_reject_count = g_telemetry.arm_reject_count;
    telemetry->control_tick_count = g_telemetry.control_tick_count;
    Board_ExitCritical(primask);
}

void Car_ControlGetWatchSnapshot(CarControlWatch_t *watch)
{
    uint32_t primask = Board_EnterCritical();

    /*
     * g_car_watch 在 5 ms ISR 末尾覆盖更新。关闭全局中断后逐项复制，UART3
     * 遥测拿到的是同一个完整控制周期，不会混合相邻周期的误差和轮速。
     */
    watch->raw_track_mask = g_car_watch.raw_track_mask;
    watch->track_mask = g_car_watch.track_mask;
    watch->track_ready = g_car_watch.track_ready;
    watch->active_count = g_car_watch.active_count;
    watch->marker = g_car_watch.marker;
    watch->armed = g_car_watch.armed;
    watch->manual_speed_mode = g_car_watch.manual_speed_mode;
    watch->recovering = g_car_watch.recovering;
    watch->recovery_phase = g_car_watch.recovery_phase;
    watch->line_fault = g_car_watch.line_fault;
    watch->line_lost_samples = g_car_watch.line_lost_samples;
    watch->center_confirm_samples = g_car_watch.center_confirm_samples;
    watch->track_phase = g_car_watch.track_phase;
    watch->curve_direction = g_car_watch.curve_direction;
    watch->line_lost_ms = g_car_watch.line_lost_ms;
    watch->raw_track_error = g_car_watch.raw_track_error;
    watch->steering_error_used = g_car_watch.steering_error_used;
    watch->filtered_error = g_car_watch.filtered_error;
    watch->filtered_derivative = g_car_watch.filtered_derivative;
    watch->p_term_mps = g_car_watch.p_term_mps;
    watch->d_term_mps = g_car_watch.d_term_mps;
    watch->steering_correction_mps =
        g_car_watch.steering_correction_mps;
    watch->phase_bias_mps = g_car_watch.phase_bias_mps;
    watch->requested_base_speed_mps =
        g_car_watch.requested_base_speed_mps;
    watch->tracking_output_scale = g_car_watch.tracking_output_scale;
    watch->desired_base_speed_mps = g_car_watch.desired_base_speed_mps;
    watch->dynamic_base_speed_mps = g_car_watch.dynamic_base_speed_mps;
    watch->left_target_mps = g_car_watch.left_target_mps;
    watch->right_target_mps = g_car_watch.right_target_mps;
    watch->left_speed_mps = g_car_watch.left_speed_mps;
    watch->right_speed_mps = g_car_watch.right_speed_mps;
    watch->left_encoder_count = g_car_watch.left_encoder_count;
    watch->right_encoder_count = g_car_watch.right_encoder_count;
    watch->left_pwm = g_car_watch.left_pwm;
    watch->right_pwm = g_car_watch.right_pwm;
    watch->control_tick_count = g_car_watch.control_tick_count;
    Board_ExitCritical(primask);
}
