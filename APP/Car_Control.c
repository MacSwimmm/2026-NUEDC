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

static IncrementalPI_t g_left_speed_pi;
static IncrementalPI_t g_right_speed_pi;
static LineTrack_t g_line_controller;
static volatile bool g_armed = false;
static volatile float g_battery_voltage = 0.0f;
static volatile CarTelemetry_t g_telemetry;

static float CountToSpeed(int32_t count, int32_t direction_sign)
{
    float pulses_per_wheel_turn = ENCODER_COUNT_MULTIPLIER *
                                  ENCODER_MOTOR_SHAFT_PPR *
                                  MOTOR_GEAR_RATIO;

    return (float)(count * direction_sign) * APP_CONTROL_FREQUENCY_HZ *
           WHEEL_CIRCUMFERENCE_M / pulses_per_wheel_turn;
}

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
    g_battery_voltage = 0.0f;
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
        TrackSensor_GetData(&track);
        if (track.ready == 0U) {
            g_telemetry.arm_reject_count++;
            Board_ExitCritical(primask);
            return false;
        }
        ResetClosedLoop();
        g_armed = true;
        Board_ExitCritical(primask);
        return true;
    }

    g_armed = false;
    ResetClosedLoop();
    Board_ExitCritical(primask);
    return true;
}

bool Car_ControlIsArmed(void)
{
    return g_armed;
}

void Car_ControlCycleBaseSpeed(void)
{
    uint32_t primask;
    float speed;

    /* base_speed_mps 同时会被 5 ms 控制中断读取，更新时短暂进入临界区。 */
    primask = Board_EnterCritical();
    speed = g_line_controller.base_speed_mps + TRACK_BASE_SPEED_STEP_MPS;
    if (speed > TRACK_BASE_SPEED_MAX_MPS) speed = TRACK_BASE_SPEED_MIN_MPS;
    LineTrack_SetBaseSpeed(&g_line_controller, speed);
    Board_ExitCritical(primask);
}

void Car_ControlSetBatteryVoltage(float voltage)
{
    uint32_t primask = Board_EnterCritical();

    g_battery_voltage = voltage;
    Board_ExitCritical(primask);
}

/**
 * @brief 固定 5 ms 执行一次的整车控制步骤。
 *
 * 顺序保持简单且固定：采循迹 → 取编码器 → 换算轮速 → 计算目标 → 两轮 PI → PWM。
 * 工程没有弯道/路口状态机；未使能或完全丢线时立即清 PI 并写零 PWM。
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

    Key_Scan5ms();
    TrackSensor_Sample();
    TrackSensor_GetData(&track);
    Encoder_ReadAndReset(&left_count, &right_count);

    left_speed = CountToSpeed(left_count, LEFT_ENCODER_SIGN);
    right_speed = CountToSpeed(right_count, RIGHT_ENCODER_SIGN);
    line_valid = (track.active_count != 0U);

    if (g_armed && line_valid) {
        LineTrack_Calculate(&g_line_controller, track.error, true,
                            &left_target, &right_target);
        left_pwm = (int32_t)PID_IncrementalCalculate(
            &g_left_speed_pi, left_target, left_speed);
        right_pwm = (int32_t)PID_IncrementalCalculate(
            &g_right_speed_pi, right_target, right_speed);
        Motor_SetPWM(left_pwm * LEFT_MOTOR_OUTPUT_SIGN,
                     right_pwm * RIGHT_MOTOR_OUTPUT_SIGN);
    } else {
        ResetClosedLoop();
    }

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
    g_telemetry.battery_voltage = g_battery_voltage;
    g_telemetry.control_tick_count++;

    UpdateStatusLED(track.ready);
}

void Car_ControlGetTelemetry(CarTelemetry_t *telemetry)
{
    uint32_t primask = Board_EnterCritical();

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
    telemetry->battery_voltage = g_telemetry.battery_voltage;
    telemetry->arm_reject_count = g_telemetry.arm_reject_count;
    telemetry->control_tick_count = g_telemetry.control_tick_count;
    Board_ExitCritical(primask);
}
