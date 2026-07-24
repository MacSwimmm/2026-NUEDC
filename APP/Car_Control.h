#ifndef G3507_BASE_CAR_CONTROL_H
#define G3507_BASE_CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 主循环和显示层只读取这份快照，不直接访问中断中的控制器内部状态。 */
typedef struct {
    uint8_t raw_track_mask;       /* 八路循迹 GPIO 原始电平 */
    uint8_t track_mask;           /* 极性统一后，1 表示检测到黑线 */
    uint8_t track_ready;          /* 启动安全检查是否通过 */
    uint8_t junction;             /* 当前是否八路全黑 */
    uint8_t armed;                /* 1=允许闭环输出，0=强制停车 */
    float track_error;            /* 加权横向误差，约 -70~+70 */
    float base_speed_mps;         /* 当前基础速度，单位 m/s */
    float left_target_mps;
    float right_target_mps;
    float left_speed_mps;
    float right_speed_mps;
    int32_t left_pwm;             /* 已写入 TB6612 的带符号 PWM */
    int32_t right_pwm;
    float battery_voltage;
    uint32_t arm_reject_count;    /* 因循迹输入未就绪而拒绝使能的次数 */
    uint32_t control_tick_count;  /* 5 ms 控制循环累计次数 */
} CarTelemetry_t;

/* 初始化后默认停车；Car_ControlStep5ms 由 BSP 定时器回调，不在主循环直接调用。 */
void Car_ControlInit(void);
void Car_ControlStep5ms(void);
bool Car_ControlSetArmed(bool armed);
bool Car_ControlIsArmed(void);
void Car_ControlCycleBaseSpeed(void);
void Car_ControlSetBatteryVoltage(float voltage);
void Car_ControlGetTelemetry(CarTelemetry_t *telemetry);

#endif
