#ifndef G3507_BASE_CAR_CONTROL_H
#define G3507_BASE_CAR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file Car_Control.h
 * @brief 整车控制应用的对外接口与只读遥测快照。
 *
 * 5 ms 闭环运行在定时器中断中，主循环、OLED 和调试串口不能直接读取其中
 * 正在变化的控制器对象，因此统一通过 CarTelemetry_t 快照交换数据。
 */

/**
 * @brief 一次完整的整车运行状态快照。
 *
 * 位图的 bit7 对应最左 CH1，bit0 对应最右 CH8；所有速度单位均为 m/s。
 * PWM 保留符号，正负号表示 Motor BSP 最终收到的方向。
 */
typedef struct {
    uint8_t raw_track_mask;       /* 八路 GPIO 原始电平，尚未转换有效极性 */
    uint8_t track_mask;           /* 极性统一后的位图，1 表示该通道检测到黑线 */
    uint8_t track_ready;          /* 连续正常采样达到门限后为 1，允许用户使能 */
    uint8_t junction;             /* 8 路全黑时为 1，仅作为诊断信息 */
    uint8_t armed;                /* 1=允许闭环输出，0=强制清 PI 并停车 */
    float track_error;            /* 加权横向误差，范围约 -70~+70，正值表示线偏左 */
    float base_speed_mps;         /* 循迹算法当前基础速度，单位 m/s */
    float left_target_mps;        /* 左轮目标速度，单位 m/s */
    float right_target_mps;       /* 右轮目标速度，单位 m/s */
    float left_speed_mps;         /* 左编码器换算得到的实际速度，单位 m/s */
    float right_speed_mps;        /* 右编码器换算得到的实际速度，单位 m/s */
    int32_t left_pwm;             /* 已写入左电机通道的带符号 PWM 计数 */
    int32_t right_pwm;            /* 已写入右电机通道的带符号 PWM 计数 */
    uint32_t arm_reject_count;    /* 因循迹输入未就绪而拒绝使能的累计次数 */
    uint32_t control_tick_count;  /* 5 ms 控制循环累计次数，可用于判断控制是否存活 */
} CarTelemetry_t;

/** 初始化整车 BSP 与算法对象；初始化结束后保持停车且未使能。 */
void Car_ControlInit(void);

/** 固定每 5 ms 由 Board 定时器回调一次，禁止在主循环额外调用。 */
void Car_ControlStep5ms(void);

/**
 * 请求使能或停车。使能时若循迹输入未就绪则返回 false；停车始终返回 true。
 */
bool Car_ControlSetArmed(bool armed);

/** 返回当前整车使能状态。 */
bool Car_ControlIsArmed(void);

/** 按预设步长循环切换基础速度，到达最大值后回到最小值。 */
void Car_ControlCycleBaseSpeed(void);

/** 在临界区复制一份完整遥测快照给主循环或显示层。 */
void Car_ControlGetTelemetry(CarTelemetry_t *telemetry);

#endif
