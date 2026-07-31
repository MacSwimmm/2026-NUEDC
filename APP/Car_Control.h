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
    uint8_t active_count;         /* 当前检测到黑线的探头数量 */
    uint8_t marker;               /* 1=连续相邻至少 3 路黑，仅作宽线诊断 */
    uint8_t junction;             /* 1=连续相邻至少 4 路黑，A 点强候选 */
    uint8_t armed;                /* 1=允许闭环输出，0=强制清 PI 并停车 */
    uint8_t manual_speed_mode;    /* 1=SetSpeed_Left/Right 独立速度闭环模式 */
    uint8_t line_fault;           /* 持续丢线超过保护时间后置 1 */
    uint16_t line_lost_ms;        /* 当前连续丢线时间，单位 ms */
    float track_error;            /* 源权重误差约 -350~+350，负左、正右 */
    float requested_base_speed_mps; /* 比赛层请求的源算法速度上限，单位 m/s */
    float base_speed_mps;         /* 原公式按误差计算的 60~100 RPM 基础速度 */
    float left_target_mps;        /* 左轮目标速度，单位 m/s */
    float right_target_mps;       /* 右轮目标速度，单位 m/s */
    float left_speed_mps;         /* 左编码器换算得到的实际速度，单位 m/s */
    float right_speed_mps;        /* 右编码器换算得到的实际速度，单位 m/s */
    int32_t left_encoder_count;   /* 最近 5 ms 左编码器增量，车辆前进方向为正 */
    int32_t right_encoder_count;  /* 最近 5 ms 右编码器增量，车辆前进方向为正 */
    int32_t left_pwm;             /* 已写入左电机通道的带符号 PWM 计数 */
    int32_t right_pwm;            /* 已写入右电机通道的带符号 PWM 计数 */
    uint32_t arm_reject_count;    /* 因循迹输入未就绪而拒绝使能的累计次数 */
    uint32_t control_tick_count;  /* 5 ms 控制循环累计次数，可用于判断控制是否存活 */
} CarTelemetry_t;

/**
 * @brief Keil Watch 使用的循迹闭环关键量镜像。
 *
 * 该结构只由 5 ms 控制中断覆盖写入，控制算法从不读取其中字段。因此在调试器
 * 中误改 g_car_watch 只会暂时改变显示值，不会改变转向、速度或电机输出。
 * 需要在线修改 PI 参数时仍使用已有的 g_pid_watch。
 */
typedef struct {
    uint8_t raw_track_mask;          /* 原始八路电平：白=1、黑=0 */
    uint8_t track_mask;              /* 统一极性后：1=检测到黑线 */
    uint8_t track_ready;             /* 1=循迹输入满足启动稳定门限 */
    uint8_t active_count;            /* 本周期黑色探头数量 */
    uint8_t marker;                  /* 1=连续相邻至少 3 路黑，仅作诊断 */
    uint8_t armed;                   /* 1=循迹闭环已使能 */
    uint8_t manual_speed_mode;       /* 1=独立轮速调试模式 */
    uint8_t recovering;              /* 1=正向最后真实转向的反方向原地搜线 */
    uint8_t recovery_phase;          /* 兼容旧帧布局，源算法固定为 0 */
    uint8_t line_fault;              /* 1=持续丢线，已经锁定停车 */
    uint8_t line_lost_samples;       /* 连续丢线样本数，超过 255 时饱和显示 */
    uint8_t center_confirm_samples;  /* 兼容旧帧布局，固定为 0 */
    uint8_t track_phase;             /* 兼容旧帧布局，固定为 0 */
    int8_t curve_direction;          /* -1左转、+1右转、0尚未越过死区 */
    uint16_t line_lost_ms;           /* 连续丢线时间，单位 ms */
    float raw_track_error;           /* 八路源权重原始误差，约 -350~+350 */
    float steering_error_used;       /* 有线时送入源 PD 的误差，丢线时为 0 */
    float filtered_error;            /* 兼容字段：实际为未滤波的最后有效误差 */
    float filtered_derivative;       /* 兼容字段：实际为相邻有效帧离散差分 */
    float p_term_mps;                 /* Kp×当前误差的比例修正 */
    float d_term_mps;                 /* Kd×离散误差差分，不单独限幅 */
    float steering_correction_mps;   /* 源差速修正；正值使左轮快、向右转 */
    float phase_bias_mps;            /* 兼容旧帧布局，固定为 0 */
    float requested_base_speed_mps;  /* 比赛层请求基础速度 */
    float desired_base_speed_mps;    /* 源公式本周期计算出的基础速度 */
    float dynamic_base_speed_mps;    /* 与 desired 相同；不再经过旧斜坡 */
    float left_target_mps;           /* 左轮最终目标速度 */
    float right_target_mps;          /* 右轮最终目标速度 */
    float left_speed_mps;            /* 左轮编码器反馈速度 */
    float right_speed_mps;           /* 右轮编码器反馈速度 */
    int32_t left_encoder_count;      /* 最近 5 ms 左轮计数，前进为正 */
    int32_t right_encoder_count;     /* 最近 5 ms 右轮计数，前进为正 */
    int32_t left_pwm;                /* Motor BSP 实际左 PWM */
    int32_t right_pwm;               /* Motor BSP 实际右 PWM */
    uint32_t control_tick_count;     /* 最后写入，变化表示整组镜像已刷新 */
} CarControlWatch_t;

/** 只读循迹镜像，可把表达式 g_car_watch 直接添加到 Keil Watch。 */
extern volatile CarControlWatch_t g_car_watch;

/**
 * 在极短临界区复制完整循迹诊断镜像，供 UART3 遥测等只读消费者使用。
 * 调用方不得直接读取 g_car_watch，否则可能跨越两个 5 ms 控制周期。
 */
void Car_ControlGetWatchSnapshot(CarControlWatch_t *watch);

/** 初始化整车 BSP 与算法对象；初始化结束后保持停车且未使能。 */
void Car_ControlInit(void);

/**
 * 切换到独立轮速闭环并设置左轮目标速度，单位 m/s；正值向前、负值向后。
 * 目标会限制到 WHEEL_SPEED_TARGET_LIMIT_MPS，调用前必须完成 Car_ControlInit()。
 */
void SetSpeed_Left(float target_speed_mps);

/** 设置右轮独立闭环目标速度，约定与 SetSpeed_Left 相同。 */
void SetSpeed_Right(float target_speed_mps);

/** 退出独立轮速模式、清空两侧 PI 历史并立即停车。 */
void SetSpeed_Stop(void);

/** 固定每 5 ms 由 Board 定时器回调一次，禁止在主循环额外调用。 */
void Car_ControlStep5ms(void);

/**
 * 请求使能或停车。使能时若循迹输入未就绪则返回 false；停车始终返回 true。
 */
bool Car_ControlSetArmed(bool armed);

/** 返回当前整车使能状态。 */
bool Car_ControlIsArmed(void);

/** 设置循迹基础速度上限；有线速度仍按原误差公式连续计算。 */
void Car_ControlSetBaseSpeed(float base_speed_mps);

/** 按预设步长循环切换基础速度，到达最大值后回到最小值。 */
void Car_ControlCycleBaseSpeed(void);

/** 在临界区复制一份完整遥测快照给主循环或显示层。 */
void Car_ControlGetTelemetry(CarTelemetry_t *telemetry);

#endif
