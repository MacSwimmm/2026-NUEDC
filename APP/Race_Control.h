#ifndef G3507_BASE_RACE_CONTROL_H
#define G3507_BASE_RACE_CONTROL_H

#include <stdint.h>

/**
 * @file Race_Control.h
 * @brief PB21/PB0/PB1 三种循迹流程、A 点防误判和计时接口。
 *
 * 本模块只运行在主循环，不参与 5 ms 电机闭环。它根据 CarTelemetry_t 中已经
 * 采样好的横线标志和控制周期编号，决定何时使能、预减速和停车；底层 PWM、
 * 编码器与 PI 状态仍完全由 Car_Control 管理。
 */

/** 当前由哪个按键取得底盘控制权。 */
typedef enum {
    RACE_MODE_NONE = 0,
    RACE_MODE_CURRENT_LAP,       /* PB21：当前 5.08 倍参数完成一圈 */
    RACE_MODE_AUX_LAP,           /* PB0：3.50 倍参数缓启动完成一圈 */
    RACE_MODE_AUX_STRAIGHT       /* PB1：3.50 倍直线循迹 6 s + 2 s 缓停 */
} RaceMode_t;

/** 三种模式共用四个流程状态。 */
typedef enum {
    RACE_STATE_WAITING = 0,  /* 等待任一模式按键；底盘保持零 PWM */
    RACE_STATE_LEAVING_A,    /* 已启动，等待起点 A 横线连续消失 */
    RACE_STATE_RUNNING,      /* 一圈已驶离 A，或 PB1 正在定时直线循迹 */
    RACE_STATE_FINISHED      /* 成功完成或故障停车，时间已经冻结 */
} RaceState_t;

/** 终止原因独立于状态，便于 FINISHED 页面区分正常完成与丢线停车。 */
typedef enum {
    RACE_RESULT_NONE = 0,
    RACE_RESULT_SUCCESS,
    RACE_RESULT_LINE_LOST,
    RACE_RESULT_START_REJECTED
} RaceResult_t;

/** OLED 和调试代码读取的一致比赛流程快照。 */
typedef struct {
    RaceMode_t mode;               /* 当前/最近一次选择的按键模式 */
    RaceState_t state;             /* 当前四状态流程 */
    RaceResult_t result;           /* 正常完成、丢线或启动被拒绝 */
    uint8_t marker_cleared;        /* 起点横线已连续消失并开放终点识别 */
    uint8_t approach_active;       /* 当前处于缓启动、接近或缓减速阶段 */
    uint8_t marker_clear_streak;   /* 起点横线连续消失的 5 ms 样本数 */
    uint8_t marker_confirm_streak; /* 终点强横线连续出现的 5 ms 样本数 */
    uint32_t elapsed_ms;           /* 运行中实时计时，FINISHED 后保持冻结值 */
    uint32_t final_time_ms;        /* 停车时冻结的总时间，未结束时为 0 */
    uint32_t start_reject_count;   /* 因循迹输入未就绪而拒绝启动的累计次数 */
} RaceTelemetry_t;

/**
 * @brief Keil Watch 使用的比赛状态机镜像。
 *
 * 字段由主循环覆盖写入，Race_Control 不从该镜像读取控制条件。调试器误写不会
 * 改变真实状态机；state/result 的数值含义直接对应上方两个枚举。
 */
typedef struct {
    RaceMode_t mode;               /* 0空闲、1 PB21、2 PB0、3 PB1 */
    RaceState_t state;             /* 0等待、1驶离A、2跑圈、3终止 */
    RaceResult_t result;           /* 0无、1成功、2丢线、3启动拒绝 */
    uint8_t marker_cleared;        /* 1=已经确认驶离起点横线 */
    uint8_t approach_active;       /* 1=当前处于缓启动、接近或缓减速 */
    uint8_t marker_clear_streak;   /* 横线连续消失样本数 */
    uint8_t marker_confirm_streak; /* 终点横线连续确认样本数 */
    uint32_t start_ms;             /* 当前模式有效按键对应的开始时刻 */
    uint32_t elapsed_ms;           /* 当前/冻结行驶时间 */
    uint32_t final_time_ms;        /* 正常或故障停车时冻结的时间 */
    uint32_t last_control_tick;    /* 状态机已经处理的最后一个 5 ms 样本 */
    uint32_t start_reject_count;   /* 启动输入未就绪累计次数 */
} RaceControlWatch_t;

/** 只读比赛流程镜像，可把表达式 g_race_watch 直接添加到 Keil Watch。 */
extern volatile RaceControlWatch_t g_race_watch;

/** 初始化为等待状态、巡航速度请求和零 PWM。 */
void Race_ControlInit(void);

/**
 * 处理一次消抖后的 PB21 按下事件。
 * WAITING/FINISHED 时尝试启动；运行中再次按下则人工中止并回到 WAITING。
 */
void Race_ControlHandleUserKey(void);

/** PB0：使用独立 3.50 倍速度档缓启动，并沿用 A 点逻辑完成一圈。 */
void Race_ControlHandleLapKey(void);

/** PB1：使用独立 3.50 倍速度档直线循迹，6 s 后用 2 s 缓慢减速至停。 */
void Race_ControlHandleStraightKey(void);

/** 任一模式运行中无条件停车并回到未选择模式的 WAITING。 */
void Race_ControlStop(void);

/**
 * 主循环高频调用：更新缓启停、计时、A 点强弱确认和丢线终止。
 * 横线计数只在 control_tick_count 变化时推进，不能在同一 5 ms 样本上重复累加。
 */
void Race_ControlProcess(void);

/** 复制比赛流程快照；本模块只在主循环访问，因此无需关闭控制中断。 */
void Race_ControlGetTelemetry(RaceTelemetry_t *telemetry);

#endif
