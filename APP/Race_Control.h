#ifndef G3507_BASE_RACE_CONTROL_H
#define G3507_BASE_RACE_CONTROL_H

#include <stdint.h>

/**
 * @file Race_Control.h
 * @brief 赛题第 2 项的一圈循迹流程、A 点防误判和计时接口。
 *
 * 本模块只运行在主循环，不参与 5 ms 电机闭环。它根据 CarTelemetry_t 中已经
 * 采样好的横线标志和控制周期编号，决定何时使能、预减速和停车；底层 PWM、
 * 编码器与 PI 状态仍完全由 Car_Control 管理。
 */

/** 赛题第 2 项仅需四个流程状态，暂不引入摆球或多任务模式。 */
typedef enum {
    RACE_STATE_WAITING = 0,  /* 等待 PB21；底盘保持零 PWM */
    RACE_STATE_LEAVING_A,    /* 已启动，等待起点 A 横线连续消失 */
    RACE_STATE_RUNNING,      /* 已驶离 A，允许识别第二次 A 横线 */
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
    RaceState_t state;             /* 当前四状态流程 */
    RaceResult_t result;           /* 正常完成、丢线或启动被拒绝 */
    uint8_t marker_cleared;        /* 起点横线已连续消失并开放终点识别 */
    uint8_t approach_active;       /* 已到预减速时刻，当前请求接近速度 */
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
    RaceState_t state;             /* 0等待、1驶离A、2跑圈、3终止 */
    RaceResult_t result;           /* 0无、1成功、2丢线、3启动拒绝 */
    uint8_t marker_cleared;        /* 1=已经确认驶离起点横线 */
    uint8_t approach_active;       /* 1=已经跨过配置的圈末预减速时刻 */
    uint8_t marker_clear_streak;   /* 横线连续消失样本数 */
    uint8_t marker_confirm_streak; /* 终点横线连续确认样本数 */
    uint32_t start_ms;             /* PB21 有效事件对应的开始时刻 */
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

/**
 * 主循环高频调用：更新计时、圈末预减速、A 点强弱确认和丢线终止。
 * 横线计数只在 control_tick_count 变化时推进，不能在同一 5 ms 样本上重复累加。
 */
void Race_ControlProcess(void);

/** 复制比赛流程快照；本模块只在主循环访问，因此无需关闭控制中断。 */
void Race_ControlGetTelemetry(RaceTelemetry_t *telemetry);

#endif
