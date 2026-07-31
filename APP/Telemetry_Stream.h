#ifndef G3507_BASE_TELEMETRY_STREAM_H
#define G3507_BASE_TELEMETRY_STREAM_H

#include <stdint.h>

/**
 * @file Telemetry_Stream.h
 * @brief UART3/PB12 无线 DAP 循迹诊断遥测协议与主循环调度接口。
 *
 * 固件始终只发送，不接收电脑命令，也不改变比赛状态、电机目标或 PID 参数。
 * 每帧固定 74 字节：8 字节帧头、64 字节负载和 2 字节 CRC16。电脑端通过
 * 帧序号、CRC 和板端丢帧计数区分无线链路丢包与 MCU 发送队列拥塞。
 */

#define TELEMETRY_FRAME_SYNC_0          0xA5U
#define TELEMETRY_FRAME_SYNC_1          0x5AU
#define TELEMETRY_PROTOCOL_VERSION      1U
#define TELEMETRY_FRAME_TYPE_CONTROL    1U
#define TELEMETRY_PAYLOAD_LENGTH        64U
#define TELEMETRY_FRAME_LENGTH          74U

/** 单帧事件只用于电脑端自动切分一圈记录，不参与 Race_Control 的真实状态。 */
typedef enum {
    TELEMETRY_EVENT_NONE = 0,
    TELEMETRY_EVENT_RACE_STARTED = 1,
    TELEMETRY_EVENT_RACE_FINISHED = 2,
    TELEMETRY_EVENT_MANUAL_STOP = 3
} TelemetryEvent_t;

/** Keil Watch 可直接观察的 UART3 遥测运行状态；控制算法从不读取这些字段。 */
typedef struct {
    uint16_t last_sequence;       /* 最近一次尝试发送的帧序号 */
    uint16_t last_frame_length;   /* 固定应为 74 */
    uint16_t queued_bytes;        /* UART3 软件环形缓冲待发字节数 */
    uint8_t last_event;           /* TelemetryEvent_t */
    uint32_t frames_queued;       /* 成功整帧入队累计数 */
    uint32_t frames_dropped;      /* 队列空间不足导致的整帧丢弃数 */
    uint32_t last_frame_ms;       /* 最近一次生成遥测帧的板端毫秒时刻 */
} TelemetryStreamWatch_t;

/** 只读遥测状态镜像，可在 Keil Watch 中添加 g_telemetry_watch。 */
extern volatile TelemetryStreamWatch_t g_telemetry_watch;

/** 初始化 UART3 非阻塞 TX 队列和比赛状态边沿跟踪；不发送首帧。 */
void Telemetry_StreamInit(void);

/**
 * 在主循环高频调用；内部每 TELEMETRY_STREAM_PERIOD_MS 生成并尝试入队一帧。
 * 该函数只读取 Car/Race 快照，不得从 5 ms 控制 ISR 中调用。
 */
void Telemetry_StreamProcess(void);

#endif
