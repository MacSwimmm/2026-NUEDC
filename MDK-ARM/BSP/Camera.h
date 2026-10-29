#ifndef F407_BALANCE_CAMERA_H
#define F407_BALANCE_CAMERA_H

/**
 ******************************************************************************
 * @file    Camera.h
 * @brief   MaixCam 小球坐标接收与状态快照接口。
 *
 * @details
 * 本模块只负责“把摄像头串口文本转换成可供控制层读取的坐标”，不负责：
 * 1. 启动 USART2/DMA（由 UART_DMA 模块统一完成）；
 * 2. 视觉坐标标定（第三问约定发送端直接给出毫米）；
 * 3. PID 计算或张大头电机控制。
 *
 * 当前支持的文本协议：
 * - "x,y\n"：二维有符号整数坐标；
 * - "x\n"  ：只有 X 轴坐标，Y 自动记为 0；
 * - "N\n"  ：摄像头通信正常，但本帧没有识别到小球。
 *
 * 第三问固定约定 X 为相对 O 点的有符号毫米。STM32 不修改视觉端代码，
 * 只对 X 做不可调的 |x|<=120 mm 物理合理性检查。
 ******************************************************************************
 */

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART2 单次 ReceiveToIdle DMA 的接收缓存长度。
 * 典型坐标帧只有十几个字节，64 字节可覆盖多帧粘连或较长调试文本。
 */
#define CAMERA_RX_DMA_BUFFER_SIZE   64U

/* 一条完整文本帧（不含 \r/\n 和字符串结尾 \0）的最大长度。 */
#define CAMERA_LINE_BUFFER_SIZE     32U

/* 超过该时间没有收到任何“完整文本行”时，把摄像头链路判为离线。
 * 30 FPS 的正常帧间隔约为 33 ms，200 ms 能容忍少量丢帧而又不会长期沿用旧坐标。
 */
#define CAMERA_LINK_TIMEOUT_MS      200U

/* 轨道物理坐标合理性边界，不属于 Watch 调参，非法值不得刷新有效坐标时间。 */
#define CAMERA_X_PHYSICAL_LIMIT_MM  120

/* 最近一条完整文本帧的解析结果，主要供 Keil Watch 快速判断。 */
#define CAMERA_FRAME_NONE           0U
#define CAMERA_FRAME_COORD          1U
#define CAMERA_FRAME_LOST           2U
#define CAMERA_FRAME_INVALID        3U

/**
 * @brief 提供给上层控制算法的摄像头数据快照。
 */
typedef struct
{
    int16_t x;                    /* 最近一次有效坐标帧中的 X，单位 mm */
    int16_t y;                    /* 当前相机协议第二字段：整数 cm/s；一维协议时为 0 */
    uint32_t frame_sequence;      /* 每收到一条完整文本行加 1，含 N 和非法帧 */
    uint32_t last_frame_tick;     /* 最近完整文本行到达的 HAL 毫秒计数 */
    uint32_t valid_sequence;      /* 每收到一个通过格式和物理检查的坐标加 1 */
    uint32_t last_valid_tick;     /* 最近有效坐标到达时间；N/非法帧绝不刷新 */
    uint8_t target_found;         /* 1=最近一帧含有效坐标；0=N、无效帧或已超时 */
    uint8_t online;               /* 1=链路近期有完整行；0=从未收到或已超时 */
} Camera_Data_t;

/**
 * @brief 摄像头链路的完整调试镜像。
 *
 * @note 在 Keil Watch 中添加 g_camera_debug 即可观察。计数器用于区分：
 * “DMA 收到了字节”“组成了完整文本行”“坐标解析成功”以及“链路仍在线”。
 */
typedef struct
{
    Camera_Data_t data;               /* 控制层可读取的数据快照 */
    uint32_t rx_chunk_count;           /* UART_DMA 向本模块提交数据块的次数 */
    uint32_t rx_byte_count;            /* 累计收到的原始字节数 */
    uint32_t completed_line_count;     /* 已遇到行结束符并组成完整行的次数 */
    uint32_t valid_coord_count;        /* 成功解析为 x 或 x,y 的次数 */
    uint32_t lost_frame_count;         /* 收到 N/n 的次数 */
    uint32_t parse_error_count;        /* 数字格式、尾随字符或长度异常次数 */
    uint32_t physical_reject_count;    /* |x|>120 mm 的物理不合理坐标次数 */
    uint32_t line_overflow_count;      /* 单行超过 CAMERA_LINE_BUFFER_SIZE 的次数 */
    uint32_t uart_error_count;         /* USART2 进入 HAL 错误回调的次数 */
    uint32_t last_uart_error_code;     /* 最近一次 huart2.ErrorCode 原始值 */
    uint32_t last_rx_tick;             /* 最近收到任意 DMA 数据块的时间 */
    uint32_t last_error_tick;          /* 最近发生 UART 错误的时间 */
    uint16_t last_rx_len;              /* 最近一个 DMA 数据块的实际字节数 */
    uint8_t last_frame_type;           /* CAMERA_FRAME_*，最近完整行的类型 */
    char last_frame[CAMERA_LINE_BUFFER_SIZE + 1U]; /* 最近完整行，已补 \0 */
} Camera_Debug_t;

/* volatile：该结构的部分字段会在 USART2 中断上下文中更新。 */
extern volatile Camera_Debug_t g_camera_debug;

/** @brief 清空文本组帧状态和全部调试计数；启动 DMA 前调用一次。 */
void Camera_Init(void);

/** @brief 主循环周期调用，依据 CAMERA_LINK_TIMEOUT_MS 更新 online 状态。 */
void Camera_Process(void);

/**
 * @brief 向文本流解析器提交一段 USART2 原始数据。
 * @note 由 UART_DMA 的 HAL_UARTEx_RxEventCallback 在中断上下文调用；
 *       能处理半行跨 DMA 回调以及一个 DMA 数据块内包含多行的情况。
 */
void Camera_OnRxData(const uint8_t *data, uint16_t size);

/** @brief 记录 USART2 错误码；真正的 DMA 中止和重启由 UART_DMA 负责。 */
void Camera_OnUartError(uint32_t error_code);

/**
 * @brief 以临界区方式复制一份一致的数据快照。
 * @param data 输出缓冲区，不能为 NULL。
 * @retval 1=摄像头链路在线；0=离线或参数无效。
 */
uint8_t Camera_GetData(Camera_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_CAMERA_H */
