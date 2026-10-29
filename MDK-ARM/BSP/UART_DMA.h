#ifndef F407_BALANCE_UART_DMA_H
#define F407_BALANCE_UART_DMA_H

/**
 ******************************************************************************
 * @file    UART_DMA.h
 * @brief   USART1/2 ReceiveToIdle DMA 与 USART3 遥测 TX DMA 状态诊断层。
 *
 * @details
 * HAL_UARTEx_RxEventCallback 和 HAL_UART_ErrorCallback 在整个工程中只能各有
 * 一个强定义，因此不能让 Camera.c 和电机模块分别定义同名回调。
 * 本模块作为唯一入口，根据 huart1/2 将原始字节分发给对应业务模块：
 * - USART1：张大头电机回包，保留原始字节并分发给 Motor_Feedback；
 * - USART2：交给 Camera_OnRxData；
 * - USART3：不启动 RX，PB10 的 TX DMA 由 Telemetry 用于电脑端遥测。
 *
 * 两路 RX DMA 都使用 Normal 模式：每次 IDLE 或缓冲区满后，HAL 结束本次接收，
 * 回调处理数据并立即重新调用 ReceiveToIdle DMA。主循环还会周期检查并补挂，
 * 用于恢复偶发 HAL_BUSY、线路错误或接收状态异常。
 ******************************************************************************
 */

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART1 电机回包单次接收缓存；Motor_Feedback 会继续进行组帧和应答匹配。 */
#define UART_DMA_MOTOR_RX_BUFFER_SIZE   64U
/* 每路串口在 Watch 中保留的最近原始字节数，防止调试结构体过大。 */
#define UART_DMA_WATCH_BYTES            32U
/* 主循环发现 RX 不活跃时的最小重试间隔，避免故障状态下无间隔调用 HAL。 */
#define UART_DMA_RETRY_INTERVAL_MS      10U

/**
 * @brief 单路 UART/DMA 的运行状态和诊断信息。
 */
typedef struct
{
    uint32_t baud_rate;              /* HAL 句柄中的实际波特率 */
    uint32_t rx_event_count;         /* ReceiveToIdle 回调次数 */
    uint32_t rx_byte_count;          /* 累计交付给本层的接收字节数 */
    uint32_t rx_start_ok_count;      /* ReceiveToIdle DMA 返回 HAL_OK 的次数 */
    uint32_t rx_start_error_count;   /* 接收启动返回非 HAL_OK 的次数 */
    uint32_t uart_error_count;       /* 进入 HAL_UART_ErrorCallback 的次数 */
    uint32_t last_uart_error_code;   /* 错误恢复前保存的 huart->ErrorCode */
    uint32_t last_rx_tick;           /* 最近一次 RX 事件时间 */
    uint32_t last_error_tick;        /* 最近一次 UART 错误时间 */
    uint32_t last_start_tick;        /* 最近一次尝试启动 RX DMA 的时间 */
    uint32_t hal_tx_state;           /* huart->gState 原始值，观察 TX 状态机 */
    uint32_t hal_rx_state;           /* huart->RxState 原始值，观察 RX 状态机 */
    uint32_t dma_rx_state;           /* HAL_DMA_GetState(hdmarx) */
    uint32_t dma_tx_state;           /* HAL_DMA_GetState(hdmatx) */
    uint16_t last_rx_len;            /* 最近一次 RX 事件的数据长度 */
    uint8_t uart_initialized;        /* UART 句柄已脱离 RESET 状态 */
    uint8_t rx_dma_linked;           /* huart->hdmarx 非空 */
    uint8_t tx_dma_linked;           /* huart->hdmatx 非空 */
    uint8_t dma_mapping_ok;          /* 实际 Stream 与本工程固定映射完全一致 */
    uint8_t rx_active;               /* huart->RxState 当前是否为 BUSY_RX */
    uint8_t last_start_status;       /* 最近一次启动返回的 HAL_StatusTypeDef */
    uint8_t last_abort_status;       /* 最近一次错误恢复时 AbortReceive 的返回值 */
    uint8_t last_event_type;         /* HAL_UART_RXEVENT_IDLE/TC/HT 原始枚举值 */
    uint8_t rx_stream_number;        /* 便于 Watch 阅读的 RX Stream 编号 */
    uint8_t tx_stream_number;        /* 便于 Watch 阅读的 TX Stream 编号 */
    uint8_t last_rx_data[UART_DMA_WATCH_BYTES]; /* 最近数据块的前若干字节 */
} UART_DMA_PortDebug_t;

/**
 * @brief 三路串口的总调试入口。
 * @note 进入 main 的 while(1) 后，正常应看到 all_config_ok=1、all_rx_active=1，
 *       各路 last_start_status=HAL_OK(0)。收到外设数据后对应 rx_event_count 增长。
 */
typedef struct
{
    UART_DMA_PortDebug_t motor_usart1;  /* PA9/PA10，张大头电机总线 */
    UART_DMA_PortDebug_t camera_usart2; /* PA2/PA3，MaixCam 坐标链路 */
    /* 字段名为兼容既有 Watch 保留；当前角色是 PB10 USART3 电脑遥测 TX。 */
    UART_DMA_PortDebug_t jy901s_usart3;
    uint32_t init_tick;                 /* UART_DMA_Init 完成时间 */
    uint32_t process_count;             /* 主循环调用 UART_DMA_Process 的次数 */
    uint8_t all_config_ok;              /* 三路 UART 和固定 DMA 映射均正确 */
    uint8_t all_rx_active;              /* USART1/2 两路必需 RX 均为 BUSY_RX */
} UART_DMA_Debug_t;

/* volatile：中断回调与主循环都会更新此结构。 */
extern volatile UART_DMA_Debug_t g_uart_dma_debug;

/** @brief 核对三路 DMA 映射，并启动 USART1/2 ReceiveToIdle DMA。 */
void UART_DMA_Init(void);
/** @brief 主循环周期维护状态镜像，并对停止的 RX DMA 进行限频重试。 */
void UART_DMA_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_UART_DMA_H */
