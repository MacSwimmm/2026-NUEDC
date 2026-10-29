#include "UART_DMA.h"

#include "Camera.h"
#include "Motor_Feedback.h"
#include "usart.h"

#include <string.h>

/*
 * 本文件是 USART1/2 串口接收的唯一 HAL 回调实现，解决两个问题：
 * 1. HAL 的全局回调不能在 Camera.c、Motor_Feedback.c 中重复定义；
 * 2. CubeMX 生成 DMA 句柄和中断，只代表“硬件资源已配置”，并不会自动调用
 *    HAL_UARTEx_ReceiveToIdle_DMA()。必须在运行时显式启动，串口才真正接收。
 *
 * 接收采用 Normal DMA：一次 IDLE/满缓存事件结束后立即重挂。相比 Circular DMA，
 * 这种方式得到的 size 就是本次有效字节数，不需要维护循环缓冲区的累计位置。
 */

/* Keil Watch 总入口；在中断回调和主循环维护函数中都会更新。 */
volatile UART_DMA_Debug_t g_uart_dma_debug = {0};

/* USART1：张大头驱动器回包。既保留原始数据，也分发给 Motor_Feedback 组帧。 */
static uint8_t s_motor_rx_buffer[UART_DMA_MOTOR_RX_BUFFER_SIZE];
/* USART2：MaixCam 文本坐标数据。 */
static uint8_t s_camera_rx_buffer[CAMERA_RX_DMA_BUFFER_SIZE];
/**
 * @brief 根据 HAL UART 句柄取得对应的 Watch 子结构。
 * @retval 三路已登记句柄返回对应结构；未知 UART 返回 NULL，不误处理其他外设。
 */
static UART_DMA_PortDebug_t *UART_DMA_GetPort(UART_HandleTypeDef *huart)
{
    if (huart == &huart1) {
        return (UART_DMA_PortDebug_t *)&g_uart_dma_debug.motor_usart1;
    }
    if (huart == &huart2) {
        return (UART_DMA_PortDebug_t *)&g_uart_dma_debug.camera_usart2;
    }
    if (huart == &huart3) {
        return (UART_DMA_PortDebug_t *)&g_uart_dma_debug.jy901s_usart3;
    }
    return NULL;
}

/**
 * @brief 把 HAL UART/DMA 状态机的当前值镜像到调试结构。
 *
 * @details rx_active 直接由 huart->RxState 是否等于 BUSY_RX 得出。
 * 该值比“上次启动返回 HAL_OK”更接近此刻是否真的挂着接收，但仍属于软件状态；
 * 只有 rx_event_count 增长才能证明线上确实有字节触发回调。
 */
static void UART_DMA_UpdatePortState(UART_HandleTypeDef *huart,
                                     UART_DMA_PortDebug_t *port)
{
    port->hal_tx_state = (uint32_t)huart->gState;
    port->hal_rx_state = (uint32_t)huart->RxState;
    port->rx_active = (uint8_t)(huart->RxState == HAL_UART_STATE_BUSY_RX);

    if (huart->hdmarx != NULL) {
        port->dma_rx_state = (uint32_t)HAL_DMA_GetState(huart->hdmarx);
    }
    if (huart->hdmatx != NULL) {
        port->dma_tx_state = (uint32_t)HAL_DMA_GetState(huart->hdmatx);
    }
}

/**
 * @brief 初始化单路串口的静态配置诊断信息。
 * @param expected_rx/expected_tx 本工程固定要求的 DMA Stream 地址。
 *
 * @details 不只检查 hdmarx/hdmatx 非空，还核对它们是否链接到预期 Stream，
 * 防止以后 CubeMX 重新分配 Stream 而 Watch 仍显示“DMA 已链接”却掩盖映射变化。
 */
static void UART_DMA_SetConfig(UART_HandleTypeDef *huart,
                               UART_DMA_PortDebug_t *port,
                               DMA_Stream_TypeDef *expected_rx,
                               DMA_Stream_TypeDef *expected_tx,
                               uint8_t rx_stream_number,
                               uint8_t tx_stream_number)
{
    port->baud_rate = huart->Init.BaudRate;
    port->uart_initialized = (uint8_t)((huart->Instance != NULL) &&
                                       (huart->gState != HAL_UART_STATE_RESET) &&
                                       (huart->RxState != HAL_UART_STATE_RESET));
    port->rx_dma_linked = (uint8_t)(huart->hdmarx != NULL);
    port->tx_dma_linked = (uint8_t)(huart->hdmatx != NULL);
    port->dma_mapping_ok = (uint8_t)((huart->hdmarx != NULL) &&
                                     (huart->hdmatx != NULL) &&
                                     (huart->hdmarx->Instance == expected_rx) &&
                                     (huart->hdmatx->Instance == expected_tx));
    port->rx_stream_number = rx_stream_number;
    port->tx_stream_number = tx_stream_number;
    UART_DMA_UpdatePortState(huart, port);
}

/**
 * @brief 为指定 UART 启动一次 ReceiveToIdle DMA，并记录 HAL 返回值。
 *
 * @details HAL 在 Normal DMA 模式下可能因 IDLE、传输完成或错误结束本次接收。
 * 每次重新启动后都关闭 DMA 半传输中断：短文本/固定帧协议不需要在缓冲区一半时
 * 额外回调，否则会增加中断次数，并可能让调试者误以为一帧被完整接收。
 */
static HAL_StatusTypeDef UART_DMA_StartReceive(UART_HandleTypeDef *huart,
                                                uint8_t *buffer,
                                                uint16_t size,
                                                UART_DMA_PortDebug_t *port)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(huart, buffer, size);
    port->last_start_status = (uint8_t)status;
    port->last_start_tick = HAL_GetTick();

    if (status == HAL_OK) {
        port->rx_start_ok_count++;
        if (huart->hdmarx != NULL) {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
        }
    } else {
        port->rx_start_error_count++;
    }

    UART_DMA_UpdatePortState(huart, port);
    return status;
}

/**
 * @brief 根据 UART 角色选择正确的 DMA 缓冲区和长度，然后启动接收。
 */
static HAL_StatusTypeDef UART_DMA_StartForUart(UART_HandleTypeDef *huart)
{
    UART_DMA_PortDebug_t *port = UART_DMA_GetPort(huart);

    if (port == NULL) {
        return HAL_ERROR;
    }
    if (huart == &huart1) {
        return UART_DMA_StartReceive(huart, s_motor_rx_buffer,
                                     UART_DMA_MOTOR_RX_BUFFER_SIZE, port);
    }
    if (huart == &huart2) {
        return UART_DMA_StartReceive(huart, s_camera_rx_buffer,
                                     CAMERA_RX_DMA_BUFFER_SIZE, port);
    }
    /* USART3 已改为电脑遥测 TX，不启动 ReceiveToIdle RX。 */
    return HAL_ERROR;
}

/**
 * @brief 记录一次 RX 事件的长度、时间、事件类型和前若干原始字节。
 *
 * @details Watch 缓冲只保存数据块前 UART_DMA_WATCH_BYTES 字节；业务模块仍会收到
 * 完整 size 字节。未使用的尾部清零，避免查看短帧时把上一次长帧残留误认为新数据。
 */
static void UART_DMA_RecordRx(UART_HandleTypeDef *huart,
                              UART_DMA_PortDebug_t *port,
                              const uint8_t *buffer,
                              uint16_t size)
{
    uint16_t copy_size = size;

    if (copy_size > UART_DMA_WATCH_BYTES) {
        copy_size = UART_DMA_WATCH_BYTES;
    }
    if ((buffer != NULL) && (copy_size != 0U)) {
        memcpy((void *)port->last_rx_data, buffer, copy_size);
    }
    if (copy_size < UART_DMA_WATCH_BYTES) {
        memset((void *)&port->last_rx_data[copy_size], 0,
               UART_DMA_WATCH_BYTES - copy_size);
    }

    port->rx_event_count++;
    port->rx_byte_count += size;
    port->last_rx_len = size;
    port->last_rx_tick = HAL_GetTick();
    port->last_event_type = (uint8_t)HAL_UARTEx_GetRxEventType(huart);
}

/**
 * @brief 初始化三路 UART/DMA 诊断并启动 USART1/2 接收。
 *
 * @details 固定映射：
 * - USART1 RX DMA2_Stream2，TX DMA2_Stream7；
 * - USART2 RX DMA1_Stream5，TX DMA1_Stream6；
 * - USART3 RX DMA1_Stream1，TX DMA1_Stream3。
 *
 * 调用本函数前，MX_DMA_Init() 和三个 MX_USARTx_UART_Init() 必须已经完成，
 * Camera_Init()/Motor_FeedbackInit() 也应先清好业务状态，避免接收启动后再清数据。
 * USART3 的 TX DMA 由 Telemetry 模块单独使用，PB11 当前不启动接收。
 */
void UART_DMA_Init(void)
{
    memset((void *)&g_uart_dma_debug, 0, sizeof(g_uart_dma_debug));
    memset(s_motor_rx_buffer, 0, sizeof(s_motor_rx_buffer));
    memset(s_camera_rx_buffer, 0, sizeof(s_camera_rx_buffer));

    UART_DMA_SetConfig(&huart1,
                       (UART_DMA_PortDebug_t *)&g_uart_dma_debug.motor_usart1,
                       DMA2_Stream2, DMA2_Stream7, 2U, 7U);
    UART_DMA_SetConfig(&huart2,
                       (UART_DMA_PortDebug_t *)&g_uart_dma_debug.camera_usart2,
                       DMA1_Stream5, DMA1_Stream6, 5U, 6U);
    UART_DMA_SetConfig(&huart3,
                       (UART_DMA_PortDebug_t *)&g_uart_dma_debug.jy901s_usart3,
                       DMA1_Stream1, DMA1_Stream3, 1U, 3U);

    /* all_config_ok 只验证 HAL 句柄和 DMA 映射，不代表线上外设已接好或在发数据。 */
    g_uart_dma_debug.all_config_ok =
        (uint8_t)(g_uart_dma_debug.motor_usart1.uart_initialized &&
                  g_uart_dma_debug.motor_usart1.dma_mapping_ok &&
                  g_uart_dma_debug.camera_usart2.uart_initialized &&
                  g_uart_dma_debug.camera_usart2.dma_mapping_ok &&
                  g_uart_dma_debug.jy901s_usart3.uart_initialized &&
                  g_uart_dma_debug.jy901s_usart3.dma_mapping_ok);

    (void)UART_DMA_StartForUart(&huart1);
    (void)UART_DMA_StartForUart(&huart2);

    g_uart_dma_debug.init_tick = HAL_GetTick();
    UART_DMA_Process();
}

/**
 * @brief 主循环中的非阻塞自恢复与 Watch 状态刷新。
 *
 * @details 正常接收时 USART1/2 RxState 均保持 BUSY_RX，本函数只读取状态。
 * 若某一路停止且距离上次启动至少 10 ms，则补调一次 ReceiveToIdle DMA。
 * 限频可以避免接线/配置长期错误时在 while(1) 中疯狂调用 HAL 并刷爆计数器。
 */
void UART_DMA_Process(void)
{
    uint32_t now = HAL_GetTick();
    UART_DMA_PortDebug_t *motor =
        (UART_DMA_PortDebug_t *)&g_uart_dma_debug.motor_usart1;
    UART_DMA_PortDebug_t *camera =
        (UART_DMA_PortDebug_t *)&g_uart_dma_debug.camera_usart2;
    UART_DMA_PortDebug_t *telemetry =
        (UART_DMA_PortDebug_t *)&g_uart_dma_debug.jy901s_usart3;

    UART_DMA_UpdatePortState(&huart1, motor);
    UART_DMA_UpdatePortState(&huart2, camera);
    UART_DMA_UpdatePortState(&huart3, telemetry);

    /* 分路重试，某一路故障不会阻止另外两路继续工作。 */
    if ((!motor->rx_active) &&
        ((uint32_t)(now - motor->last_start_tick) >= UART_DMA_RETRY_INTERVAL_MS)) {
        (void)UART_DMA_StartForUart(&huart1);
    }
    if ((!camera->rx_active) &&
        ((uint32_t)(now - camera->last_start_tick) >= UART_DMA_RETRY_INTERVAL_MS)) {
        (void)UART_DMA_StartForUart(&huart2);
    }
    UART_DMA_UpdatePortState(&huart1, motor);
    UART_DMA_UpdatePortState(&huart2, camera);
    UART_DMA_UpdatePortState(&huart3, telemetry);
    g_uart_dma_debug.all_rx_active =
        (uint8_t)(motor->rx_active && camera->rx_active);
    g_uart_dma_debug.process_count++;
}

/**
 * @brief HAL ReceiveToIdle 的唯一全局接收事件回调。
 *
 * @details 回调执行顺序必须保持为：
 * 1. 在旧 DMA 缓冲仍有效时记录/解析本次 size 字节；
 * 2. 根据 UART 角色分发给 Motor_Feedback 或 Camera；
 * 3. 立即重新挂接下一次 DMA 接收，缩短接收空窗。
 *
 * USART1 原始数据会进入 motor_usart1.last_rx_data，并交给 Motor_Feedback 组帧、
 * 匹配最近一次发送命令的地址和功能码。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    UART_DMA_PortDebug_t *port = UART_DMA_GetPort(huart);

    if (port == NULL) {
        return;
    }

    if (huart == &huart1) {
        UART_DMA_RecordRx(huart, port, s_motor_rx_buffer, size);
        Motor_FeedbackOnRxData(s_motor_rx_buffer, size);
    } else if (huart == &huart2) {
        UART_DMA_RecordRx(huart, port, s_camera_rx_buffer, size);
        Camera_OnRxData(s_camera_rx_buffer, size);
    } else {
        /* USART3 当前为单向遥测 TX，不应进入 RX 事件。 */
        return;
    }

    (void)UART_DMA_StartForUart(huart);
}

/**
 * @brief HAL UART 的唯一全局错误恢复回调。
 *
 * @details 恢复步骤：
 * 1. 在 HAL 修改 ErrorCode 前保存错误值和时间；
 * 2. 通知 Camera 累计错误计数；
 * 3. 只中止 RX，不影响 USART1 可能正在进行的张大头 TX DMA；
 * 4. 通过读取 SR/DR 清除 F4 的 ORE/FE/NE/PE 类接收错误；
 * 5. 清 HAL 错误码并重新启动 ReceiveToIdle DMA。
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    UART_DMA_PortDebug_t *port = UART_DMA_GetPort(huart);
    uint32_t error_code;
    HAL_StatusTypeDef abort_status;

    if (port == NULL) {
        return;
    }

    error_code = huart->ErrorCode;
    port->uart_error_count++;
    port->last_uart_error_code = error_code;
    port->last_error_tick = HAL_GetTick();

    if (huart == &huart2) {
        Camera_OnUartError(error_code);
    } else if (huart == &huart3) {
        /* USART3 不启用 RX；只保留 HAL/DMA 状态供 Watch 诊断。 */
        __HAL_UART_CLEAR_OREFLAG(huart);
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        UART_DMA_UpdatePortState(huart, port);
        return;
    }

    abort_status = HAL_UART_AbortReceive(huart);
    port->last_abort_status = (uint8_t)abort_status;
    __HAL_UART_CLEAR_OREFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;
    (void)UART_DMA_StartForUart(huart);
}
