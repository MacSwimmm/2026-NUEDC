#include "Serial.h"

#include <stdio.h>

#include "ti_msp_dl_config.h"

/* 防止外设异常时永久卡死；这是循环次数上限，不是精确的毫秒超时。 */
#define UART_TX_TIMEOUT_LOOPS 100000U
/* 必须为 2 的幂；127 字节有效容量足以容纳最长 95 字节调参命令。 */
#define DEBUG_RX_BUFFER_LENGTH 128U
#define DEBUG_RX_BUFFER_MASK   (DEBUG_RX_BUFFER_LENGTH - 1U)
/*
 * UART3 遥测发送环形缓冲。511 字节有效容量可容纳 6 个完整遥测帧，
 * 即使 OLED 刷新暂时占用主循环，低优先级 TX 中断也能继续把数据送出。
 */
#define TELEMETRY_TX_BUFFER_LENGTH 512U
#define TELEMETRY_TX_BUFFER_MASK   (TELEMETRY_TX_BUFFER_LENGTH - 1U)
#define TELEMETRY_UART_IRQ_PRIORITY 3U

/* ISR 只收字节进环形缓冲，协议解析仍在 APP 主循环中完成。 */
static volatile uint8_t g_debug_rx_buffer[DEBUG_RX_BUFFER_LENGTH];
static volatile uint8_t g_debug_rx_head = 0U;
static volatile uint8_t g_debug_rx_tail = 0U;
static volatile uint32_t g_debug_rx_overflow_count = 0U;

/*
 * UART3 采用单生产者（主循环）/单消费者（UART3 ISR）环形缓冲。
 * head 只由主循环更新，tail 只由中断更新；写入整帧前先检查剩余空间，
 * 空间不足时整帧拒绝，绝不发送半帧破坏电脑端同步。
 */
static volatile uint8_t g_telemetry_tx_buffer[TELEMETRY_TX_BUFFER_LENGTH];
static volatile uint16_t g_telemetry_tx_head = 0U;
static volatile uint16_t g_telemetry_tx_tail = 0U;

/** @brief UART0/UART1 共用的有限超时逐字节发送实现。 */
static bool WritePort(UART_Regs *instance, const uint8_t *data, uint16_t length)
{
    uint16_t i;

    /* 长度为 0 时允许 data=NULL；有数据时空指针直接判错。 */
    if ((data == NULL) && (length != 0U)) return false;

    for (i = 0U; i < length; i++) {
        uint32_t timeout = UART_TX_TIMEOUT_LOOPS;

        /* 仅等待 FIFO 出现空位，不等待最后一个字节完全移出 TX 引脚。 */
        while (DL_UART_Main_isTXFIFOFull(instance)) {
            if (--timeout == 0U) return false;
        }
        DL_UART_Main_transmitData(instance, data[i]);
    }
    return true;
}

bool Serial_DebugWrite(const uint8_t *data, uint16_t length)
{
    return WritePort(DEBUG_UART_INST, data, length);
}

void Serial_DebugRxInit(void)
{
    /* 初始化期间先关闭接收中断，丢弃上电噪声，再开放 UART0 NVIC。 */
    NVIC_DisableIRQ(DEBUG_UART_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
        (void)DL_UART_Main_receiveData(DEBUG_UART_INST);
    }
    g_debug_rx_head = 0U;
    g_debug_rx_tail = 0U;
    g_debug_rx_overflow_count = 0U;
    NVIC_ClearPendingIRQ(DEBUG_UART_INST_INT_IRQN);
    DL_UART_Main_enableInterrupt(DEBUG_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_EnableIRQ(DEBUG_UART_INST_INT_IRQN);
}

bool Serial_DebugReadByte(uint8_t *data)
{
    uint8_t tail = g_debug_rx_tail;

    if ((data == NULL) || (tail == g_debug_rx_head)) return false;
    *data = g_debug_rx_buffer[tail];
    g_debug_rx_tail = (uint8_t)((tail + 1U) & DEBUG_RX_BUFFER_MASK);
    return true;
}

void DEBUG_UART_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(DEBUG_UART_INST) == DL_UART_MAIN_IIDX_RX) {
        while (!DL_UART_Main_isRXFIFOEmpty(DEBUG_UART_INST)) {
            uint8_t byte = DL_UART_Main_receiveData(DEBUG_UART_INST);
            uint8_t head = g_debug_rx_head;
            uint8_t next = (uint8_t)((head + 1U) & DEBUG_RX_BUFFER_MASK);

            if (next != g_debug_rx_tail) {
                g_debug_rx_buffer[head] = byte;
                g_debug_rx_head = next;
            } else {
                /* 缓冲已满时丢弃最新字节并累计诊断计数，绝不覆盖尚未解析的数据。 */
                g_debug_rx_overflow_count++;
            }
        }
    }
}

/** @brief 返回 UART3 环形缓冲中尚未写入硬件 FIFO 的字节数。 */
uint16_t Serial_TelemetryPendingBytes(void)
{
    return (uint16_t)((g_telemetry_tx_head - g_telemetry_tx_tail) &
                      TELEMETRY_TX_BUFFER_MASK);
}

/**
 * @brief 尽可能把 UART3 软件缓冲中的字节装入硬件 TX FIFO。
 *
 * 该函数只能在 UART3 NVIC 已关闭的主循环临界段或 UART3 ISR 内调用。
 * 队列清空后立即关闭 TX 中断，避免空 FIFO 持续触发无意义中断。
 */
static void FillTelemetryTxFifo(void)
{
    uint16_t tail = g_telemetry_tx_tail;
    uint16_t head = g_telemetry_tx_head;

    while ((tail != head) &&
           !DL_UART_Main_isTXFIFOFull(TELEMETRY_UART_INST)) {
        DL_UART_Main_transmitData(
            TELEMETRY_UART_INST, g_telemetry_tx_buffer[tail]);
        tail = (uint16_t)((tail + 1U) & TELEMETRY_TX_BUFFER_MASK);
    }
    g_telemetry_tx_tail = tail;

    if (tail == head) {
        DL_UART_Main_disableInterrupt(
            TELEMETRY_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    } else {
        DL_UART_Main_enableInterrupt(
            TELEMETRY_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    }
}

void Serial_TelemetryInit(void)
{
    /*
     * UART3 只发送，不开放 RX，也不解析任何电脑命令。中断优先级 3 低于
     * 编码器、JY901S 和 1 ms/5 ms 控制时基，因此遥测绝不能阻挡运动控制。
     */
    NVIC_DisableIRQ(TELEMETRY_UART_INST_INT_IRQN);
    DL_UART_Main_disableInterrupt(
        TELEMETRY_UART_INST, DL_UART_MAIN_INTERRUPT_TX);
    g_telemetry_tx_head = 0U;
    g_telemetry_tx_tail = 0U;
    NVIC_ClearPendingIRQ(TELEMETRY_UART_INST_INT_IRQN);
    NVIC_SetPriority(TELEMETRY_UART_INST_INT_IRQN,
                     TELEMETRY_UART_IRQ_PRIORITY);
    NVIC_EnableIRQ(TELEMETRY_UART_INST_INT_IRQN);
}

bool Serial_TelemetryWrite(const uint8_t *data, uint16_t length)
{
    uint16_t head;
    uint16_t used;
    uint16_t free_bytes;
    uint16_t i;

    if ((data == NULL) && (length != 0U)) return false;
    if (length == 0U) return true;
    if (length >= TELEMETRY_TX_BUFFER_LENGTH) return false;

    /*
     * 这里只屏蔽 UART3 自己的 NVIC，编码器和 5 ms 控制中断仍可抢占。
     * 因而复制一个约 74 字节的遥测帧不会改变底盘控制周期。
     */
    NVIC_DisableIRQ(TELEMETRY_UART_INST_INT_IRQN);
    head = g_telemetry_tx_head;
    used = (uint16_t)((head - g_telemetry_tx_tail) &
                      TELEMETRY_TX_BUFFER_MASK);
    free_bytes = (uint16_t)(TELEMETRY_TX_BUFFER_MASK - used);
    if (length > free_bytes) {
        NVIC_EnableIRQ(TELEMETRY_UART_INST_INT_IRQN);
        return false;
    }

    for (i = 0U; i < length; i++) {
        g_telemetry_tx_buffer[head] = data[i];
        head = (uint16_t)((head + 1U) & TELEMETRY_TX_BUFFER_MASK);
    }
    /* head 最后提交，ISR 不可能看到尚未复制完成的半帧。 */
    g_telemetry_tx_head = head;
    FillTelemetryTxFifo();
    NVIC_EnableIRQ(TELEMETRY_UART_INST_INT_IRQN);
    return true;
}

void TELEMETRY_UART_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(TELEMETRY_UART_INST)) {
        case DL_UART_MAIN_IIDX_TX:
            FillTelemetryTxFifo();
            break;
        default:
            break;
    }
}

bool Serial_ZigBeeWrite(const uint8_t *data, uint16_t length)
{
    /* 这里只提供透明串口发送，不实现任何接收命令解析。 */
    return WritePort(ZIGBEE_UART_INST, data, length);
}

/**
 * @brief 将 printf 固定重定向到板载 CH340 的 UART0。
 *
 * 调试打印不会发送到无线串口；UART0 接收数据由 APP 层调参协议按需解析。
 */
int fputc(int ch, FILE *stream)
{
    uint8_t byte = (uint8_t)ch;
    (void)stream;
    (void)Serial_DebugWrite(&byte, 1U);
    return ch;
}
