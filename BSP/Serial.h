#ifndef G3507_BASE_SERIAL_H
#define G3507_BASE_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file Serial.h
 * @brief UART0/1 基础收发与 UART3 非阻塞遥测发送封装。
 *
 * UART0 连接板载 CH340 并承接 printf；RX 中断只把字节写入固定环形缓冲，
 * 上层再通过非阻塞接口取走并解析。UART1 保留透明发送；UART3/PB12
 * 专用于无线 DAP 遥测，采用软件环形缓冲和最低优先级 TX 中断发送。
 */

/** 通过 UART0/CH340 发送 length 字节，成功返回 true。 */
bool Serial_DebugWrite(const uint8_t *data, uint16_t length);

/** 清空 UART0 接收状态并开放 RX 中断；应在 SYSCFG_DL_init() 后调用一次。 */
void Serial_DebugRxInit(void);

/** 环形缓冲中有数据时读出一个字节；无数据或参数无效时返回 false。 */
bool Serial_DebugReadByte(uint8_t *data);

/** 通过 UART1 透明发送，不解析或生成任何控制协议。 */
bool Serial_ZigBeeWrite(const uint8_t *data, uint16_t length);

/** 清空 UART3 遥测队列、关闭 RX 并开放最低优先级 TX 中断。 */
void Serial_TelemetryInit(void);

/**
 * 将一整帧压入 UART3 发送队列；空间不足时不写入任何字节并返回 false。
 * 该接口不等待串口移位发送，可在比赛主循环中安全调用。
 */
bool Serial_TelemetryWrite(const uint8_t *data, uint16_t length);

/** 返回仍在 UART3 软件队列中等待发送的字节数，供 Watch/遥测自检使用。 */
uint16_t Serial_TelemetryPendingBytes(void);

#endif
