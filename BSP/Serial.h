#ifndef G3507_BASE_SERIAL_H
#define G3507_BASE_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file Serial.h
 * @brief 三路 UART 的有限超时阻塞发送封装。
 *
 * UART0 连接板载 CH340 并承接 printf；UART1/3 只提供透明发送能力。
 * 本工程没有串口接收运动命令的解析入口。
 */

/** 通过 UART0/CH340 发送 length 字节，成功返回 true。 */
bool Serial_DebugWrite(const uint8_t *data, uint16_t length);

/** 通过 UART1 透明发送，不解析或生成任何控制协议。 */
bool Serial_ZigBeeWrite(const uint8_t *data, uint16_t length);

/** 通过 UART3 向预留相机/视觉模块发送数据。 */
bool Serial_CameraWrite(const uint8_t *data, uint16_t length);

#endif
