#ifndef G3507_BASE_SERIAL_H
#define G3507_BASE_SERIAL_H

#include <stdbool.h>
#include <stdint.h>

/* 三个接口都是有限超时的阻塞发送；UART0 同时承接 printf 输出。 */
bool Serial_DebugWrite(const uint8_t *data, uint16_t length);
bool Serial_ZigBeeWrite(const uint8_t *data, uint16_t length);
bool Serial_CameraWrite(const uint8_t *data, uint16_t length);

#endif
