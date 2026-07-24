#include "Serial.h"

#include <stdio.h>

#include "ti_msp_dl_config.h"

/* 防止外设异常时永久卡死；这是循环次数上限，不是精确的毫秒超时。 */
#define UART_TX_TIMEOUT_LOOPS 100000U

/** @brief 三路串口共用的逐字节发送实现。 */
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

bool Serial_ZigBeeWrite(const uint8_t *data, uint16_t length)
{
    /* 这里只提供透明串口发送，不实现任何接收命令解析。 */
    return WritePort(ZIGBEE_UART_INST, data, length);
}

bool Serial_CameraWrite(const uint8_t *data, uint16_t length)
{
    return WritePort(CAMERA_UART_INST, data, length);
}

/**
 * @brief 将 printf 固定重定向到板载 CH340 的 UART0。
 *
 * 调试打印不会发送到无线串口，也不存在通过串口修改电机目标或 PWM 的入口。
 */
int fputc(int ch, FILE *stream)
{
    uint8_t byte = (uint8_t)ch;
    (void)stream;
    (void)Serial_DebugWrite(&byte, 1U);
    return ch;
}
