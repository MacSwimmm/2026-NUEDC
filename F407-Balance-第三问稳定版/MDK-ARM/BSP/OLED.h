#ifndef F407_BALANCE_OLED_H
#define F407_BALANCE_OLED_H

/**
 ******************************************************************************
 * @file    OLED.h
 * @brief   SSD1306 128x64 四针 I2C OLED 的低优先级 DMA 与自动恢复层。
 *
 * PB6=SCL、PB7=SDA，7 位地址 0x3C。正常刷新完全使用 I2C1 TX DMA；NACK、
 * BUSY、DMA/HAL 错误或超时只会让 OLED 进入恢复等待，主循环随后释放总线、输出
 * 9 个 SCL 脉冲、重新初始化 I2C1 和 SSD1306，不会永久停在最后一帧。
 ******************************************************************************
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OLED_WIDTH                    128U
#define OLED_HEIGHT                    64U
#define OLED_FRAME_BUFFER_SIZE       1024U
#define OLED_FRAME_PACKET_SIZE       1025U
#define OLED_DATA_CONTROL_BYTE       0x40U
#define OLED_I2C_ADDRESS_7BIT        0x3CU

typedef enum
{
    OLED_DRIVER_DISABLED = 0,
    OLED_DRIVER_WAIT_POWER,
    OLED_DRIVER_INIT_TX,
    OLED_DRIVER_READY,
    OLED_DRIVER_FRAME_TX,
    OLED_DRIVER_RECOVERY_WAIT,
    OLED_DRIVER_RECOVERY_BUS
} OLED_DriverState_t;

/** @brief Keil Watch 中用于确认 OLED 非阻塞刷新和自动恢复的只读镜像。 */
typedef struct
{
    uint8_t enabled;
    uint8_t initialized;
    uint8_t tx_busy;
    uint8_t state;
    uint8_t transfer_kind;
    uint8_t address_7bit;
    uint8_t last_hal_status;
    uint8_t consecutive_fault_count;
    uint32_t init_count;
    uint32_t frame_count;
    uint32_t start_fail_count;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t recovery_count;
    uint32_t recovery_attempt_count;
    uint32_t recovery_success_count;
    uint32_t recovery_fail_count;
    uint32_t last_error_code;
    uint32_t last_tx_start_tick;
    uint32_t last_tx_complete_tick;
    uint32_t last_recovery_tick;
} OLED_Debug_t;

extern volatile OLED_Debug_t g_oled_debug;

/** @brief 清空驱动状态；上电等待、初始化和后续恢复均由 OLED_Process 推进。 */
void OLED_Init(void);

/** @brief 主循环末尾高频调用；不等待 DMA，故障恢复只在这里执行。 */
void OLED_Process(uint32_t now);

/** @retval 1=SSD1306 已初始化且 I2C1 可立即接收一帧；否则为 0。 */
uint8_t OLED_CanAcceptFrame(void);

/** @brief 启动一帧 1025 字节 I2C1 TX DMA 并立即返回。 */
uint8_t OLED_StartFrame(uint8_t *packet, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_OLED_H */
