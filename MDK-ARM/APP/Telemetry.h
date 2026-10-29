#ifndef F407_BALANCE_TELEMETRY_H
#define F407_BALANCE_TELEMETRY_H

/**
 ******************************************************************************
 * @file    Telemetry.h
 * @brief   USART3 到电脑端的只读、非阻塞闭环遥测。
 ******************************************************************************
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define TELEMETRY_PROTOCOL_VERSION   2U
#define TELEMETRY_BAUD_RATE          115200U
#define TELEMETRY_SAMPLE_PERIOD_MS   50U
#define TELEMETRY_CONFIG_PERIOD_MS   5000U

typedef struct
{
    uint8_t enabled;
    uint8_t protocol_version;
    uint8_t tx_busy;
    uint8_t last_tx_status;
    uint16_t last_tx_len;
    uint16_t sample_period_ms;
    uint32_t sequence;
    uint32_t tx_frame_count;
    uint32_t tx_byte_count;
    uint32_t sample_frame_count;
    uint32_t config_frame_count;
    uint32_t hello_frame_count;
    uint32_t busy_observation_count;
    uint32_t tx_error_count;
    uint32_t build_error_count;
    uint32_t last_tx_tick;
} TelemetryDebug_t;

extern volatile TelemetryDebug_t g_telemetry_debug;

void Telemetry_Init(void);
void Telemetry_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_TELEMETRY_H */
