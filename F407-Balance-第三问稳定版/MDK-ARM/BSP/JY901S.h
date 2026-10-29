#ifndef F407_BALANCE_JY901S_H
#define F407_BALANCE_JY901S_H

/**
 ******************************************************************************
 * @file    JY901S.h
 * @brief   JY901S/WIT 标准串口协议解析与姿态数据快照接口。
 *
 * @details
 * 标准普通输出帧固定为 11 字节：
 *   Byte0      0x55 帧头
 *   Byte1      数据类型（0x51 加速度、0x52 角速度、0x53 角度、0x54 磁场）
 *   Byte2~9    4 个小端 int16_t 数据
 *   Byte10     Byte0~Byte9 的低 8 位累加和
 *
 * 本模块只解析传感器已经输出的数据，不负责修改 JY901S 的波特率、输出频率
 * 或保存参数。USART3 配置为 115200 时，传感器本体也必须是 115200。
 ******************************************************************************
 */

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 单次 DMA 接收缓冲区可同时容纳多种连续输出帧。 */
#define JY901S_RX_DMA_BUFFER_SIZE    128U
/* WIT 普通协议固定帧长，修改会直接破坏解析状态机。 */
#define JY901S_FRAME_SIZE            11U
/* 超过 250 ms 没有收到受支持且校验正确的数据帧，就判定传感器离线。 */
#define JY901S_ONLINE_TIMEOUT_MS     250U

/* update_flag 位定义：上层可只处理自己关心、且刚刚更新的数据种类。 */
#define JY901S_UPDATE_ACC            0x01U
#define JY901S_UPDATE_GYRO           0x02U
#define JY901S_UPDATE_ANGLE          0x04U
#define JY901S_UPDATE_MAG            0x08U
#define JY901S_UPDATE_TEMP           0x10U

/**
 * @brief 已完成比例换算、可直接供控制/显示层使用的数据。
 */
typedef struct
{
    float acc[3];                    /* X/Y/Z 加速度，单位 g，量程按 ±16 g 换算 */
    float gyro[3];                   /* X/Y/Z 角速度，单位 °/s，按 ±2000 °/s 换算 */
    float angle[3];                  /* Roll/Pitch/Yaw，单位 °，按 ±180° 换算 */
    int16_t mag[3];                  /* X/Y/Z 磁场原始值；未在此处换算成物理单位 */
    float temperature;               /* 0x51 帧中的温度，单位 ℃ */
    uint32_t update_flag;            /* JY901S_UPDATE_* 的按位或结果 */
    uint32_t last_update_tick;       /* 最近有效且受支持帧的 HAL 毫秒计数 */
    uint32_t acc_update_sequence;    /* 每收到一帧 0x51 加一，用于判断是否有新加速度 */
    uint32_t gyro_update_sequence;   /* 每收到一帧 0x52 加一，用于判断是否有新角速度 */
    uint8_t online;                  /* 1=近期有有效帧；0=从未收到或已经超时 */
} JY901S_Data_t;

/**
 * @brief JY901S 接收、组帧、校验与换算的完整调试镜像。
 *
 * @note 在 Keil Watch 中添加 g_jy901s_debug。raw_* 用于确认字节序和原始值，
 * data 用于观察换算结果，计数器用于定位是“没收到”“校验失败”还是“帧类型未启用”。
 */
typedef struct
{
    JY901S_Data_t data;               /* 换算后的最新数据快照 */
    int16_t raw_acc[3];               /* 最近一帧 0x51 的 X/Y/Z 原始值 */
    int16_t raw_gyro[3];              /* 最近一帧 0x52 的 X/Y/Z 原始值 */
    int16_t raw_angle[3];             /* 最近一帧 0x53 的 Roll/Pitch/Yaw 原始值 */
    int16_t raw_mag[3];               /* 最近一帧 0x54 的 X/Y/Z 原始值 */
    int16_t raw_temperature;          /* 最近一帧 0x51 的温度原始值 */
    uint32_t rx_chunk_count;          /* UART_DMA 提交数据块的次数 */
    uint32_t rx_byte_count;           /* USART3 累计收到的原始字节数 */
    uint32_t valid_frame_count;       /* 帧头和校验均正确的总帧数 */
    uint32_t acc_frame_count;         /* 0x51 帧数 */
    uint32_t gyro_frame_count;        /* 0x52 帧数 */
    uint32_t angle_frame_count;       /* 0x53 帧数 */
    uint32_t mag_frame_count;         /* 0x54 帧数 */
    uint32_t unsupported_frame_count; /* 校验正确但当前未解析的数据类型数量 */
    uint32_t checksum_error_count;    /* 11 字节成帧后校验不一致的次数 */
    uint32_t discarded_byte_count;    /* 等待帧头时丢弃的非 0x55 字节数 */
    uint32_t uart_error_count;        /* USART3 HAL 错误回调次数 */
    uint32_t last_uart_error_code;    /* 最近一次 huart3.ErrorCode */
    uint32_t last_rx_tick;            /* 最近收到任意 DMA 数据块的时间 */
    uint32_t last_error_tick;         /* 最近发生 UART 错误的时间 */
    uint16_t last_rx_len;             /* 最近一个 DMA 数据块的长度 */
    uint8_t parser_index;             /* 当前已收集到的帧字节数，正常范围 0~10 */
    uint8_t last_frame_type;          /* 最近校验正确帧的 Byte1 类型码 */
    uint8_t last_checksum_calculated; /* STM32 根据 Byte0~9 算出的校验和 */
    uint8_t last_checksum_received;   /* 传感器帧 Byte10 携带的校验和 */
    uint8_t last_frame[JY901S_FRAME_SIZE]; /* 最近一次凑满的 11 字节原始帧 */
} JY901S_Debug_t;

/* volatile：解析器在 USART3 中断上下文更新这些字段。 */
extern volatile JY901S_Debug_t g_jy901s_debug;

/** @brief 清空组帧状态、姿态数据和调试计数；启动 DMA 前调用一次。 */
void JY901S_Init(void);
/** @brief 主循环周期调用，根据最后有效帧时间维护 online。 */
void JY901S_Process(void);
/** @brief 在 USART3 接收回调中提交任意长度的原始字节流。 */
void JY901S_OnRxData(const uint8_t *data, uint16_t size);
/** @brief 记录 USART3 错误；DMA 恢复由 UART_DMA 模块负责。 */
void JY901S_OnUartError(uint32_t error_code);
/** @brief 临界区复制一致快照；返回 1 表示传感器当前在线。 */
uint8_t JY901S_GetData(JY901S_Data_t *data);
/** @brief 返回尚未被上层清除的 JY901S_UPDATE_* 标志。 */
uint32_t JY901S_GetUpdateFlag(void);
/** @brief 按位清除已经消费的数据更新标志，不会清除实际测量值。 */
void JY901S_ClearUpdateFlag(uint32_t flag);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_JY901S_H */
