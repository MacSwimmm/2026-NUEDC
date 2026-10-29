#ifndef F407_BALANCE_MOTOR_FEEDBACK_H
#define F407_BALANCE_MOTOR_FEEDBACK_H

/**
 ******************************************************************************
 * @file    Motor_Feedback.h
 * @brief   张大头 USART1 串口回包捕获、组帧与期望应答匹配。
 *
 * @details
 * 商家命令注释约定回包以电机地址和功能码开头、以 0x6B 结尾。运动/使能类命令
 * 常见为 4 字节应答，读取实时位置等查询命令会携带更多数据。本模块不修改商家
 * Emm_V5 发送函数，只接收 UART_DMA.c 分发的 USART1 原始字节并保存到 Watch。
 *
 * 当前首先用于确认“驱动器是否实际收到了某条串口命令”。对状态字节和位置数据
 * 的物理含义暂不武断解释，保留完整原始帧，待实机回包确认后再增加字段级解析。
 ******************************************************************************
 */

#include "main.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_FEEDBACK_MAX_FRAME_SIZE  32U

typedef struct
{
    uint32_t rx_callback_count;       /* UART_DMA 分发 USART1 数据块的次数 */
    uint32_t rx_byte_count;           /* 累计收到的 USART1 原始字节数 */
    uint32_t frame_count;             /* 成功收到 0x6B 结尾帧的数量 */
    uint32_t ack_frame_count;         /* 长度为 4 的命令应答帧数量 */
    uint32_t ack_success_count;       /* 4 字节应答中状态码为 0x02 的数量 */
    uint32_t ack_error_count;         /* 收到应答，但状态码不是 0x02 */
    uint32_t position_frame_count;    /* 长度正确的 0x36 实时位置帧数量 */
    uint32_t matched_response_count;  /* 地址和功能码均匹配期望命令的帧数量 */
    uint32_t unexpected_frame_count; /* 完整但不匹配当前期望的帧数量 */
    uint32_t invalid_frame_count;     /* 已到规定长度但帧尾不是 0x6B */
    uint32_t discarded_byte_count;   /* 等待应答时为寻找正确帧头丢弃的字节数 */
    uint32_t overflow_count;          /* 单帧超过缓存长度后放弃的次数 */
    uint32_t request_count;           /* BeginExpectedResponse 调用次数 */
    uint32_t timeout_count;           /* WaitExpectedResponse 超时次数 */
    uint32_t last_rx_tick;            /* 最近收到任意 USART1 字节的 HAL tick */
    uint32_t last_frame_tick;         /* 最近完整帧的 HAL tick */

    uint8_t expected_active;          /* 当前是否正在等待指定地址/功能码 */
    uint8_t expected_address;         /* 正在等待的电机地址 */
    uint8_t expected_function;        /* 正在等待的功能码 */
    uint8_t expected_length;          /* 该功能码按协议应返回的固定长度 */
    uint8_t expected_received;        /* 已收到匹配帧 */
    uint8_t expected_status_ok;       /* 查询帧有效，或 4 字节应答状态为 0x02 */

    uint8_t last_frame_address;       /* 最近完整帧 byte[0] */
    uint8_t last_frame_function;      /* 最近完整帧 byte[1] */
    uint8_t last_frame_status;        /* 最近完整帧 byte[2] 原始值，不预设语义 */
    uint8_t last_frame_length;        /* last_frame 中的有效字节数 */
    uint8_t last_ack_status;          /* 最近 4 字节命令应答的状态码 */
    uint8_t last_ack_success;         /* last_ack_status 是否等于 0x02 */
    uint8_t last_frame[MOTOR_FEEDBACK_MAX_FRAME_SIZE];

    uint8_t position_valid;           /* 已收到并解析过严格 8 字节 0x36 帧 */
    uint8_t position_direction;       /* 0 为正方向，非 0 为负方向 */
    uint32_t position_raw;            /* byte[3..6] 拼出的无符号位置量 */
    int64_t position_signed_units;    /* 带方向位置量，65536 单位对应电机一圈 */
    float position_degrees;           /* 按商家例程换算的实时电机角度 */

    uint8_t partial_length;           /* 最近一次超时时尚未收全的字节数 */
    uint8_t partial_data[MOTOR_FEEDBACK_MAX_FRAME_SIZE];
} MotorFeedbackState_t;

/** @brief 非阻塞命令层查询当前期望 ACK 的结果。 */
typedef enum
{
    MOTOR_FEEDBACK_POLL_WAITING = 0,
    MOTOR_FEEDBACK_POLL_SUCCESS = 1,
    MOTOR_FEEDBACK_POLL_REJECTED = 2
} MotorFeedbackPollResult_t;

extern volatile MotorFeedbackState_t g_motor_feedback;

/** @brief 清空接收与应答匹配状态，不发送串口命令。 */
void Motor_FeedbackInit(void);

/** @brief 由 UART_DMA.c 在 USART1 ReceiveToIdle 回调中分发原始回包。 */
void Motor_FeedbackOnRxData(const uint8_t *data, uint16_t size);

/** @brief 发送命令前登记本次期望收到的电机地址和功能码。 */
void Motor_FeedbackBeginExpectedResponse(uint8_t address, uint8_t function);

/**
 * @brief 不等待、不延时地读取当前匹配结果。
 * @note 统一 Motor_Command 层在主循环轮询本接口；Balance 路径不得调用忙等接口。
 */
MotorFeedbackPollResult_t Motor_FeedbackPollExpectedResponse(void);

/**
 * @brief 取消当前期望并清除半帧。
 * @param count_timeout 非 0 时同时累计一次超时，供非阻塞管理层统一记账。
 */
void Motor_FeedbackCancelExpectedResponse(uint8_t count_timeout);

/**
 * @brief 等待匹配地址和功能码的完整回包。
 * @param timeout_ms 最大等待时间；中断仍可运行并调用 Motor_FeedbackOnRxData。
 * @retval true 已收到匹配帧；false 超时。
 */
bool Motor_FeedbackWaitExpectedResponse(uint32_t timeout_ms);

/** @brief 查询最近一次匹配应答是否表示驱动器接受了命令。 */
bool Motor_FeedbackExpectedResponseSucceeded(void);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_MOTOR_FEEDBACK_H */
