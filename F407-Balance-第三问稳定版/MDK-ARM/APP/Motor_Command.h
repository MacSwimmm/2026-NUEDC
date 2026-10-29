#ifndef F407_BALANCE_MOTOR_COMMAND_H
#define F407_BALANCE_MOTOR_COMMAND_H

/**
 ******************************************************************************
 * @file    Motor_Command.h
 * @brief   USART1 张大头命令唯一所有权、非阻塞 ACK 与机械硬限位层。
 *
 * @details
 * 所有自动控制和人工标定的绝对位置命令都必须经过本模块。新机构实测生产硬限位
 * 固定为右端 -340 pulse、左端 +330 pulse，Watch、标定镜像和 PID 参数均无权
 * 扩大。临时人工探索外放已经关闭，CALIBRATION 与 Balance 使用同一最终硬门。
 ******************************************************************************
 */

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_HARD_MIN_PULSE   (-340)  /* 右端：负脉冲 */
#define MOTOR_HARD_MAX_PULSE   330     /* 左端：正脉冲 */
#define MOTOR_CAL_DISCOVERY_EXTRA_PULSE  0
#define MOTOR_CAL_DISCOVERY_MIN_PULSE \
    (MOTOR_HARD_MIN_PULSE - MOTOR_CAL_DISCOVERY_EXTRA_PULSE)
#define MOTOR_CAL_DISCOVERY_MAX_PULSE \
    (MOTOR_HARD_MAX_PULSE + MOTOR_CAL_DISCOVERY_EXTRA_PULSE)
#define MOTOR_CALIBRATION_POSITION_VELOCITY_RPM  10U
#define MOTOR_COMMAND_ADDRESS  1U

typedef enum
{
    MOTOR_OWNER_NONE = 0,
    MOTOR_OWNER_BALANCE = 1,
    MOTOR_OWNER_CALIBRATION = 2,
    MOTOR_OWNER_SAFETY = 3
} MotorOwner_t;

typedef enum
{
    MOTOR_COMMAND_NONE = 0,
    MOTOR_COMMAND_QUERY_POSITION,
    MOTOR_COMMAND_RESET_ZERO,
    MOTOR_COMMAND_ENABLE,
    MOTOR_COMMAND_STOP,
    MOTOR_COMMAND_DISABLE,
    MOTOR_COMMAND_ABSOLUTE_POSITION
} MotorCommandType_t;

typedef enum
{
    MOTOR_COMMAND_STATE_IDLE = 0,
    MOTOR_COMMAND_STATE_WAIT_TX_COMPLETE,
    MOTOR_COMMAND_STATE_WAIT_ACK,
    MOTOR_COMMAND_STATE_RETRY_PENDING
} MotorCommandState_t;

typedef enum
{
    MOTOR_REQUEST_UNKNOWN = 0,
    MOTOR_REQUEST_PENDING,
    MOTOR_REQUEST_SUCCESS,
    MOTOR_REQUEST_TX_FAILED,
    MOTOR_REQUEST_ACK_TIMEOUT,
    MOTOR_REQUEST_DEVICE_REJECTED,
    MOTOR_REQUEST_HARD_LIMIT_REJECTED,
    MOTOR_REQUEST_OWNER_REJECTED,
    MOTOR_REQUEST_SUPERSEDED,
    MOTOR_REQUEST_PREEMPTED
} MotorRequestResult_t;

typedef struct
{
    uint8_t owner;
    uint8_t state;
    uint8_t command_type;
    uint8_t retry_count;
    uint32_t request_id;
    uint32_t tx_tick;
    uint32_t ack_deadline;
    uint8_t ack_status;
    uint8_t busy;
    uint8_t pending_latest_valid;
    uint8_t last_completed_result;
    uint32_t last_completed_request_id;
    uint8_t last_completed_type;
    uint8_t last_completed_owner;
    int32_t pending_latest_pulse;
    uint32_t pending_latest_request_id;
    int32_t last_transmitted_pulse;
    int32_t last_acked_pulse;
    uint32_t tx_fail_count;
    uint32_t ack_timeout_count;
    uint32_t ack_reject_count;
    uint32_t retry_total_count;
    uint32_t busy_defer_count;
    uint32_t hard_limit_reject_count;
    uint32_t owner_reject_count;
    uint32_t safety_preempt_count;
} MotorCommandDebug_t;

extern volatile MotorCommandDebug_t g_motor_command_debug;

void Motor_CommandInit(void);
void Motor_CommandProcess(void);
uint8_t Motor_CommandAcquire(MotorOwner_t owner);
uint8_t Motor_CommandRelease(MotorOwner_t owner);
void Motor_CommandSafetyTakeover(void);
MotorOwner_t Motor_CommandGetOwner(void);
uint8_t Motor_CommandIsBusy(void);

uint32_t Motor_CommandRequestQueryPosition(MotorOwner_t owner);
uint32_t Motor_CommandRequestResetZero(MotorOwner_t owner);
uint32_t Motor_CommandRequestEnable(MotorOwner_t owner);
uint32_t Motor_CommandRequestStop(MotorOwner_t owner);
uint32_t Motor_CommandRequestDisable(MotorOwner_t owner);
uint32_t Motor_CommandRequestAbsolute(MotorOwner_t owner, int32_t target_pulses);
MotorRequestResult_t Motor_CommandGetResult(uint32_t request_id);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_MOTOR_COMMAND_H */
