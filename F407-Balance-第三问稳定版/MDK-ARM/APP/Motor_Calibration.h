#ifndef F407_BALANCE_MOTOR_CALIBRATION_H
#define F407_BALANCE_MOTOR_CALIBRATION_H

/**
 ******************************************************************************
 * @file    Motor_Calibration.h
 * @brief   张大头电机机械零位和双向软件限位的安全标定入口。
 *
 * @details
 * 生产闭环版本彻底禁用冷上电自动清零、自动使能和端点往返。所有 Watch 人工
 * 命令均申请 Motor_Command 的 CALIBRATION 所有权。新机构最终硬限位为右端
 * -340 pulse、左端 +330 pulse；人工命令 5/6 与 Balance 自动闭环均经过该硬门，
 * 临时探索外放已关闭，Watch 无权扩大范围。
 *
 * Motor_Feedback 会解析 S_CPOS 实时位置；这里的 position_pulses 仍是从人工水平
 * 零位开始累计的命令脉冲，不与编码器 65536 单位/圈的实际位置混用。标定必须
 * 使用低速、低电流，并目视确认机构没有失步、堵转或碰撞。
 ******************************************************************************
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 在 g_motor_calibration.command 中写入的单次命令编号。
 * @note 主循环读取后会立即把 command 清回 MOTOR_CAL_COMMAND_IDLE，避免重复执行。
 */
typedef enum
{
    MOTOR_CAL_COMMAND_IDLE = 0,
    MOTOR_CAL_COMMAND_STOP_DISABLE = 1,
    MOTOR_CAL_COMMAND_SET_ZERO = 2,
    MOTOR_CAL_COMMAND_ARM = 3,
    MOTOR_CAL_COMMAND_ENABLE = 4,
    MOTOR_CAL_COMMAND_JOG_POSITIVE = 5,  /* 增加正脉冲目标，小球向左移动 */
    MOTOR_CAL_COMMAND_JOG_NEGATIVE = 6,  /* 增加负脉冲目标，小球向右移动 */
    MOTOR_CAL_COMMAND_CAPTURE_LIMIT_A = 7,
    MOTOR_CAL_COMMAND_CAPTURE_LIMIT_B = 8,
    MOTOR_CAL_COMMAND_BUILD_SAFE_LIMITS = 9,
    MOTOR_CAL_COMMAND_GOTO_ZERO = 10,
    MOTOR_CAL_COMMAND_QUERY_POSITION = 11
} MotorCalibrationCommand_t;

/** @brief 最近一次 Watch 命令的执行结果。 */
typedef enum
{
    MOTOR_CAL_RESULT_IDLE = 0,
    MOTOR_CAL_RESULT_OK = 1,
    MOTOR_CAL_RESULT_REJECTED_ZERO_UNKNOWN = 2,
    MOTOR_CAL_RESULT_REJECTED_NOT_ARMED = 3,
    MOTOR_CAL_RESULT_REJECTED_NOT_ENABLED = 4,
    MOTOR_CAL_RESULT_REJECTED_PARAMETER = 5,
    MOTOR_CAL_RESULT_REJECTED_LIMIT = 6,
    MOTOR_CAL_RESULT_REJECTED_CAPTURE = 7,
    MOTOR_CAL_RESULT_TX_FAILED = 8,
    MOTOR_CAL_RESULT_RESPONSE_TIMEOUT = 9,
    MOTOR_CAL_RESULT_DEVICE_REJECTED = 10,
    MOTOR_CAL_RESULT_SWEEP_TIMEOUT = 11,
    MOTOR_CAL_RESULT_POSITION_INVALID = 12,
    MOTOR_CAL_RESULT_SWEEP_BUSY = 13
} MotorCalibrationResult_t;

/** @brief 历史往返阶段字段仅为 Watch 兼容保留；生产版本始终不自动启动。 */
typedef enum
{
    MOTOR_CAL_SWEEP_IDLE = 0,
    MOTOR_CAL_SWEEP_WAIT_ZERO = 1,
    MOTOR_CAL_SWEEP_ENABLE = 2,
    MOTOR_CAL_SWEEP_WAIT_MAX = 3,
    MOTOR_CAL_SWEEP_DWELL_MAX = 4,
    MOTOR_CAL_SWEEP_WAIT_MIN = 5,
    MOTOR_CAL_SWEEP_DWELL_MIN = 6,
    MOTOR_CAL_SWEEP_FAILED = 7,
    MOTOR_CAL_SWEEP_STOPPED = 8
} MotorCalibrationSweepPhase_t;

/**
 * @brief Keil Watch 标定状态与操作参数。
 *
 * @note 建议把整个 g_motor_calibration 加入 Watch。只写 command 和 jog_pulses；
 * 电机地址固定为 1，标定速度固定为 10 RPM，加速度固定为 0，探索外放量固定为
 * 0 pulse。后三项若被 Watch 修改，命令会被拒绝。
 */
typedef struct
{
    uint32_t command;                    /* 写入 MotorCalibrationCommand_t 后执行一次 */
    uint8_t motor_address;               /* 张大头地址，当前默认 1 */
    uint8_t velocity_rpm;                /* 只读镜像：标定点动固定 10 RPM */
    uint8_t acceleration;                /* 只读镜像：固定 0，表示低速直接启动 */
    uint8_t safety_percent;              /* 重新标定时的保留比例；固定限位默认 100% */
    uint32_t jog_pulses;                 /* 单次点动脉冲，默认 10，越过默认到达窗口 */
    uint32_t discovery_guard_pulses;     /* 只读镜像：0 表示最终限位外不再允许探索 */

    int32_t position_pulses;             /* 从人工零位累计的命令位置：正值球向左，负值球向右 */
    int32_t captured_limit_a_pulses;     /* 固化/重新记录的正向边界 */
    int32_t captured_limit_b_pulses;     /* 固化/重新记录的负向边界 */
    int32_t safe_min_pulses;             /* 人工捕获的负向边界镜像；Balance 不读取 */
    int32_t safe_max_pulses;             /* 人工捕获的正向边界镜像；Balance 不读取 */

    uint32_t executed_command_count;     /* 成功执行的 Watch 命令数 */
    uint32_t rejected_command_count;     /* 因状态/参数/限位被拒绝的命令数 */
    uint32_t tx_failed_count;             /* USART1 发送失败次数 */
    uint32_t last_tx_success_count;       /* 最近命令后 g_emm_debug 发送成功累计值 */
    uint32_t startup_zero_due_tick;       /* 冷上电自动清零的下一次尝试时间 */
    uint8_t zero_valid;                   /* 已在水平位置执行 SET_ZERO */
    uint8_t armed;                        /* 已执行 ARM；STOP 后自动清零 */
    uint8_t motor_enabled;                /* 本模块已发送使能命令 */
    uint8_t capture_a_valid;              /* A 边界已记录 */
    uint8_t capture_b_valid;              /* B 边界已记录 */
    uint8_t limits_valid;                 /* safe_min/max 已生成 */
    uint8_t last_command;                 /* 最近处理的命令编号 */
    uint8_t last_result;                  /* MotorCalibrationResult_t */
    uint8_t last_tx_status;               /* HAL_StatusTypeDef 原始值 */
    uint8_t startup_zero_requested;       /* 本次启动检测到冷上电复位 */
    uint8_t startup_zero_pending;         /* 等待驱动器稳定或等待重试 */
    uint8_t startup_zero_complete;        /* 已查询在线并成功将当前位置清零 */
    uint8_t startup_zero_attempt_count;   /* 当前冷上电清零尝试次数 */

    int32_t sweep_target_pulses;          /* 历史字段：当前最终边界为 +330 或 -340 */
    float sweep_target_degrees;           /* 按实测比例换算的当前目标角度 */
    float sweep_last_degrees;             /* 最近一次 S_CPOS 读回角度 */
    uint32_t sweep_due_tick;              /* 下一次位置查询或端点停留结束时间 */
    uint32_t sweep_move_start_tick;       /* 当前端点命令成功发送时间 */
    uint32_t sweep_cycle_count;           /* 完成“最大 -> 最小”的往返次数 */
    uint32_t sweep_verify_count;          /* 自动测试累计位置查询次数 */
    uint32_t sweep_failure_count;         /* 自动测试停止在 FAILED 的次数 */
    uint8_t sweep_phase;                  /* MotorCalibrationSweepPhase_t */
    uint8_t sweep_requested;              /* 冷上电成功清零后请求自动往返 */
    uint8_t sweep_active;                 /* 当前正在自动往返 */
    uint8_t sweep_in_tolerance_count;     /* 连续进入目标角度窗口的次数 */
} MotorCalibrationState_t;

extern volatile MotorCalibrationState_t g_motor_calibration;

/**
 * @brief 清空标定状态并装载保守默认参数。
 * @param auto_zero_on_power_on 为兼容旧调用保留，生产版本始终忽略并关闭自动动作。
 * @note 初始化函数只清状态，不发送命令。
 */
void Motor_CalibrationInit(bool auto_zero_on_power_on);

/** @brief 主循环轮询入口；command=0 时立即返回，不产生串口通信。 */
void Motor_CalibrationProcess(void);

/**
 * @brief 在正式限位建立后发送受保护的绝对位置命令。
 * @param target_pulses 以水平零点为 0；正脉冲使小球向左，负脉冲使小球向右。
 * @retval true 表示命令通过限位检查且 USART1 发送成功。
 * @note 当前未被 Balance_ControlLoop 调用，供后续闭环控制统一经过安全门使用。
 */
bool Motor_CalibrationCommandSafeAbsolute(int32_t target_pulses);

#ifdef __cplusplus
}
#endif

#endif /* F407_BALANCE_MOTOR_CALIBRATION_H */
