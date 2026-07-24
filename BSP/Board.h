#ifndef G3507_BASE_BOARD_H
#define G3507_BASE_BOARD_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @file Board.h
 * @brief 板级公共服务：中断开放、毫秒时基、短延时、LED 与临界区。
 *
 * Board 层不包含 APP 头文件。应用通过函数指针注册控制回调，从而保持
 * “APP 依赖 BSP、BSP 不反向依赖 APP”的分层关系。
 */

/** 控制周期回调类型；回调在 1 ms 定时器中断派生的中断上下文中执行。 */
typedef void (*BoardControlCallback_t)(void);

/**
 * 保存控制回调并开放工程实际使用的 NVIC 中断。
 * @param control_callback  周期控制函数，允许传入空指针表示不执行控制回调
 * @param control_period_ms 回调周期，单位 ms；传 0 时按 1 ms 处理
 * @note 调用前必须先执行 SYSCFG_DL_init() 和各 BSP/APP 初始化。
 */
void Board_Init(BoardControlCallback_t control_callback, uint8_t control_period_ms);

/** 返回自 Board_Init 后累计的毫秒数，uint32_t 回绕可用无符号相减处理。 */
uint32_t Board_Millis(void);

/** SysTick 轮询短延时；单次最大值受 24 位 SysTick 计数范围限制。 */
void Board_DelayUs(uint32_t us);

/** 基于 Board_DelayUs 的阻塞毫秒延时，只用于初始化，不应放进控制中断。 */
void Board_DelayMs(uint32_t ms);

/** 设置板载 PB22 LED；true 点亮、false 熄灭。 */
void Board_LEDSet(bool enabled);

/** 翻转板载 PB22 LED 输出状态。 */
void Board_LEDToggle(void);

/** 关闭全局中断并返回进入前 PRIMASK，供极短共享数据访问使用。 */
uint32_t Board_EnterCritical(void);

/** 按进入前 PRIMASK 恢复中断状态，必须与 Board_EnterCritical 成对调用。 */
void Board_ExitCritical(uint32_t previous_primask);

#endif
