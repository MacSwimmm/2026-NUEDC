#include "Board.h"

#include "ti_msp_dl_config.h"

#define SYSTICK_MAX_COUNT       0xFFFFFFUL
#define SYSTICK_FREQUENCY_HZ    80000000UL

static volatile uint32_t g_system_millis = 0U;
static BoardControlCallback_t g_control_callback = 0;
static uint8_t g_control_period_ms = 1U;

void Board_Init(BoardControlCallback_t control_callback, uint8_t control_period_ms)
{
    g_system_millis = 0U;
    g_control_callback = control_callback;
    g_control_period_ms = (control_period_ms == 0U) ? 1U : control_period_ms;

    /* PB6 是板载 W25Q128 的 CS，基础工程不使用 Flash，始终保持未选中。 */
    DL_GPIO_setPins(FLASH_GUARD_PORT, FLASH_GUARD_PIN_CS_PIN);
    Board_LEDSet(false);

    /* 先清除上电期间可能产生的挂起标志，再开放 NVIC。 */
    NVIC_ClearPendingIRQ(ENCODERA_INT_IRQN);
    NVIC_ClearPendingIRQ(ENCODERB_INT_IRQN);
    NVIC_ClearPendingIRQ(JY901S_UART_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(SYSTEM_TICK_INST_INT_IRQN);
    NVIC_ClearPendingIRQ(ADC12_VOLTAGE_INST_INT_IRQN);

    NVIC_EnableIRQ(ENCODERA_INT_IRQN);
    NVIC_EnableIRQ(ENCODERB_INT_IRQN);
    NVIC_EnableIRQ(JY901S_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(SYSTEM_TICK_INST_INT_IRQN);
    NVIC_EnableIRQ(ADC12_VOLTAGE_INST_INT_IRQN);
}

uint32_t Board_Millis(void)
{
    return g_system_millis;
}

void Board_DelayUs(uint32_t us)
{
    uint32_t ticks_per_us = SYSTICK_FREQUENCY_HZ / 1000000UL;
    uint32_t maximum_us = SYSTICK_MAX_COUNT / ticks_per_us;
    uint32_t start;
    uint32_t elapsed = 0U;

    if (us > maximum_us) us = maximum_us;
    start = SysTick->VAL;

    while (elapsed < (us * ticks_per_us)) {
        uint32_t current = SysTick->VAL;
        elapsed = (current <= start) ?
                  (start - current) :
                  (start + SYSTICK_MAX_COUNT - current);
    }
}

void Board_DelayMs(uint32_t ms)
{
    while (ms-- != 0U) Board_DelayUs(1000U);
}

void Board_LEDSet(bool enabled)
{
    if (enabled) DL_GPIO_setPins(BOARD_LED_PORT, BOARD_LED_PIN_LED_PIN);
    else DL_GPIO_clearPins(BOARD_LED_PORT, BOARD_LED_PIN_LED_PIN);
}

void Board_LEDToggle(void)
{
    DL_GPIO_togglePins(BOARD_LED_PORT, BOARD_LED_PIN_LED_PIN);
}

/**
 * @brief 进入极短临界区并返回进入前的全局中断状态。
 *
 * APP 只通过本 BSP 接口保护与中断共享的数据，不直接依赖 Cortex-M 内核头文件。
 * 调用者完成少量赋值后必须配对调用 Board_ExitCritical()。
 */
uint32_t Board_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    return primask;
}

void Board_ExitCritical(uint32_t previous_primask)
{
    if (previous_primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief TIMG0 统一 1 ms 时基中断。
 *
 * BSP 只保存一个应用层注册的 5 ms 回调，不直接依赖任何 APP/Algorithm 头文件，
 * 因而硬件层仍可单独复用。回调应保持短小，不能在其中刷新 OLED 或打印串口。
 */
void SYSTEM_TICK_INST_IRQHandler(void)
{
    static uint8_t control_divider = 0U;

    if (DL_TimerG_getPendingInterrupt(SYSTEM_TICK_INST) == DL_TIMER_IIDX_ZERO) {
        g_system_millis++;
        control_divider++;
        if (control_divider >= g_control_period_ms) {
            control_divider = 0U;
            if (g_control_callback != 0) g_control_callback();
        }
    }
}
