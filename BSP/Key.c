#include "Key.h"

#include "ti_msp_dl_config.h"

#define KEY_COUNT             3U
#define KEY_DEBOUNCE_SAMPLES  4U /* 4 × 5 ms = 20 ms */

/* 每个按键独立保存“当前稳定值”和“正在确认的候选值”。 */
typedef struct {
    uint8_t stable_pressed;
    uint8_t candidate_pressed;
    uint8_t candidate_count;
} KeyDebounce_t;

static KeyDebounce_t g_keys[KEY_COUNT];
/* ISR 置位、主循环取走，使用位图可在一次循环中同时保留多个按键事件。 */
static volatile uint8_t g_key_events = 0U;

/** @brief 读取指定按键的瞬时状态，并统一转换为 1=按下、0=释放。 */
static uint8_t ReadPressed(uint8_t index)
{
    if (index == 0U) {
        return (DL_GPIO_readPins(USER_KEY_PORT, USER_KEY_PIN_KEY_PIN) == 0U) ? 1U : 0U;
    }
    if (index == 1U) {
        return (DL_GPIO_readPins(EXT_KEYS_PORT, EXT_KEYS_KEY1_PIN) == 0U) ? 1U : 0U;
    }
    return (DL_GPIO_readPins(EXT_KEYS_PORT, EXT_KEYS_KEY2_PIN) == 0U) ? 1U : 0U;
}

void Key_Init(void)
{
    uint8_t i;

    /* 不直接把上电瞬间电平当成稳定按下，必须经过后续完整消抖窗口。 */
    for (i = 0U; i < KEY_COUNT; i++) {
        g_keys[i].stable_pressed = 0U;
        g_keys[i].candidate_pressed = 0U;
        g_keys[i].candidate_count = 0U;
    }
    g_key_events = 0U;
}

/**
 * @brief 每 5 ms 调用一次的按键消抖。
 *
 * 连续 20 ms 读到同一电平才确认状态变化，并且只在“释放→按下”时产生一次
 * 事件。长按不会连续触发，因此应用层不需要额外的按键状态机。
 */
void Key_Scan5ms(void)
{
    static const uint8_t event_bits[KEY_COUNT] = {
        KEY_EVENT_USER, KEY_EVENT_KEY1, KEY_EVENT_KEY2
    };
    uint8_t i;

    for (i = 0U; i < KEY_COUNT; i++) {
        uint8_t pressed = ReadPressed(i);
        KeyDebounce_t *key = &g_keys[i];

        if (pressed != key->candidate_pressed) {
            /* 电平发生变化，重新开始连续采样计数。 */
            key->candidate_pressed = pressed;
            key->candidate_count = 1U;
            continue;
        }

        if (key->candidate_count < KEY_DEBOUNCE_SAMPLES) {
            key->candidate_count++;
        }
        if ((key->candidate_count >= KEY_DEBOUNCE_SAMPLES) &&
            (key->stable_pressed != key->candidate_pressed)) {
            key->stable_pressed = key->candidate_pressed;
            if (key->stable_pressed != 0U) {
                /* 只在稳定状态切换到“按下”时置位；释放不产生应用事件。 */
                g_key_events |= event_bits[i];
            }
        }
    }
}

uint8_t Key_TakeEvents(void)
{
    uint8_t events;
    uint32_t primask = __get_PRIMASK();

    /* 读取与清零必须不可分割，否则 ISR 可能在两条语句之间置位而被覆盖。 */
    __disable_irq();
    events = g_key_events;
    g_key_events = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return events;
}
