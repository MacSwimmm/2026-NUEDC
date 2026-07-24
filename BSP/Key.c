#include "Key.h"

#include "ti_msp_dl_config.h"

#define KEY_COUNT             3U
#define KEY_DEBOUNCE_SAMPLES  4U /* 4 × 5 ms = 20 ms */

typedef struct {
    uint8_t stable_pressed;
    uint8_t candidate_pressed;
    uint8_t candidate_count;
} KeyDebounce_t;

static KeyDebounce_t g_keys[KEY_COUNT];
static volatile uint8_t g_key_events = 0U;

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
                g_key_events |= event_bits[i];
            }
        }
    }
}

uint8_t Key_TakeEvents(void)
{
    uint8_t events;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    events = g_key_events;
    g_key_events = 0U;
    if (primask == 0U) {
        __enable_irq();
    }
    return events;
}
