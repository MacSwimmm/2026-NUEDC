#include "Track_Keys.h"
#include "main.h"

/* PE3/PE2 两键统一消抖；PE2 短按在松手时产生，避免长按先误触发短按。 */
bool g_track_key_pressed[TRACK_KEY_COUNT];
uint8_t g_track_key_stable[TRACK_KEY_COUNT];

#define BSP_TRACK_KEY_DEBOUNCE_MS             30U
#define BSP_TRACK_KEY_OK_LONG_MS             750U

typedef struct {
    uint8_t raw_pressed;
    uint8_t stable_pressed;
    uint8_t long_sent;
    uint32_t raw_change_tick;
    uint32_t press_tick;
} TrackKeyState_t;

static TrackKeyState_t s_keys[TRACK_KEY_COUNT];
static uint8_t s_key_events;

static bool track_key_raw(track_key_id_t key)
{
    static GPIO_TypeDef *const ports[TRACK_KEY_COUNT] = {
        MENU_NEXT_BUTTON_GPIO_Port,
        TARGET_LOCK_BUTTON_GPIO_Port
    };
    static const uint16_t pins[TRACK_KEY_COUNT] = {
        MENU_NEXT_BUTTON_Pin,
        TARGET_LOCK_BUTTON_Pin
    };

    return ((uint32_t)key < (uint32_t)TRACK_KEY_COUNT) &&
           (HAL_GPIO_ReadPin(ports[key], pins[key]) == GPIO_PIN_RESET);
}

void bsp_track_key_init(void)
{
    uint8_t i;
    uint32_t now = HAL_GetTick();

    for (i = 0U; i < TRACK_KEY_COUNT; i++) {
        uint8_t pressed = track_key_raw((track_key_id_t)i) ? 1U : 0U;
        s_keys[i].raw_pressed = pressed;
        s_keys[i].stable_pressed = pressed;
        s_keys[i].long_sent = 0U;
        s_keys[i].raw_change_tick = now;
        s_keys[i].press_tick = now;
        g_track_key_pressed[i] = (pressed != 0U);
        g_track_key_stable[i] = pressed;
    }
    s_key_events = 0U;
}

void bsp_track_key_update(uint32_t now)
{
    uint8_t i;

    for (i = 0U; i < TRACK_KEY_COUNT; i++) {
        TrackKeyState_t *key = &s_keys[i];
        uint8_t raw = track_key_raw((track_key_id_t)i) ? 1U : 0U;

        if (raw != key->raw_pressed) {
            key->raw_pressed = raw;
            key->raw_change_tick = now;
        }

        if ((raw != key->stable_pressed) &&
            ((uint32_t)(now - key->raw_change_tick) >=
             BSP_TRACK_KEY_DEBOUNCE_MS)) {
            key->stable_pressed = raw;
            g_track_key_pressed[i] = (raw != 0U);
            g_track_key_stable[i] = raw;

            if (raw != 0U) {
                key->press_tick = now;
                key->long_sent = 0U;
                if (i == (uint8_t)TRACK_KEY_NEXT) {
                    s_key_events |= TRACK_KEY_EVENT_NEXT_PRESS;
                }
            } else if ((i == (uint8_t)TRACK_KEY_OK) &&
                       (key->long_sent == 0U)) {
                s_key_events |= TRACK_KEY_EVENT_OK_SHORT;
            }
        }

        if ((i == (uint8_t)TRACK_KEY_OK) &&
            (key->stable_pressed != 0U) &&
            (key->long_sent == 0U) &&
            ((uint32_t)(now - key->press_tick) >=
             BSP_TRACK_KEY_OK_LONG_MS)) {
            key->long_sent = 1U;
            s_key_events |= TRACK_KEY_EVENT_OK_LONG;
        }
    }
}

uint8_t bsp_track_key_take_events(void)
{
    uint8_t events = s_key_events;
    s_key_events = 0U;
    return events;
}

bool bsp_track_key_is_pressed(track_key_id_t key)
{
    return ((uint32_t)key < (uint32_t)TRACK_KEY_COUNT) &&
           g_track_key_pressed[key];
}
