#ifndef __TRACK_KEYS_H
#define __TRACK_KEYS_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TRACK_KEY_NEXT = 0,
    TRACK_KEY_OK,
    TRACK_KEY_COUNT
} track_key_id_t;

#define TRACK_KEY_EVENT_NEXT_PRESS   (1U << 0)
#define TRACK_KEY_EVENT_OK_SHORT     (1U << 1)
#define TRACK_KEY_EVENT_OK_LONG      (1U << 2)

extern bool g_track_key_pressed[TRACK_KEY_COUNT];
extern uint8_t g_track_key_stable[TRACK_KEY_COUNT];

void bsp_track_key_init(void);
void bsp_track_key_update(uint32_t now);
uint8_t bsp_track_key_take_events(void);
bool bsp_track_key_is_pressed(track_key_id_t key);

#endif /* __TRACK_KEYS_H */
