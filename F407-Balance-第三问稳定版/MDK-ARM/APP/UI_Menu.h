#ifndef F407_BALANCE_UI_MENU_H
#define F407_BALANCE_UI_MENU_H

#include <stdint.h>

typedef enum {
    UI_PAGE_HOME = 0,
    UI_PAGE_MAIN_MENU,
    UI_PAGE_BALL_MENU,
    UI_PAGE_BALL_TRAJECTORY,
    UI_PAGE_BALL_TARGET,
    UI_PAGE_TRACK_MENU,
    UI_PAGE_TRACK_READY,
    UI_PAGE_TRACK_RUN,
    UI_PAGE_SYSTEM_STATUS
} UI_Page_t;

typedef struct {
    uint8_t page;
    uint8_t home_detail;
    uint8_t main_index;
    uint8_t ball_index;
    uint8_t track_index;
    uint8_t track_run_detail;
    uint8_t target_press_pending;
    uint8_t trajectory_press_pending;
    uint8_t balance_exit_pending;
    uint32_t target_action_count;
    uint32_t trajectory_action_count;
    uint32_t balance_exit_count;
    uint32_t track_exit_count;
    uint32_t track_start_count;
    uint32_t track_start_reject_count;
} UI_MenuSnapshot_t;

void UI_MenuInit(void);
void UI_MenuProcess(void);
uint8_t UI_MenuTakeTargetPress(void);
uint8_t UI_MenuTakeTrajectoryPress(void);
uint8_t UI_MenuTakeBalanceExit(void);
uint8_t UI_MenuGetSnapshot(UI_MenuSnapshot_t *snapshot);

#endif /* F407_BALANCE_UI_MENU_H */
