#include "UI_Menu.h"

#include "Balance_Control.h"
#include "LineTrack_Control.h"
#include "Track_Keys.h"

#include "stm32f4xx_hal.h"

#include <string.h>

#define UI_MENU_ITEM_COUNT              3U

static UI_MenuSnapshot_t s_ui;

static uint8_t UI_IsTrackRunning(uint8_t state)
{
    return (uint8_t)((state == (uint8_t)LINE_TRACK_STATE_TASK1_TRACKING) ||
                     (state == (uint8_t)LINE_TRACK_STATE_TASK3_BALANCE) ||
                     (state == (uint8_t)LINE_TRACK_STATE_TASK4_BALANCE));
}

static uint8_t UI_SelectNext(uint8_t index)
{
    return (index + 1U >= UI_MENU_ITEM_COUNT) ? 0U : (uint8_t)(index + 1U);
}

static void UI_RequestTargetAction(void)
{
    if (s_ui.target_press_pending == 0U) {
        s_ui.target_press_pending = 1U;
        s_ui.target_action_count++;
    }
    s_ui.page = (uint8_t)UI_PAGE_BALL_TARGET;
}

static void UI_RequestBalanceExit(void)
{
    if (s_ui.balance_exit_pending == 0U) {
        s_ui.balance_exit_pending = 1U;
        s_ui.balance_exit_count++;
    }
}

static void UI_HandleLongPress(uint8_t track_state)
{
    switch ((UI_Page_t)s_ui.page) {
    case UI_PAGE_HOME:
        s_ui.page = (uint8_t)UI_PAGE_MAIN_MENU;
        break;
    case UI_PAGE_MAIN_MENU:
        s_ui.page = (uint8_t)UI_PAGE_HOME;
        break;
    case UI_PAGE_BALL_MENU:
    case UI_PAGE_TRACK_MENU:
    case UI_PAGE_SYSTEM_STATUS:
        s_ui.page = (uint8_t)UI_PAGE_MAIN_MENU;
        break;
    case UI_PAGE_BALL_TRAJECTORY:
        UI_RequestBalanceExit();
        s_ui.page = (uint8_t)UI_PAGE_BALL_MENU;
        break;
    case UI_PAGE_BALL_TARGET:
        /* 选点锁定页退出时同时清理可能正在运行/已完成的慢速循迹。 */
        if (track_state != (uint8_t)LINE_TRACK_STATE_IDLE) {
            if (LineTrack_RequestStop() != 0U) {
                s_ui.track_exit_count++;
            }
        }
        UI_RequestBalanceExit();
        s_ui.page = (uint8_t)UI_PAGE_BALL_MENU;
        break;
    case UI_PAGE_TRACK_READY:
        s_ui.page = (uint8_t)UI_PAGE_TRACK_MENU;
        break;
    case UI_PAGE_TRACK_RUN:
        if (LineTrack_RequestStop() != 0U) {
            s_ui.track_exit_count++;
        }
        s_ui.page = (uint8_t)UI_PAGE_TRACK_MENU;
        break;
    default:
        s_ui.page = (uint8_t)UI_PAGE_HOME;
        break;
    }
}

static void UI_HandleNavigation(uint8_t event,
                                uint8_t track_running,
                                uint8_t captured_target_held)
{
    if ((event & TRACK_KEY_EVENT_NEXT_PRESS) == 0U) {
        return;
    }

    if (((UI_Page_t)s_ui.page == UI_PAGE_TRACK_RUN) &&
        (track_running != 0U)) {
        s_ui.track_run_detail ^= 1U;
        return;
    }

    switch ((UI_Page_t)s_ui.page) {
    case UI_PAGE_HOME:
        s_ui.home_detail = UI_SelectNext(s_ui.home_detail);
        break;
    case UI_PAGE_MAIN_MENU:
        s_ui.main_index = UI_SelectNext(s_ui.main_index);
        break;
    case UI_PAGE_BALL_MENU:
        s_ui.ball_index = UI_SelectNext(s_ui.ball_index);
        break;
    case UI_PAGE_TRACK_MENU:
        s_ui.track_index = UI_SelectNext(s_ui.track_index);
        break;
    case UI_PAGE_BALL_TARGET:
        /* 目标锁定后，PE3 单击直接启动与稳球绑定的慢速前馈循迹。 */
        if ((captured_target_held != 0U) && (track_running == 0U)) {
            if (LineTrack_RequestStart(LINE_TRACK_MODE_SLOW) != 0U) {
                s_ui.track_start_count++;
                s_ui.track_run_detail = 0U;
            } else {
                s_ui.track_start_reject_count++;
            }
        }
        break;
    default:
        break;
    }
}

static void UI_HandleShortPress(uint8_t track_running)
{
    switch ((UI_Page_t)s_ui.page) {
    case UI_PAGE_HOME:
        /* 保留原 PE2 快捷动作：主页短按直接进入目标回水平/采集流程。 */
        UI_RequestTargetAction();
        break;
    case UI_PAGE_MAIN_MENU:
        if (s_ui.main_index == 0U) {
            s_ui.page = (uint8_t)UI_PAGE_BALL_MENU;
        } else if (s_ui.main_index == 1U) {
            s_ui.page = (uint8_t)UI_PAGE_TRACK_MENU;
        } else {
            s_ui.page = (uint8_t)UI_PAGE_SYSTEM_STATUS;
        }
        break;
    case UI_PAGE_BALL_MENU:
        if (s_ui.ball_index == 0U) {
            s_ui.home_detail = 0U;
            s_ui.page = (uint8_t)UI_PAGE_HOME;
        } else if (s_ui.ball_index == 1U) {
            s_ui.page = (uint8_t)UI_PAGE_BALL_TRAJECTORY;
        } else {
            s_ui.page = (uint8_t)UI_PAGE_BALL_TARGET;
        }
        break;
    case UI_PAGE_BALL_TARGET:
        UI_RequestTargetAction();
        break;
    case UI_PAGE_BALL_TRAJECTORY:
        if (s_ui.trajectory_press_pending == 0U) {
            s_ui.trajectory_press_pending = 1U;
            s_ui.trajectory_action_count++;
        }
        break;
    case UI_PAGE_TRACK_MENU:
        s_ui.page = (uint8_t)UI_PAGE_TRACK_READY;
        break;
    case UI_PAGE_TRACK_READY:
        if (track_running == 0U) {
            LineTrackMode_t mode = (LineTrackMode_t)s_ui.track_index;
            if (LineTrack_RequestStart(mode) != 0U) {
                s_ui.page = (uint8_t)UI_PAGE_TRACK_RUN;
                s_ui.track_run_detail = 0U;
                s_ui.track_start_count++;
            } else {
                s_ui.track_start_reject_count++;
            }
        }
        break;
    default:
        break;
    }
}

void UI_MenuInit(void)
{
    memset(&s_ui, 0, sizeof(s_ui));
    s_ui.page = (uint8_t)UI_PAGE_HOME;
    bsp_track_key_init();
}

void UI_MenuProcess(void)
{
    BalanceTelemetrySnapshot_t balance;
    LineTrackDebug_t track;
    uint8_t events;
    uint8_t track_running = 0U;
    uint8_t track_state = (uint8_t)LINE_TRACK_STATE_IDLE;
    uint8_t captured_target_held = 0U;

    bsp_track_key_update(HAL_GetTick());
    events = bsp_track_key_take_events();
    if (LineTrack_GetTelemetrySnapshot(&track) != 0U) {
        track_state = track.state;
        track_running = UI_IsTrackRunning(track.state);
    }
    if (Balance_GetTelemetrySnapshot(&balance) != 0U) {
        captured_target_held = (uint8_t)
            ((balance.state == (uint8_t)BALANCE_STATE_HOLD_CAPTURED_TARGET) &&
             (balance.fault == (uint8_t)BALANCE_FAULT_NONE));
    }

    if ((events & TRACK_KEY_EVENT_OK_LONG) != 0U) {
        UI_HandleLongPress(track_state);
    }
    UI_HandleNavigation(events, track_running, captured_target_held);
    if ((events & TRACK_KEY_EVENT_OK_SHORT) != 0U) {
        UI_HandleShortPress(track_running);
    }
}

uint8_t UI_MenuTakeTargetPress(void)
{
    uint8_t pending = s_ui.target_press_pending;
    s_ui.target_press_pending = 0U;
    return pending;
}

uint8_t UI_MenuTakeTrajectoryPress(void)
{
    uint8_t pending = s_ui.trajectory_press_pending;
    s_ui.trajectory_press_pending = 0U;
    return pending;
}

uint8_t UI_MenuTakeBalanceExit(void)
{
    uint8_t pending = s_ui.balance_exit_pending;
    s_ui.balance_exit_pending = 0U;
    return pending;
}

uint8_t UI_MenuGetSnapshot(UI_MenuSnapshot_t *snapshot)
{
    if (snapshot == NULL) {
        return 0U;
    }
    *snapshot = s_ui;
    return 1U;
}
