/**
 ******************************************************************************
 * @file    OLED_Display.c
 * @brief   0.96 英寸 128x64 OLED 两键菜单与只读运行信息界面。
 *
 * 显示层只读取 Balance、LineTrack、UI 三个快照以及 OLED 自身发送状态；不修改
 * PID、运动状态机、电机目标或按键事件。显存仅在 I2C DMA 发送层空闲时重绘，
 * 主循环不等待整帧完成；I2C 故障恢复也只由 OLED_Process() 在主循环推进。
 ******************************************************************************
 */

#include "OLED_Display.h"

#include "Balance_Control.h"
#include "LineTrack_Control.h"
#include "OLED.h"
#include "UI_Menu.h"
#include "main.h"

#include <string.h>

#define OLED_FONT_WIDTH                   5U
#define OLED_FONT_SCALE_LARGE             2U
#define OLED_FONT_ADVANCE_SMALL           6U
#define OLED_FONT_ADVANCE_LARGE          11U
#define OLED_PAGE_COUNT                    8U

typedef struct
{
    char character;
    uint8_t column[OLED_FONT_WIDTH];
} OLED_Glyph_t;

volatile OLED_DisplayDebug_t g_oled_display_debug = {0};

/* 5x7 ASCII 子集：界面全部使用大写英文，兼顾 0.96 英寸屏幕可读性和 Flash 占用。 */
static const OLED_Glyph_t s_glyphs[] = {
    {' ', {0x00U, 0x00U, 0x00U, 0x00U, 0x00U}},
    {'+', {0x08U, 0x08U, 0x3EU, 0x08U, 0x08U}},
    {'-', {0x08U, 0x08U, 0x08U, 0x08U, 0x08U}},
    {'.', {0x00U, 0x60U, 0x60U, 0x00U, 0x00U}},
    {'/', {0x20U, 0x10U, 0x08U, 0x04U, 0x02U}},
    {':', {0x00U, 0x36U, 0x36U, 0x00U, 0x00U}},
    {'<', {0x08U, 0x14U, 0x22U, 0x41U, 0x00U}},
    {'=', {0x14U, 0x14U, 0x14U, 0x14U, 0x14U}},
    {'>', {0x00U, 0x41U, 0x22U, 0x14U, 0x08U}},
    {'?', {0x02U, 0x01U, 0x51U, 0x09U, 0x06U}},
    {'0', {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU}},
    {'1', {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U}},
    {'2', {0x42U, 0x61U, 0x51U, 0x49U, 0x46U}},
    {'3', {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U}},
    {'4', {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U}},
    {'5', {0x27U, 0x45U, 0x45U, 0x45U, 0x39U}},
    {'6', {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U}},
    {'7', {0x01U, 0x71U, 0x09U, 0x05U, 0x03U}},
    {'8', {0x36U, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'9', {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}},
    {'A', {0x7EU, 0x11U, 0x11U, 0x11U, 0x7EU}},
    {'B', {0x7FU, 0x49U, 0x49U, 0x49U, 0x36U}},
    {'C', {0x3EU, 0x41U, 0x41U, 0x41U, 0x22U}},
    {'D', {0x7FU, 0x41U, 0x41U, 0x22U, 0x1CU}},
    {'E', {0x7FU, 0x49U, 0x49U, 0x49U, 0x41U}},
    {'F', {0x7FU, 0x09U, 0x09U, 0x09U, 0x01U}},
    {'G', {0x3EU, 0x41U, 0x49U, 0x49U, 0x7AU}},
    {'H', {0x7FU, 0x08U, 0x08U, 0x08U, 0x7FU}},
    {'I', {0x00U, 0x41U, 0x7FU, 0x41U, 0x00U}},
    {'J', {0x20U, 0x40U, 0x41U, 0x3FU, 0x01U}},
    {'K', {0x7FU, 0x08U, 0x14U, 0x22U, 0x41U}},
    {'L', {0x7FU, 0x40U, 0x40U, 0x40U, 0x40U}},
    {'M', {0x7FU, 0x02U, 0x0CU, 0x02U, 0x7FU}},
    {'N', {0x7FU, 0x02U, 0x04U, 0x08U, 0x7FU}},
    {'O', {0x3EU, 0x41U, 0x41U, 0x41U, 0x3EU}},
    {'P', {0x7FU, 0x09U, 0x09U, 0x09U, 0x06U}},
    {'Q', {0x3EU, 0x41U, 0x51U, 0x21U, 0x5EU}},
    {'R', {0x7FU, 0x09U, 0x19U, 0x29U, 0x46U}},
    {'S', {0x46U, 0x49U, 0x49U, 0x49U, 0x31U}},
    {'T', {0x01U, 0x01U, 0x7FU, 0x01U, 0x01U}},
    {'U', {0x3FU, 0x40U, 0x40U, 0x40U, 0x3FU}},
    {'V', {0x1FU, 0x20U, 0x40U, 0x20U, 0x1FU}},
    {'W', {0x3FU, 0x40U, 0x38U, 0x40U, 0x3FU}},
    {'X', {0x63U, 0x14U, 0x08U, 0x14U, 0x63U}},
    {'Y', {0x07U, 0x08U, 0x70U, 0x08U, 0x07U}},
    {'Z', {0x61U, 0x51U, 0x49U, 0x45U, 0x43U}}
};

/* packet[0] 保留为兼容控制字节，其后 1024 字节为 SSD1306 页式显存。 */
static uint8_t s_frame_packet[OLED_FRAME_PACKET_SIZE];
static uint32_t s_last_refresh_tick;

static const uint8_t *OLED_FindGlyph(char character)
{
    uint32_t index;

    for (index = 0U; index < (sizeof(s_glyphs) / sizeof(s_glyphs[0])); index++) {
        if (s_glyphs[index].character == character) {
            return s_glyphs[index].column;
        }
    }
    return s_glyphs[0].column;
}

static uint16_t OLED_ExpandColumn2x(uint8_t column)
{
    uint16_t expanded = 0U;
    uint8_t row;

    for (row = 0U; row < 7U; row++) {
        if ((column & (uint8_t)(1U << row)) != 0U) {
            expanded |= (uint16_t)(0x0003U << (row * 2U));
        }
    }
    return expanded;
}

static void OLED_DrawCharSmall(uint8_t x,
                               uint8_t page,
                               char character,
                               uint8_t color)
{
    const uint8_t *glyph = OLED_FindGlyph(character);
    uint16_t base = (uint16_t)(1U + (uint16_t)page * OLED_WIDTH);
    uint8_t column;
    uint8_t draw_x;

    if ((page >= OLED_PAGE_COUNT) || (x >= OLED_WIDTH)) {
        return;
    }

    for (column = 0U; column < OLED_FONT_WIDTH; column++) {
        draw_x = (uint8_t)(x + column);
        if (draw_x >= OLED_WIDTH) {
            break;
        }
        if (color != 0U) {
            s_frame_packet[base + draw_x] |= glyph[column];
        } else {
            s_frame_packet[base + draw_x] &= (uint8_t)(~glyph[column]);
        }
    }
}

static void OLED_DrawCharScale2(uint8_t x,
                                uint8_t page_top,
                                char character,
                                uint8_t color)
{
    const uint8_t *glyph = OLED_FindGlyph(character);
    uint16_t top_base = (uint16_t)(1U + (uint16_t)page_top * OLED_WIDTH);
    uint16_t bottom_base = (uint16_t)(top_base + OLED_WIDTH);
    uint16_t expanded;
    uint8_t low;
    uint8_t high;
    uint8_t column;
    uint8_t draw_x;

    if ((page_top >= (OLED_PAGE_COUNT - 1U)) || (x >= OLED_WIDTH)) {
        return;
    }

    for (column = 0U; column < OLED_FONT_WIDTH; column++) {
        draw_x = (uint8_t)(x + column * OLED_FONT_SCALE_LARGE);
        if ((uint16_t)draw_x + 1U >= OLED_WIDTH) {
            break;
        }

        expanded = OLED_ExpandColumn2x(glyph[column]);
        low = (uint8_t)(expanded & 0x00FFU);
        high = (uint8_t)((expanded >> 8U) & 0x003FU);
        if (color != 0U) {
            s_frame_packet[top_base + draw_x] |= low;
            s_frame_packet[top_base + draw_x + 1U] |= low;
            s_frame_packet[bottom_base + draw_x] |= high;
            s_frame_packet[bottom_base + draw_x + 1U] |= high;
        } else {
            s_frame_packet[top_base + draw_x] &= (uint8_t)(~low);
            s_frame_packet[top_base + draw_x + 1U] &= (uint8_t)(~low);
            s_frame_packet[bottom_base + draw_x] &= (uint8_t)(~high);
            s_frame_packet[bottom_base + draw_x + 1U] &= (uint8_t)(~high);
        }
    }
}

static uint8_t OLED_TextWidth(const char *text, uint8_t advance)
{
    uint16_t length = 0U;
    uint16_t width;

    while ((text != NULL) && (text[length] != '\0')) {
        length++;
    }
    if (length == 0U) {
        return 0U;
    }
    width = length * advance - 1U;
    return (width > 255U) ? 255U : (uint8_t)width;
}

static void OLED_DrawTextSmall(uint8_t x,
                               uint8_t page,
                               const char *text,
                               uint8_t color)
{
    while ((text != NULL) && (*text != '\0') && (x < OLED_WIDTH)) {
        OLED_DrawCharSmall(x, page, *text, color);
        x = (uint8_t)(x + OLED_FONT_ADVANCE_SMALL);
        text++;
    }
}

static void OLED_DrawTextCenteredSmall(uint8_t page,
                                       const char *text,
                                       uint8_t color)
{
    uint8_t width = OLED_TextWidth(text, OLED_FONT_ADVANCE_SMALL);
    uint8_t x = (width < OLED_WIDTH) ? (uint8_t)((OLED_WIDTH - width) / 2U) : 0U;
    OLED_DrawTextSmall(x, page, text, color);
}

static void OLED_DrawTextCenteredLarge(uint8_t page_top,
                                       const char *text,
                                       uint8_t color)
{
    uint8_t width = OLED_TextWidth(text, OLED_FONT_ADVANCE_LARGE);
    uint8_t x = (width < OLED_WIDTH) ? (uint8_t)((OLED_WIDTH - width) / 2U) : 0U;

    while ((text != NULL) && (*text != '\0') && (x < OLED_WIDTH)) {
        OLED_DrawCharScale2(x, page_top, *text, color);
        x = (uint8_t)(x + OLED_FONT_ADVANCE_LARGE);
        text++;
    }
}

static void OLED_FillPage(uint8_t page, uint8_t value)
{
    if (page < OLED_PAGE_COUNT) {
        memset(&s_frame_packet[1U + (uint16_t)page * OLED_WIDTH],
               value,
               OLED_WIDTH);
    }
}

static void OLED_DrawRow(uint8_t page, const char *text, uint8_t inverse)
{
    if (inverse != 0U) {
        OLED_FillPage(page, 0xFFU);
        OLED_DrawTextCenteredSmall(page, text, 0U);
    } else {
        OLED_DrawTextCenteredSmall(page, text, 1U);
    }
}

static void OLED_DrawTitle(const char *text)
{
    OLED_DrawRow(0U, text, 1U);
}

static void OLED_WriteUnsignedDigits(char *destination,
                                     uint32_t value,
                                     uint8_t digits,
                                     uint32_t limit)
{
    uint8_t index;

    if (value > limit) {
        value = limit;
    }
    for (index = digits; index > 0U; index--) {
        destination[index - 1U] = (char)('0' + (value % 10U));
        value /= 10U;
    }
}

static void OLED_WriteSignedDigits(char *destination,
                                   int32_t value,
                                   uint8_t digits,
                                   int32_t limit)
{
    uint32_t magnitude;

    if (value > limit) {
        value = limit;
    } else if (value < -limit) {
        value = -limit;
    }

    destination[0] = (value < 0) ? '-' : '+';
    magnitude = (value < 0) ? (uint32_t)(-value) : (uint32_t)value;
    OLED_WriteUnsignedDigits(&destination[1], magnitude, digits, (uint32_t)limit);
}

static void OLED_WriteTimeTenths(char *destination, uint32_t elapsed_ms)
{
    uint32_t tenths = elapsed_ms / 100U;

    if (tenths > 9999U) {
        tenths = 9999U;
    }
    destination[0] = (char)('0' + ((tenths / 1000U) % 10U));
    destination[1] = (char)('0' + ((tenths / 100U) % 10U));
    destination[2] = (char)('0' + ((tenths / 10U) % 10U));
    destination[3] = '.';
    destination[4] = (char)('0' + (tenths % 10U));
}

static int32_t OLED_RoundFloat(float value, int32_t limit)
{
    if (value != value) {
        return 0;
    }
    if (value > (float)limit) {
        return limit;
    }
    if (value < (float)(-limit)) {
        return -limit;
    }
    return (value >= 0.0f) ? (int32_t)(value + 0.5f) : (int32_t)(value - 0.5f);
}

static const char *OLED_GetBalanceModeText(uint8_t state)
{
    switch ((BalanceState_t)state) {
    case BALANCE_STATE_BOOT_SAFE:
        return "BOOT";
    case BALANCE_STATE_WAIT_BUTTON_RELEASE:
        return "WAIT";
    case BALANCE_STATE_IDLE:
        return "IDLE";
    case BALANCE_STATE_START_QUERY_MOTOR:
    case BALANCE_STATE_START_SET_ZERO:
    case BALANCE_STATE_START_VERIFY_ZERO:
    case BALANCE_STATE_START_ENABLE:
        return "START";
    case BALANCE_STATE_MOVE_POSITIVE:
        return "MOVE +50";
    case BALANCE_STATE_MOVE_NEGATIVE:
        return "MOVE -50";
    case BALANCE_STATE_HOLD_NEGATIVE:
        return "HOLD -50";
    case BALANCE_STATE_VISION_HOLD:
        return "VISION";
    case BALANCE_STATE_RETURN_ZERO:
        return "RETURN";
    case BALANCE_STATE_ZERO_HOLD_ABORTED:
        return "ABORT";
    case BALANCE_STATE_FAULT_LATCHED:
        return "FAULT";
    case BALANCE_STATE_DISABLING:
        return "DISABLE";
    case BALANCE_STATE_DISABLED:
        return "OFF";
    case BALANCE_STATE_HOLD_ORIGIN:
        return "HOLD 0";
    case BALANCE_STATE_TARGET_LEVELING:
        return "LEVEL";
    case BALANCE_STATE_TARGET_WAIT_CAPTURE:
        return "SET POS";
    case BALANCE_STATE_HOLD_CAPTURED_TARGET:
        return "HOLD T";
    default:
        return "UNKNOWN";
    }
}

static const char *OLED_GetTrackStateText(uint8_t state)
{
    switch ((LineTrackState_t)state) {
    case LINE_TRACK_STATE_IDLE:
        return "IDLE";
    case LINE_TRACK_STATE_TASK1_TRACKING:
        return "FAST RUN";
    case LINE_TRACK_STATE_TASK1_FINISHED:
        return "FAST DONE";
    case LINE_TRACK_STATE_TASK3_BALANCE:
        return "STRAIGHT RUN";
    case LINE_TRACK_STATE_TASK3_FINISHED:
        return "STRAIGHT DONE";
    case LINE_TRACK_STATE_TASK4_BALANCE:
        return "SLOW RUN";
    case LINE_TRACK_STATE_TASK4_FINISHED:
        return "SLOW DONE";
    default:
        return "UNKNOWN";
    }
}

static const char *OLED_GetTrackModeText(uint8_t index)
{
    if (index == (uint8_t)LINE_TRACK_MODE_FAST) {
        return "FAST LAP 30";
    }
    if (index == (uint8_t)LINE_TRACK_MODE_STRAIGHT) {
        return "STRAIGHT 20";
    }
    return "SLOW LAP 20";
}

static uint8_t OLED_IsTrackRunning(uint8_t state)
{
    return (uint8_t)((state == (uint8_t)LINE_TRACK_STATE_TASK1_TRACKING) ||
                     (state == (uint8_t)LINE_TRACK_STATE_TASK3_BALANCE) ||
                     (state == (uint8_t)LINE_TRACK_STATE_TASK4_BALANCE));
}

static OLED_DisplayMode_t OLED_GetDisplayMode(const BalanceTelemetrySnapshot_t *balance)
{
    if (balance->camera_online == 0U) {
        return OLED_DISPLAY_CAMERA_OFF;
    }
    if (balance->target_found == 0U) {
        return OLED_DISPLAY_LOST;
    }
    return OLED_DISPLAY_LOCK;
}

static void OLED_DrawCameraBanner(OLED_DisplayMode_t mode, uint32_t now)
{
    const char *text;
    uint8_t inverse;

    if (mode == OLED_DISPLAY_LOCK) {
        text = "LOCK";
        inverse = 1U;
    } else if (mode == OLED_DISPLAY_LOST) {
        text = "LOST";
        inverse = (uint8_t)(((now / OLED_DISPLAY_BLINK_HALF_MS) & 1U) == 0U);
    } else {
        text = "CAM OFF";
        inverse = (uint8_t)(((now / OLED_DISPLAY_BLINK_HALF_MS) & 1U) == 0U);
    }
    OLED_DrawRow(0U, text, inverse);
}

static void OLED_FormatTrackBits(char *text, uint8_t bits)
{
    uint8_t channel;

    for (channel = 0U; channel < 8U; channel++) {
        text[channel] = ((bits & (uint8_t)(1U << channel)) != 0U) ? '1' : '0';
    }
}

static void OLED_DrawEncoderLine(uint8_t page, const LineTrackDebug_t *track)
{
    char text[] = "ENC L:OK R:OK";

    if (track->left_encoder_fault != 0U) {
        text[6] = 'E';
        text[7] = 'R';
    }
    if (track->right_encoder_fault != 0U) {
        text[11] = 'E';
        text[12] = 'R';
    }
    OLED_DrawTextCenteredSmall(page, text, 1U);
}

static void OLED_DrawMotionLines(const BalanceTelemetrySnapshot_t *balance,
                                 uint8_t target_page,
                                 uint8_t motion_page)
{
    char target_text[] = "TG:+000 ER:+000";
    char motion_text[] = "X:+000 V:+0000";

    OLED_WriteSignedDigits(&target_text[3], balance->target_mm, 3U, 999);
    if ((balance->camera_online != 0U) && (balance->target_found != 0U)) {
        OLED_WriteSignedDigits(&target_text[11],
                               OLED_RoundFloat(balance->error_mm, 999),
                               3U,
                               999);
        OLED_WriteSignedDigits(&motion_text[2], balance->position_mm, 3U, 999);
        OLED_WriteSignedDigits(&motion_text[9],
                               OLED_RoundFloat(balance->velocity_mm_s, 9999),
                               4U,
                               9999);
    } else {
        memcpy(&target_text[11], "----", 4U);
        memcpy(&motion_text[2], "----", 4U);
        memcpy(&motion_text[9], "-----", 5U);
    }
    OLED_DrawTextCenteredSmall(target_page, target_text, 1U);
    OLED_DrawTextCenteredSmall(motion_page, motion_text, 1U);
}

static void OLED_DrawTrackSpeedLine(uint8_t page,
                                    const char *prefix,
                                    float left,
                                    float right)
{
    char text[] = "SPD L:+000 R:+000";

    if (prefix != NULL) {
        text[0] = prefix[0];
        text[1] = prefix[1];
        text[2] = prefix[2];
    }
    OLED_WriteSignedDigits(&text[6], OLED_RoundFloat(left, 999), 3U, 999);
    OLED_WriteSignedDigits(&text[13], OLED_RoundFloat(right, 999), 3U, 999);
    OLED_DrawTextCenteredSmall(page, text, 1U);
}

static void OLED_DrawTrackPwmLine(uint8_t page, int32_t left, int32_t right)
{
    char text[] = "PWM L:+000 R:+000";

    OLED_WriteSignedDigits(&text[6], left, 3U, 999);
    OLED_WriteSignedDigits(&text[13], right, 3U, 999);
    OLED_DrawTextCenteredSmall(page, text, 1U);
}

static void OLED_RenderHomeBall(const BalanceTelemetrySnapshot_t *balance,
                                OLED_DisplayMode_t mode,
                                uint32_t now)
{
    const char *state_text = OLED_GetBalanceModeText(balance->state);
    uint32_t elapsed_ms = 0U;
    char time_text[] = "T:000.0S";

    if (balance->trajectory_started != 0U) {
        elapsed_ms = (balance->trajectory_completed != 0U) ?
                     balance->completion_time_ms : balance->run_elapsed_ms;
    }

    OLED_DrawCameraBanner(mode, now);
    OLED_DrawTextCenteredSmall(1U, "PE3 NEXT PE2 TGT/MENU", 1U);
    OLED_DrawTextCenteredLarge(2U, state_text, 1U);
    OLED_WriteTimeTenths(&time_text[2], elapsed_ms);
    OLED_DrawTextCenteredLarge(4U, time_text, 1U);
    OLED_DrawMotionLines(balance, 6U, 7U);
}

static void OLED_RenderHomeTrack(const LineTrackDebug_t *track)
{
    char time_text[] = "T:000.0";
    char bits_error[] = "C:00000000 E:+0000";
    uint32_t display_time = (track->final_time_ms != 0U) ?
                            track->final_time_ms : track->elapsed_time_ms;

    OLED_DrawTitle("LINE TRACK");
    OLED_DrawTextCenteredSmall(1U, OLED_GetTrackStateText(track->state), 1U);
    OLED_WriteTimeTenths(&time_text[2], display_time);
    OLED_DrawTextCenteredLarge(2U, time_text, 1U);
    OLED_FormatTrackBits(&bits_error[2], track->track_bits);
    OLED_WriteSignedDigits(&bits_error[13], track->line_error, 4U, 9999);
    OLED_DrawTextCenteredSmall(4U, bits_error, 1U);
    OLED_DrawTrackSpeedLine(5U, "SPD", track->left_speed, track->right_speed);
    OLED_DrawTrackPwmLine(6U, track->left_pwm, track->right_pwm);
    OLED_DrawTextCenteredSmall(7U, "PE3 NEXT PE2 TGT/MENU", 1U);
}

static void OLED_RenderSystem(const BalanceTelemetrySnapshot_t *balance,
                              const LineTrackDebug_t *track,
                              uint8_t menu_page)
{
    char camera_text[] = "CAM:OK AGE:0000";
    char balance_fault[] = "F:00";
    char motor_text[] = "MOTOR O:0 S:00 A:0";
    char oled_text[] = "OLED S:00 F:0000";
    char track_text[] = "TRACK S:00";

    if (balance->camera_online == 0U) {
        camera_text[4] = 'O';
        camera_text[5] = 'F';
    }
    OLED_WriteUnsignedDigits(&camera_text[11], balance->last_valid_age_ms, 4U, 9999U);
    OLED_WriteUnsignedDigits(&balance_fault[2], balance->fault, 2U, 99U);
    motor_text[8] = (char)('0' + ((balance->motor_owner <= 9U) ? balance->motor_owner : 9U));
    OLED_WriteUnsignedDigits(&motor_text[12], balance->motor_state, 2U, 99U);
    motor_text[17] = (char)('0' + ((balance->motor_ack_status <= 9U) ?
                                  balance->motor_ack_status : 9U));
    OLED_WriteUnsignedDigits(&oled_text[7], g_oled_debug.state, 2U, 99U);
    OLED_WriteUnsignedDigits(&oled_text[12], g_oled_debug.frame_count, 4U, 9999U);
    OLED_WriteUnsignedDigits(&track_text[8], track->state, 2U, 99U);

    OLED_DrawTitle("SYSTEM STATUS");
    OLED_DrawTextCenteredSmall(1U, camera_text, 1U);
    OLED_DrawTextSmall(0U, 2U, "BAL", 1U);
    OLED_DrawTextSmall(24U, 2U, OLED_GetBalanceModeText(balance->state), 1U);
    OLED_DrawTextSmall(102U, 2U, balance_fault, 1U);
    OLED_DrawTextCenteredSmall(3U, motor_text, 1U);
    OLED_DrawTextCenteredSmall(4U, oled_text, 1U);
    OLED_DrawTextCenteredSmall(5U, track_text, 1U);
    OLED_DrawEncoderLine(6U, track);
    OLED_DrawTextCenteredSmall(7U,
                               (menu_page != 0U) ?
                               "HOLD PE2 BACK" : "PE3 NEXT PE2 TGT/MENU",
                               1U);
}

static void OLED_RenderMainMenu(uint8_t selected)
{
    OLED_DrawTitle("MAIN MENU");
    OLED_DrawTextCenteredSmall(1U, "SELECT FUNCTION", 1U);
    OLED_DrawRow(2U, "BALL CONTROL", (uint8_t)(selected == 0U));
    OLED_DrawRow(3U, "LINE TRACK", (uint8_t)(selected == 1U));
    OLED_DrawRow(4U, "SYSTEM STATUS", (uint8_t)(selected == 2U));
    OLED_DrawTextCenteredSmall(6U, "PE3 NEXT", 1U);
    OLED_DrawTextCenteredSmall(7U, "PE2 OK HOLD BACK", 1U);
}

static void OLED_RenderBallMenu(uint8_t selected)
{
    OLED_DrawTitle("BALL CONTROL");
    OLED_DrawTextCenteredSmall(1U, "SELECT PAGE", 1U);
    OLED_DrawRow(2U, "LIVE BALL", (uint8_t)(selected == 0U));
    OLED_DrawRow(3U, "SCORE TRAJECTORY", (uint8_t)(selected == 1U));
    OLED_DrawRow(4U, "CAPTURE TARGET", (uint8_t)(selected == 2U));
    OLED_DrawTextCenteredSmall(6U, "PE3 NEXT", 1U);
    OLED_DrawTextCenteredSmall(7U, "PE2 OK HOLD BACK", 1U);
}

static void OLED_RenderTrajectory(const BalanceTelemetrySnapshot_t *balance)
{
    char time_position[] = "T:000.0 X:+000";
    char target_speed[] = "TG:+000 V:+0000";
    const char *status;
    const char *result;

    if (balance->trajectory_completed != 0U) {
        status = (balance->score_pass != 0U) ? "DONE PASS" : "DONE CHECK";
        result = (balance->score_pass != 0U) ? "RESULT: PASS" : "RESULT: CHECK";
    } else if (balance->trajectory_started != 0U) {
        status = "RUNNING";
        result = "RESULT: RUNNING";
    } else if (balance->trajectory_ready != 0U) {
        status = "READY";
        result = "RESULT: START READY";
    } else {
        status = "NOT READY";
        result = "RESULT: WAIT ORIGIN";
    }

    OLED_DrawTitle("SCORE TRAJECTORY");
    OLED_DrawRow(1U, status, (uint8_t)(balance->trajectory_ready != 0U));
    OLED_DrawTextCenteredSmall(2U, "0 > +50 > -50", 1U);
    OLED_DrawTextCenteredSmall(3U, OLED_GetBalanceModeText(balance->state), 1U);
    OLED_WriteTimeTenths(&time_position[2],
                         (balance->trajectory_completed != 0U) ?
                         balance->completion_time_ms : balance->run_elapsed_ms);
    OLED_WriteSignedDigits(&target_speed[3], balance->target_mm, 3U, 999);
    if ((balance->camera_online != 0U) && (balance->target_found != 0U)) {
        OLED_WriteSignedDigits(&time_position[10], balance->position_mm, 3U, 999);
        OLED_WriteSignedDigits(&target_speed[10],
                               OLED_RoundFloat(balance->velocity_mm_s, 9999),
                               4U,
                               9999);
    } else {
        memcpy(&time_position[10], "----", 4U);
        memcpy(&target_speed[10], "-----", 5U);
    }
    OLED_DrawTextCenteredSmall(4U, time_position, 1U);
    OLED_DrawTextCenteredSmall(5U, target_speed, 1U);
    OLED_DrawTextCenteredSmall(6U, result, 1U);
    OLED_DrawTextCenteredSmall(7U, "PE2/PA0 START H=EXIT", 1U);
}

static void OLED_RenderTarget(const BalanceTelemetrySnapshot_t *balance,
                              const LineTrackDebug_t *track)
{
    OLED_DrawTitle("CAPTURE TARGET");
    OLED_DrawTextCenteredSmall(1U, "PE3 SLOW H=PE2 EXIT", 1U);

    if (balance->state == (uint8_t)BALANCE_STATE_TARGET_LEVELING) {
        OLED_DrawTextCenteredLarge(2U, "LEVEL", 1U);
        OLED_DrawTextCenteredSmall(4U, "AXIS TO ZERO", 1U);
        OLED_DrawTextCenteredSmall(5U, "WAIT MOTOR ACK", 1U);
    } else if (balance->state == (uint8_t)BALANCE_STATE_TARGET_WAIT_CAPTURE) {
        OLED_DrawTextCenteredLarge(2U, "SET POS", 1U);
        OLED_DrawTextCenteredSmall(4U, "PLACE BALL", 1U);
        OLED_DrawRow(5U, "PE2 CAPTURE", 1U);
    } else if (balance->state == (uint8_t)BALANCE_STATE_HOLD_CAPTURED_TARGET) {
        if ((track != NULL) &&
            (track->state == (uint8_t)LINE_TRACK_STATE_TASK4_BALANCE)) {
            OLED_DrawTextCenteredLarge(2U, "SLOW", 1U);
            OLED_DrawTextCenteredSmall(4U, "TRACK RUNNING", 1U);
            OLED_DrawTextCenteredSmall(5U, "HOLD PE2 EXIT", 1U);
        } else if ((track != NULL) &&
                   (track->state == (uint8_t)LINE_TRACK_STATE_TASK4_FINISHED)) {
            OLED_DrawTextCenteredLarge(2U, "DONE", 1U);
            OLED_DrawTextCenteredSmall(4U, "SLOW FINISHED", 1U);
            OLED_DrawTextCenteredSmall(5U, "HOLD PE2 MENU", 1U);
        } else {
            OLED_DrawTextCenteredLarge(2U, "HOLD T", 1U);
            OLED_DrawTextCenteredSmall(4U, "TARGET LOCKED", 1U);
            OLED_DrawTextCenteredSmall(5U, "PE3 START SLOW", 1U);
        }
    } else {
        OLED_DrawTextCenteredLarge(2U, "PE2", 1U);
        OLED_DrawTextCenteredSmall(4U, "BEGIN CAPTURE", 1U);
        OLED_DrawTextCenteredSmall(5U, OLED_GetBalanceModeText(balance->state), 1U);
    }

    OLED_DrawMotionLines(balance, 6U, 7U);
}

static void OLED_RenderTrackMenu(uint8_t selected)
{
    OLED_DrawTitle("LINE TRACK");
    OLED_DrawTextCenteredSmall(1U, "SELECT MODE", 1U);
    OLED_DrawRow(2U, "FAST LAP 30", (uint8_t)(selected == 0U));
    OLED_DrawRow(3U, "STRAIGHT 20", (uint8_t)(selected == 1U));
    OLED_DrawRow(4U, "SLOW LAP 20", (uint8_t)(selected == 2U));
    OLED_DrawTextCenteredSmall(6U, "PE3 NEXT", 1U);
    OLED_DrawTextCenteredSmall(7U, "PE2 OK HOLD BACK", 1U);
}

static void OLED_RenderTrackReady(const UI_MenuSnapshot_t *ui,
                                  const LineTrackDebug_t *track)
{
    char bits_text[] = "00000000";
    char error_text[] = "ERR:+0000";

    OLED_DrawTitle("TRACK READY");
    OLED_DrawTextCenteredSmall(1U, OLED_GetTrackModeText(ui->track_index), 1U);
    OLED_FormatTrackBits(bits_text, track->track_bits);
    OLED_DrawTextCenteredLarge(2U, bits_text, 1U);
    OLED_WriteSignedDigits(&error_text[4], track->line_error, 4U, 9999);
    OLED_DrawTextCenteredSmall(4U, error_text, 1U);
    OLED_DrawTrackSpeedLine(5U, "SPD", track->left_speed, track->right_speed);
    OLED_DrawEncoderLine(6U, track);
    OLED_DrawTextCenteredSmall(7U, "PE2 START HOLD BACK", 1U);
}

static void OLED_RenderTrackRunPrimary(const LineTrackDebug_t *track)
{
    char time_text[] = "T:000.0";
    char bits_error[] = "C:00000000 E:+0000";
    uint32_t display_time = (track->final_time_ms != 0U) ?
                            track->final_time_ms : track->elapsed_time_ms;

    OLED_DrawTitle("TRACK RUN 1/2");
    OLED_DrawTextCenteredSmall(1U, OLED_GetTrackStateText(track->state), 1U);
    OLED_WriteTimeTenths(&time_text[2], display_time);
    OLED_DrawTextCenteredLarge(2U, time_text, 1U);
    OLED_FormatTrackBits(&bits_error[2], track->track_bits);
    OLED_WriteSignedDigits(&bits_error[13], track->line_error, 4U, 9999);
    OLED_DrawTextCenteredSmall(4U, bits_error, 1U);
    OLED_DrawTrackSpeedLine(5U, "SPD", track->left_speed, track->right_speed);
    OLED_DrawTrackPwmLine(6U, track->left_pwm, track->right_pwm);
    OLED_DrawTextCenteredSmall(7U,
                               (OLED_IsTrackRunning(track->state) != 0U) ?
                               "PE3 PAGE HOLD EXIT" : "HOLD PE2 EXIT",
                               1U);
}

static void OLED_RenderTrackRunSecondary(const LineTrackDebug_t *track)
{
    char finish_text[] = "FIN ARM:0 PEND:0";
    char recovery_text[] = "RECOVERY:0000";
    char final_text[] = "FINAL:000.0S";

    finish_text[8] = (track->finish_armed != 0U) ? '1' : '0';
    finish_text[15] = (track->finish_pending != 0U) ? '1' : '0';
    OLED_WriteUnsignedDigits(&recovery_text[9], track->recovery_enter_count, 4U, 9999U);
    OLED_WriteTimeTenths(&final_text[6],
                         (track->final_time_ms != 0U) ?
                         track->final_time_ms : track->elapsed_time_ms);

    OLED_DrawTitle("TRACK RUN 2/2");
    OLED_DrawTextCenteredSmall(1U, OLED_GetTrackStateText(track->state), 1U);
    OLED_DrawTrackSpeedLine(2U, "SET", track->left_speed_set, track->right_speed_set);
    OLED_DrawEncoderLine(3U, track);
    OLED_DrawTextCenteredSmall(4U, finish_text, 1U);
    OLED_DrawTextCenteredSmall(5U, recovery_text, 1U);
    OLED_DrawTextCenteredSmall(6U, final_text, 1U);
    OLED_DrawTextCenteredSmall(7U,
                               (OLED_IsTrackRunning(track->state) != 0U) ?
                               "PE3 PAGE HOLD EXIT" : "HOLD PE2 EXIT",
                               1U);
}

static void OLED_Render(const BalanceTelemetrySnapshot_t *balance,
                        const LineTrackDebug_t *track,
                        const UI_MenuSnapshot_t *ui,
                        OLED_DisplayMode_t mode,
                        uint32_t now)
{
    memset(&s_frame_packet[1], 0, OLED_FRAME_BUFFER_SIZE);

    switch ((UI_Page_t)ui->page) {
    case UI_PAGE_HOME:
        if (ui->home_detail == 0U) {
            OLED_RenderHomeBall(balance, mode, now);
        } else if (ui->home_detail == 1U) {
            OLED_RenderHomeTrack(track);
        } else {
            OLED_RenderSystem(balance, track, 0U);
        }
        break;
    case UI_PAGE_MAIN_MENU:
        OLED_RenderMainMenu(ui->main_index);
        break;
    case UI_PAGE_BALL_MENU:
        OLED_RenderBallMenu(ui->ball_index);
        break;
    case UI_PAGE_BALL_TRAJECTORY:
        OLED_RenderTrajectory(balance);
        break;
    case UI_PAGE_BALL_TARGET:
        OLED_RenderTarget(balance, track);
        break;
    case UI_PAGE_TRACK_MENU:
        OLED_RenderTrackMenu(ui->track_index);
        break;
    case UI_PAGE_TRACK_READY:
        OLED_RenderTrackReady(ui, track);
        break;
    case UI_PAGE_TRACK_RUN:
        if (ui->track_run_detail == 0U) {
            OLED_RenderTrackRunPrimary(track);
        } else {
            OLED_RenderTrackRunSecondary(track);
        }
        break;
    case UI_PAGE_SYSTEM_STATUS:
        OLED_RenderSystem(balance, track, 1U);
        break;
    default:
        OLED_RenderHomeBall(balance, mode, now);
        break;
    }
}

void OLED_DisplayInit(void)
{
    memset(s_frame_packet, 0, sizeof(s_frame_packet));
    memset((void *)&g_oled_display_debug, 0, sizeof(g_oled_display_debug));
    s_frame_packet[0] = OLED_DATA_CONTROL_BYTE;
    s_last_refresh_tick = HAL_GetTick();
    OLED_Init();
}

void OLED_DisplayProcess(void)
{
    BalanceTelemetrySnapshot_t balance;
    LineTrackDebug_t track;
    UI_MenuSnapshot_t ui;
    OLED_DisplayMode_t mode;
    uint32_t now;

    now = HAL_GetTick();
    OLED_Process(now);
    if ((g_oled_debug.enabled == 0U) ||
        ((uint32_t)(now - s_last_refresh_tick) < OLED_DISPLAY_REFRESH_MS) ||
        (OLED_CanAcceptFrame() == 0U)) {
        return;
    }

    if ((Balance_GetTelemetrySnapshot(&balance) == 0U) ||
        (LineTrack_GetTelemetrySnapshot(&track) == 0U) ||
        (UI_MenuGetSnapshot(&ui) == 0U)) {
        s_last_refresh_tick = now;
        g_oled_display_debug.submit_drop_count++;
        return;
    }

    mode = OLED_GetDisplayMode(&balance);
    OLED_Render(&balance, &track, &ui, mode, now);

    g_oled_display_debug.mode = (uint8_t)mode;
    g_oled_display_debug.page = ui.page;
    g_oled_display_debug.home_detail = ui.home_detail;
    g_oled_display_debug.track_run_detail = ui.track_run_detail;
    g_oled_display_debug.balance_state = balance.state;
    g_oled_display_debug.track_state = track.state;
    g_oled_display_debug.position_mm = balance.position_mm;
    g_oled_display_debug.velocity_mm_s = OLED_RoundFloat(balance.velocity_mm_s, 9999);
    g_oled_display_debug.last_valid_age_ms = balance.last_valid_age_ms;
    g_oled_display_debug.render_count++;
    g_oled_display_debug.last_render_tick = now;

    s_last_refresh_tick = now;
    if (OLED_StartFrame(s_frame_packet, OLED_FRAME_PACKET_SIZE) != 0U) {
        g_oled_display_debug.submit_count++;
    } else {
        g_oled_display_debug.submit_drop_count++;
    }
}
