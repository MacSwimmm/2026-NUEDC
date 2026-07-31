#include "PID_Tuner.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Board.h"
#include "Car_Control.h"
#include "PID.h"
#include "Serial.h"
#include "app_config.h"

#define PID_TUNER_RX_LINE_LENGTH       96U
#define PID_TUNER_TX_LINE_LENGTH       128U
#define PID_TUNER_TELEMETRY_PERIOD_MS  20U
#define PID_TUNER_COMMAND_TIMEOUT_MS   1000U
#define PID_TUNER_PARAMETER_MAX        5000

static char g_rx_line[PID_TUNER_RX_LINE_LENGTH];
static uint8_t g_rx_index = 0U;
static uint8_t g_host_armed = 0U;
static uint8_t g_stream_enabled = 0U;
static uint32_t g_last_command_ms = 0U;
static uint32_t g_last_telemetry_ms = 0U;

static void SendText(const char *text)
{
    (void)Serial_DebugWrite((const uint8_t *)text, (uint16_t)strlen(text));
}

static char *NextToken(char **cursor)
{
    char *start = *cursor;
    char *end;

    while ((*start == ' ') || (*start == '\t')) start++;
    if (*start == '\0') {
        *cursor = start;
        return NULL;
    }

    end = start;
    while ((*end != '\0') && (*end != ' ') && (*end != '\t')) end++;
    if (*end != '\0') *end++ = '\0';
    *cursor = end;
    return start;
}

static bool ParseInt32(const char *text, int32_t *value)
{
    int32_t result = 0;
    int32_t sign = 1;

    if ((text == NULL) || (value == NULL) || (*text == '\0')) return false;
    if (*text == '-') {
        sign = -1;
        text++;
    } else if (*text == '+') {
        text++;
    }
    if ((*text < '0') || (*text > '9')) return false;

    while (*text != '\0') {
        if ((*text < '0') || (*text > '9')) return false;
        if (result > 1000000) return false;
        result = result * 10 + (int32_t)(*text - '0');
        text++;
    }
    *value = result * sign;
    return true;
}

static bool ParsePair(char *cursor, int32_t *left, int32_t *right)
{
    char *left_text = NextToken(&cursor);
    char *right_text = NextToken(&cursor);

    return ParseInt32(left_text, left) && ParseInt32(right_text, right) &&
           (NextToken(&cursor) == NULL);
}

static int32_t SpeedToMilli(float speed_mps)
{
    float scaled = speed_mps * 1000.0f;
    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

static void SendStatus(void)
{
    char output[PID_TUNER_TX_LINE_LENGTH];
    float left_kp;
    float left_ki;
    float left_limit;
    float right_kp;
    float right_ki;
    float right_limit;
    uint32_t primask = Board_EnterCritical();
    int length;

    left_kp = g_pid_watch.left.kp;
    left_ki = g_pid_watch.left.ki;
    left_limit = g_pid_watch.left.output_limit;
    right_kp = g_pid_watch.right.kp;
    right_ki = g_pid_watch.right.ki;
    right_limit = g_pid_watch.right.output_limit;
    Board_ExitCritical(primask);

    length = snprintf(output, sizeof(output),
                      "P,%u,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                      (unsigned int)g_host_armed,
                      (long)left_kp, (long)left_ki, (long)left_limit,
                      (long)right_kp, (long)right_ki, (long)right_limit);
    if (length <= 0) return;
    if (length >= (int)sizeof(output)) length = (int)sizeof(output) - 1;
    (void)Serial_DebugWrite((const uint8_t *)output, (uint16_t)length);
}

static void SendTelemetry(uint32_t now)
{
    CarTelemetry_t car;
    char output[PID_TUNER_TX_LINE_LENGTH];
    int32_t left_output;
    int32_t right_output;
    uint32_t primask;
    int length;

    Car_ControlGetTelemetry(&car);
    primask = Board_EnterCritical();
    left_output = (int32_t)g_pid_watch.left.output;
    right_output = (int32_t)g_pid_watch.right.output;
    Board_ExitCritical(primask);

    length = snprintf(output, sizeof(output),
                      "T,%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld\r\n",
                      (unsigned long)now,
                      (long)SpeedToMilli(car.left_target_mps),
                      (long)SpeedToMilli(car.left_speed_mps),
                      (long)left_output,
                      (long)SpeedToMilli(car.right_target_mps),
                      (long)SpeedToMilli(car.right_speed_mps),
                      (long)right_output,
                      (long)car.left_encoder_count,
                      (long)car.right_encoder_count);
    if (length <= 0) return;
    if (length >= (int)sizeof(output)) length = (int)sizeof(output) - 1;
    (void)Serial_DebugWrite((const uint8_t *)output, (uint16_t)length);
}

static void ApplyPIDPair(const char *command, int32_t left, int32_t right)
{
    uint32_t primask = Board_EnterCritical();

    if (strcmp(command, "KP") == 0) {
        g_pid_watch.left.kp = (float)left;
        g_pid_watch.right.kp = (float)right;
    } else if (strcmp(command, "KI") == 0) {
        g_pid_watch.left.ki = (float)left;
        g_pid_watch.right.ki = (float)right;
    } else {
        g_pid_watch.left.output_limit = (float)left;
        g_pid_watch.right.output_limit = (float)right;
    }
    g_pid_watch.reset_request = 1U;
    Board_ExitCritical(primask);
}

static void HandleCommand(char *line)
{
    char *cursor = line;
    char *command = NextToken(&cursor);
    int32_t left;
    int32_t right;
    uint32_t now = Board_Millis();

    if (command == NULL) return;

    if ((strcmp(command, "KEEP") == 0) || (strcmp(command, "PING") == 0)) {
        if (NextToken(&cursor) != NULL) {
            SendText("ERR,ARGUMENTS\r\n");
            return;
        }
        g_last_command_ms = now;
        if (command[0] == 'P') SendText("OK,PONG\r\n");
        return;
    }
    if (strcmp(command, "ARM") == 0) {
        char *confirmation = NextToken(&cursor);
        if ((confirmation == NULL) || (strcmp(confirmation, "LIFTED") != 0) ||
            (NextToken(&cursor) != NULL)) {
            SendText("ERR,ARM_CONFIRM\r\n");
            return;
        }
        SetSpeed_Left(0.0f);
        SetSpeed_Right(0.0f);
        g_host_armed = 1U;
        g_last_command_ms = now;
        SendText("OK,ARMED\r\n");
        return;
    }
    if (strcmp(command, "STOP") == 0) {
        if (NextToken(&cursor) != NULL) {
            SendText("ERR,ARGUMENTS\r\n");
            return;
        }
        SetSpeed_Stop();
        g_host_armed = 0U;
        SendText("OK,STOPPED\r\n");
        return;
    }
    if (strcmp(command, "STATUS") == 0) {
        if (NextToken(&cursor) != NULL) {
            SendText("ERR,ARGUMENTS\r\n");
            return;
        }
        g_last_command_ms = now;
        SendStatus();
        return;
    }
    if (strcmp(command, "RESET") == 0) {
        uint32_t primask;
        if (NextToken(&cursor) != NULL) {
            SendText("ERR,ARGUMENTS\r\n");
            return;
        }
        primask = Board_EnterCritical();
        g_pid_watch.reset_request = 1U;
        Board_ExitCritical(primask);
        g_last_command_ms = now;
        SendText("OK,RESET\r\n");
        return;
    }
    if (strcmp(command, "STREAM") == 0) {
        char *enabled_text = NextToken(&cursor);
        int32_t enabled;
        if (!ParseInt32(enabled_text, &enabled) ||
            ((enabled != 0) && (enabled != 1)) ||
            (NextToken(&cursor) != NULL)) {
            SendText("ERR,STREAM\r\n");
            return;
        }
        g_stream_enabled = (uint8_t)enabled;
        g_last_command_ms = now;
        SendText("OK,STREAM\r\n");
        return;
    }
    if (strcmp(command, "TARGET") == 0) {
        int32_t maximum = (int32_t)(WHEEL_SPEED_TARGET_LIMIT_MPS * 1000.0f);
        if (!g_host_armed) {
            SendText("ERR,DISARMED\r\n");
            return;
        }
        if (!ParsePair(cursor, &left, &right) ||
            (left < -maximum) || (left > maximum) ||
            (right < -maximum) || (right > maximum)) {
            SendText("ERR,TARGET\r\n");
            return;
        }
        SetSpeed_Left((float)left / 1000.0f);
        SetSpeed_Right((float)right / 1000.0f);
        g_last_command_ms = now;
        SendText("OK,TARGET\r\n");
        return;
    }
    if ((strcmp(command, "KP") == 0) ||
        (strcmp(command, "KI") == 0) ||
        (strcmp(command, "LIMIT") == 0)) {
        int32_t maximum = (strcmp(command, "LIMIT") == 0) ?
                          (int32_t)SPEED_PI_OUTPUT_LIMIT_COUNTS :
                          PID_TUNER_PARAMETER_MAX;
        if (!ParsePair(cursor, &left, &right) ||
            (left < 0) || (right < 0) ||
            (left > maximum) || (right > maximum)) {
            SendText("ERR,PARAM\r\n");
            return;
        }
        ApplyPIDPair(command, left, right);
        g_last_command_ms = now;
        SendText("OK,PARAM\r\n");
        SendStatus();
        return;
    }

    SendText("ERR,UNKNOWN\r\n");
}

static void PushByte(uint8_t byte)
{
    if ((byte == '\r') || (byte == '\n')) {
        if (g_rx_index != 0U) {
            g_rx_line[g_rx_index] = '\0';
            HandleCommand(g_rx_line);
            g_rx_index = 0U;
        }
        return;
    }
    if (g_rx_index < (PID_TUNER_RX_LINE_LENGTH - 1U)) {
        g_rx_line[g_rx_index++] = (char)byte;
    } else {
        g_rx_index = 0U;
        SendText("ERR,LINE\r\n");
    }
}

void PIDTuner_Init(void)
{
    Serial_DebugRxInit();
    g_rx_index = 0U;
    g_host_armed = 0U;
    g_stream_enabled = 0U;
    g_last_command_ms = Board_Millis();
    g_last_telemetry_ms = g_last_command_ms;
    SetSpeed_Stop();
    SendText("READY,PID_TUNER,115200\r\n");
}

void PIDTuner_Process(void)
{
    uint8_t byte;
    uint32_t now;

    while (Serial_DebugReadByte(&byte)) PushByte(byte);

    now = Board_Millis();
    if (g_host_armed &&
        ((now - g_last_command_ms) > PID_TUNER_COMMAND_TIMEOUT_MS)) {
        SetSpeed_Stop();
        g_host_armed = 0U;
        SendText("EVENT,TIMEOUT_STOP\r\n");
    }
    if (g_stream_enabled &&
        ((now - g_last_telemetry_ms) >= PID_TUNER_TELEMETRY_PERIOD_MS)) {
        g_last_telemetry_ms = now;
        SendTelemetry(now);
    }
}
