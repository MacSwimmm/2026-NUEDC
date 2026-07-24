#include "Display.h"

#include "Car_Control.h"
#include "JY901S.h"
#include "OLED.h"
#include "Servo.h"

#include <stdint.h>

#define DISPLAY_PAGE_COUNT 3U

static uint8_t g_page = 0U;

static void ShowSignedInteger(uint8_t x, uint8_t y, int32_t value, uint8_t digits)
{
    uint32_t magnitude;

    if (value < 0) {
        OLED_ShowString(x, y, "-");
        magnitude = (uint32_t)(-value);
    } else {
        OLED_ShowString(x, y, "+");
        magnitude = (uint32_t)value;
    }
    OLED_ShowUnsigned((uint8_t)(x + 8U), y, magnitude, digits);
}

static void ShowFixed1(uint8_t x, uint8_t y, float value, uint8_t integer_digits)
{
    int32_t scaled = (int32_t)(value * 10.0f);
    uint32_t magnitude;

    if (scaled < 0) {
        OLED_ShowString(x, y, "-");
        magnitude = (uint32_t)(-scaled);
    } else {
        OLED_ShowString(x, y, "+");
        magnitude = (uint32_t)scaled;
    }
    OLED_ShowUnsigned((uint8_t)(x + 8U), y, magnitude / 10U, integer_digits);
    OLED_ShowString((uint8_t)(x + 8U + integer_digits * 8U), y, ".");
    OLED_ShowUnsigned((uint8_t)(x + 16U + integer_digits * 8U), y,
                      magnitude % 10U, 1U);
}

static void ShowTrackBits(uint8_t x, uint8_t y, uint8_t mask)
{
    char bits[9];
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        bits[i] = ((mask & (uint8_t)(0x80U >> i)) != 0U) ? '1' : '0';
    }
    bits[8] = '\0';
    OLED_ShowString(x, y, bits);
}

static void DrawChassisPage(const CarTelemetry_t *car)
{
    OLED_ShowString(0U, 0U, car->armed ? "CAR ARM" : "CAR OFF");
    OLED_ShowString(72U, 0U, car->track_ready ? "RDY" : "WAIT");

    OLED_ShowString(0U, 13U, "S:");
    ShowTrackBits(16U, 13U, car->track_mask);
    OLED_ShowString(88U, 13U, "E");
    ShowSignedInteger(96U, 13U, (int32_t)car->track_error, 2U);

    OLED_ShowString(0U, 26U, "LT");
    ShowSignedInteger(16U, 26U, (int32_t)(car->left_target_mps * 1000.0f), 3U);
    OLED_ShowString(64U, 26U, "LV");
    ShowSignedInteger(80U, 26U, (int32_t)(car->left_speed_mps * 1000.0f), 3U);

    OLED_ShowString(0U, 39U, "RT");
    ShowSignedInteger(16U, 39U, (int32_t)(car->right_target_mps * 1000.0f), 3U);
    OLED_ShowString(64U, 39U, "RV");
    ShowSignedInteger(80U, 39U, (int32_t)(car->right_speed_mps * 1000.0f), 3U);

    OLED_ShowString(0U, 52U, "P");
    ShowSignedInteger(8U, 52U, car->left_pwm, 4U);
    ShowSignedInteger(56U, 52U, car->right_pwm, 4U);
}

static void DrawIMUPage(const JY901S_Data_t *imu)
{
    OLED_ShowString(0U, 0U, "JY901S");
    OLED_ShowString(64U, 0U, imu->online ? "ONLINE" : "OFFLINE");

    OLED_ShowString(0U, 13U, "R");
    ShowFixed1(8U, 13U, imu->roll_deg, 3U);
    OLED_ShowString(72U, 13U, "P");
    ShowFixed1(80U, 13U, imu->pitch_deg, 3U);

    OLED_ShowString(0U, 26U, "YAW");
    ShowFixed1(24U, 26U, imu->yaw_deg, 3U);
    OLED_ShowString(80U, 26U, "T");
    ShowFixed1(88U, 26U, imu->temperature_c, 2U);

    OLED_ShowString(0U, 39U, "GX");
    ShowSignedInteger(16U, 39U, (int32_t)imu->gyro_x_dps, 4U);
    OLED_ShowString(72U, 39U, "GY");
    ShowSignedInteger(88U, 39U, (int32_t)imu->gyro_y_dps, 4U);

    OLED_ShowString(0U, 52U, "GZ");
    ShowSignedInteger(16U, 52U, (int32_t)imu->gyro_z_dps, 4U);
    OLED_ShowString(72U, 52U, "AZ");
    ShowFixed1(88U, 52U, imu->acc_z_g, 1U);
}

static void DrawSystemPage(const CarTelemetry_t *car)
{
    uint16_t servo1 = Servo_GetPulseUS(1U);
    uint16_t servo2 = Servo_GetPulseUS(2U);

    OLED_ShowString(0U, 0U, "SYSTEM BASE");
    OLED_ShowString(0U, 13U, "BATT");
    ShowFixed1(40U, 13U, car->battery_voltage, 2U);
    OLED_ShowString(88U, 13U, "V");

    OLED_ShowString(0U, 26U, "BASE mm/s");
    OLED_ShowUnsigned(88U, 26U, (uint32_t)(car->base_speed_mps * 1000.0f), 3U);

    OLED_ShowString(0U, 39U, "SERVO1");
    OLED_ShowUnsigned(64U, 39U, servo1, 4U);
    OLED_ShowString(104U, 39U, "us");

    OLED_ShowString(0U, 52U, "SERVO2");
    OLED_ShowUnsigned(64U, 52U, servo2, 4U);
    OLED_ShowString(104U, 52U, "us");
}

void Display_Init(void)
{
    g_page = 0U;
    OLED_Init();
}

void Display_NextPage(void)
{
    g_page++;
    if (g_page >= DISPLAY_PAGE_COUNT) g_page = 0U;
}

void Display_Update(void)
{
    CarTelemetry_t car;
    JY901S_Data_t imu;

    Car_ControlGetTelemetry(&car);
    JY901S_GetData(&imu);
    OLED_ClearBuffer();

    if (g_page == 0U) DrawChassisPage(&car);
    else if (g_page == 1U) DrawIMUPage(&imu);
    else DrawSystemPage(&car);

    /* 先完成整帧显存绘制，再一次刷新，避免逐字段更新造成闪烁。 */
    OLED_Refresh();
}
