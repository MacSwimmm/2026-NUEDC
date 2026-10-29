#include "Track_Sensor.h"
#include "main.h"

/* F4-MIX/bsp/bsp_track.c 原传感器读取与位置加权算法。 */
volatile bool g_track_c1_ready = false;
volatile uint8_t g_track_c1_raw_level = 0U;

static const uint16_t s_track_pins[8] = {
    TRACK_C1_Pin, TRACK_C2_Pin, TRACK_C3_Pin, TRACK_C4_Pin,
    TRACK_C5_Pin, TRACK_C6_Pin, TRACK_C7_Pin, TRACK_C8_Pin
};

void bsp_track_init(void)
{
    GPIO_InitTypeDef gpio = {0};

    /* 保留 F4-MIX 上电后 C1/PD0 先高阻 3 秒的处理。 */
    gpio.Pin = TRACK_C1_Pin;
    gpio.Mode = GPIO_MODE_ANALOG;
    gpio.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(TRACK_C1_GPIO_Port, &gpio);
    g_track_c1_ready = false;
}

void bsp_track_enable_c1_pullup(void)
{
    GPIO_InitTypeDef gpio = {0};

    gpio.Pin = TRACK_C1_Pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(TRACK_C1_GPIO_Port, &gpio);
    g_track_c1_ready = true;
}

bool bsp_track_get_channel(uint8_t channel)
{
    GPIO_PinState raw;

    if (channel >= 8U) return false;
    if ((channel == 0U) && !g_track_c1_ready) return false;
    raw = HAL_GPIO_ReadPin(GPIOD, s_track_pins[channel]);
    if (channel == 0U) g_track_c1_raw_level = (uint8_t)raw;
    /* F4-MIX 模块定义：白底=0，黑线=1；硬件为低电平检测到黑线。 */
    return raw == GPIO_PIN_RESET;
}

uint8_t bsp_track_read(void)
{
    uint8_t value = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        if (bsp_track_get_channel(i)) {
            value |= (uint8_t)(1U << i);
        }
    }
    return value;
}

int16_t bsp_track_line_position(bool *valid)
{
    return bsp_track_line_position_from_bits(bsp_track_read(), valid);
}

int16_t bsp_track_line_position_from_bits(uint8_t bits, bool *valid)
{
    static const int16_t weights[8] = {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    int32_t sum = 0;
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++) {
        if ((bits & (1U << i)) != 0U) {
            sum += weights[i];
            count++;
        }
    }
    if (valid != NULL) *valid = count != 0U;
    return (count == 0U) ? 0 : (int16_t)(sum / count);
}
