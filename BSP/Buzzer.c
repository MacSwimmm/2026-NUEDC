#include "Buzzer.h"

#include "ti_msp_dl_config.h"

static bool g_buzzer_enabled = false;

void Buzzer_Init(void)
{
    Buzzer_Off();
}

void Buzzer_Set(bool enabled)
{
    if (enabled) {
        DL_GPIO_setPins(BUZZER_PORT, BUZZER_PIN_BUZZER_PIN);
    } else {
        DL_GPIO_clearPins(BUZZER_PORT, BUZZER_PIN_BUZZER_PIN);
    }
    g_buzzer_enabled = enabled;
}

void Buzzer_On(void)
{
    Buzzer_Set(true);
}

void Buzzer_Off(void)
{
    Buzzer_Set(false);
}

bool Buzzer_IsOn(void)
{
    return g_buzzer_enabled;
}
