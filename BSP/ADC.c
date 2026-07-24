#include "ADC.h"

#include <stdbool.h>

#include "Board.h"
#include "bsp_config.h"
#include "ti_msp_dl_config.h"

static volatile bool g_adc_finished = false;

float ADC_ReadBatteryVoltage(void)
{
    uint32_t start_ms = Board_Millis();
    uint32_t raw;

    g_adc_finished = false;
    DL_ADC12_startConversion(ADC12_VOLTAGE_INST);

    /* 正常转换会很快完成；5 ms 超时只用于避免接线/外设异常造成永久阻塞。 */
    while ((!g_adc_finished) && ((Board_Millis() - start_ms) < 5U)) {
        __WFI();
    }

    raw = DL_ADC12_getMemResult(ADC12_VOLTAGE_INST, ADC12_VOLTAGE_ADCMEM_0);
    return (float)raw * BATTERY_ADC_REFERENCE_V * BATTERY_DIVIDER_RATIO /
           BATTERY_ADC_FULL_SCALE;
}

void ADC12_VOLTAGE_INST_IRQHandler(void)
{
    if (DL_ADC12_getPendingInterrupt(ADC12_VOLTAGE_INST) ==
        DL_ADC12_IIDX_MEM0_RESULT_LOADED) {
        g_adc_finished = true;
    }
}
