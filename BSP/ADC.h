#ifndef G3507_BASE_ADC_H
#define G3507_BASE_ADC_H

/*
 * 读取 PA15 外部分压后的电池电压，返回单位 V。
 * PA15 没有在天猛星板上自动接电池；未接分压网络时该值没有物理意义。
 */
float ADC_ReadBatteryVoltage(void);

#endif
