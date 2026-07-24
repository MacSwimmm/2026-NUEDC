#ifndef G3507_BASE_TRACK_SENSOR_H
#define G3507_BASE_TRACK_SENSOR_H

#include <stdint.h>

typedef struct {
    uint8_t raw_mask;       /* GPIO 原始电平：bit7=CH8 最左，bit0=CH1 最右 */
    uint8_t line_mask;      /* 极性统一后：1 表示该通道检测到黑线 */
    uint8_t active_count;   /* 当前检测到黑线的通道数量 */
    uint8_t ready;          /* 连续获得正常图案后置 1 */
    uint8_t junction;       /* 8 路全黑，通常表示宽线或交叉区域 */
    float error;            /* 约 -70~+70；正值表示黑线位于左侧 */
} TrackSensorData_t;

void TrackSensor_Init(void);
void TrackSensor_Sample(void);
void TrackSensor_GetData(TrackSensorData_t *data);

#endif
