#ifndef G3507_BASE_TRACK_SENSOR_H
#define G3507_BASE_TRACK_SENSOR_H

#include <stdint.h>

/**
 * @file TrackSensor.h
 * @brief 八路数字循迹输入的位序、极性统一和加权误差快照。
 *
 * 实车从左到右安装 CH1~CH8，软件固定映射为 bit7~bit0。辅助板原始输出
 * 白色=1、黑色=0，BSP 统一转换后 line_mask 中的 1 始终表示检测到黑线。
 */
typedef struct {
    uint8_t raw_mask;       /* GPIO 原始电平：bit7=CH1 最左，bit0=CH8 最右 */
    uint8_t line_mask;      /* 极性统一后：1 表示该通道检测到黑线 */
    uint8_t active_count;   /* 当前检测到黑线的通道数量 */
    uint8_t ready;          /* 连续获得正常图案后置 1 */
    uint8_t junction;       /* 8 路全黑，通常表示宽线或交叉区域 */
    float error;            /* 约 -70~+70；正值表示黑线位于左侧 */
} TrackSensorData_t;

/** 清空采样快照和启动就绪连续计数。 */
void TrackSensor_Init(void);

/** 采样 8 路 GPIO，更新位图、有效通道数、路口标志和加权误差。 */
void TrackSensor_Sample(void);

/** 在临界区复制完整循迹快照，供 APP 控制中断读取。 */
void TrackSensor_GetData(TrackSensorData_t *data);

#endif
