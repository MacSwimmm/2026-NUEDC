#ifndef G3507_BASE_ENCODER_H
#define G3507_BASE_ENCODER_H

#include <stdint.h>

/**
 * @file Encoder.h
 * @brief 两台编码器电机的 A/B 相边沿计数接口。
 *
 * 左右编码器均配置 A、B 两相上升沿中断，即 2 倍频计数。GPIO Group1 ISR
 * 只累计有符号计数，速度和安装方向补偿由 APP 层完成。
 */

/** 清零软件累计值并清除上电期间可能遗留的 GPIO 中断标志。 */
void Encoder_Init(void);

/** 原子读取一个控制周期内的左右增量并清零，输出可正可负。 */
void Encoder_ReadAndReset(int32_t *left_count, int32_t *right_count);

#endif
