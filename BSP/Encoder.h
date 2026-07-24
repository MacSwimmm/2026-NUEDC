#ifndef G3507_BASE_ENCODER_H
#define G3507_BASE_ENCODER_H

#include <stdint.h>

/* 左右编码器都由 GROUP1_IRQHandler 累计；读取函数会原子取值并清零。 */
void Encoder_Init(void);
void Encoder_ReadAndReset(int32_t *left_count, int32_t *right_count);

#endif
