#ifndef __TRACK_ENCODER_H
#define __TRACK_ENCODER_H

#include <stdint.h>

extern volatile int32_t g_encoderA_cnt;
extern volatile int32_t g_encoderB_cnt;

void bsp_encoder_init(void);
void bsp_encoder_update(void);

#endif /* __TRACK_ENCODER_H */
