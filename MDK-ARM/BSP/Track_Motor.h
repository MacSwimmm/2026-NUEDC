#ifndef __TRACK_MOTOR_H
#define __TRACK_MOTOR_H

#include <stdint.h>

#define BSP_MOTOR_PWM_MAX (99)

typedef enum {
    BSP_MOTOR_A = 0,
    BSP_MOTOR_B,
    BSP_MOTOR_COUNT
} bsp_motor_id_t;

void bsp_motor_init(void);
void bsp_motor_set_pwm(bsp_motor_id_t motor, int32_t pwm);
void bsp_motor_set_all_pwm(int32_t motor_a_pwm, int32_t motor_b_pwm);
void bsp_motor_stop_all(void);

#endif /* __TRACK_MOTOR_H */
