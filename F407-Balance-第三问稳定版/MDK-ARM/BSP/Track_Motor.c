#include "Track_Motor.h"
#include "main.h"
#include "tim.h"

/* F4-MIX/bsp/bsp_motor.c 原 TB6612 驱动算法。 */
typedef struct {
    GPIO_TypeDef *in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef *in2_port;
    uint16_t in2_pin;
    uint32_t timer_channel;
} bsp_motor_config_t;

static const bsp_motor_config_t s_motor_config[BSP_MOTOR_COUNT] = {
    [BSP_MOTOR_A] = {
        TRACK_MOTOR_AIN1_GPIO_Port, TRACK_MOTOR_AIN1_Pin,
        TRACK_MOTOR_AIN2_GPIO_Port, TRACK_MOTOR_AIN2_Pin,
        TIM_CHANNEL_1
    },
    [BSP_MOTOR_B] = {
        TRACK_MOTOR_BIN1_GPIO_Port, TRACK_MOTOR_BIN1_Pin,
        TRACK_MOTOR_BIN2_GPIO_Port, TRACK_MOTOR_BIN2_Pin,
        TIM_CHANNEL_2
    }
};

static int32_t bsp_motor_clamp_pwm(int32_t pwm)
{
    if (pwm > BSP_MOTOR_PWM_MAX) return BSP_MOTOR_PWM_MAX;
    if (pwm < -BSP_MOTOR_PWM_MAX) return -BSP_MOTOR_PWM_MAX;
    return pwm;
}

void bsp_motor_init(void)
{
    bsp_motor_stop_all();
    if ((HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1) != HAL_OK) ||
        (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)) {
        Error_Handler();
    }
}

void bsp_motor_set_pwm(bsp_motor_id_t motor, int32_t pwm)
{
    const bsp_motor_config_t *config;
    uint32_t compare;

    if ((uint32_t)motor >= (uint32_t)BSP_MOTOR_COUNT) return;

    config = &s_motor_config[motor];
    pwm = bsp_motor_clamp_pwm(pwm);

    if (pwm > 0) {
        HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);
        compare = (uint32_t)pwm;
    } else if (pwm < 0) {
        HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_SET);
        compare = (uint32_t)(-pwm);
    } else {
        /* 与 F4-MIX 相同：STBY 外部拉高，IN1=IN2=0、PWM=0 为滑行/高阻。 */
        HAL_GPIO_WritePin(config->in1_port, config->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(config->in2_port, config->in2_pin, GPIO_PIN_RESET);
        compare = 0U;
    }

    __HAL_TIM_SET_COMPARE(&htim3, config->timer_channel, compare);
}

void bsp_motor_set_all_pwm(int32_t motor_a_pwm, int32_t motor_b_pwm)
{
    bsp_motor_set_pwm(BSP_MOTOR_A, motor_a_pwm);
    bsp_motor_set_pwm(BSP_MOTOR_B, motor_b_pwm);
}

void bsp_motor_stop_all(void)
{
    bsp_motor_set_all_pwm(0, 0);
}
