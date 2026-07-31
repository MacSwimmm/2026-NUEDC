#ifndef G3507_HOST_TEST_TI_MSP_DL_CONFIG_H
#define G3507_HOST_TEST_TI_MSP_DL_CONFIG_H

#include <stdint.h>

/*
 * 仅供电脑端编译 BSP/TrackSensor.c 的最小硬件桩。
 * 各“引脚值”直接使用对应的循迹位图位，测试代码即可用一个 uint8_t 模拟 GPIO。
 */
typedef struct {
    uint8_t unused;
} GPIO_Regs;

extern GPIO_Regs g_test_track_port_a;
extern GPIO_Regs g_test_track_port_b;
extern uint8_t g_test_track_raw_mask;

#define TRACK_A_PORT       (&g_test_track_port_a)
#define TRACK_B_PORT       (&g_test_track_port_b)
#define TRACK_A_CH1_PIN    0x80U
#define TRACK_A_CH2_PIN    0x40U
#define TRACK_A_CH3_PIN    0x20U
#define TRACK_A_CH4_PIN    0x10U
#define TRACK_A_CH5_PIN    0x08U
#define TRACK_A_CH6_PIN    0x04U
#define TRACK_B_CH7_PIN    0x02U
#define TRACK_B_CH8_PIN    0x01U

static inline uint32_t DL_GPIO_readPins(GPIO_Regs *port, uint32_t pin)
{
    (void)port;
    return ((uint32_t)g_test_track_raw_mask & pin);
}

/* TrackSensor_GetData 的临界区在单线程主机测试中无需真实屏蔽中断。 */
static inline uint32_t __get_PRIMASK(void)
{
    return 0U;
}

static inline void __disable_irq(void)
{
}

static inline void __enable_irq(void)
{
}

#endif
