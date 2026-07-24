/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     80000000



/* Defines for PWM_0 */
#define PWM_0_INST                                                         TIMA1
#define PWM_0_INST_IRQHandler                                   TIMA1_IRQHandler
#define PWM_0_INST_INT_IRQN                                     (TIMA1_INT_IRQn)
#define PWM_0_INST_CLK_FREQ                                             80000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_0_C0_PORT                                                 GPIOB
#define GPIO_PWM_0_C0_PIN                                          DL_GPIO_PIN_2
#define GPIO_PWM_0_C0_IOMUX                                      (IOMUX_PINCM15)
#define GPIO_PWM_0_C0_IOMUX_FUNC                     IOMUX_PINCM15_PF_TIMA1_CCP0
#define GPIO_PWM_0_C0_IDX                                    DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_0_C1_PORT                                                 GPIOB
#define GPIO_PWM_0_C1_PIN                                          DL_GPIO_PIN_3
#define GPIO_PWM_0_C1_IOMUX                                      (IOMUX_PINCM16)
#define GPIO_PWM_0_C1_IOMUX_FUNC                     IOMUX_PINCM16_PF_TIMA1_CCP1
#define GPIO_PWM_0_C1_IDX                                    DL_TIMER_CC_1_INDEX

/* Defines for SERVO_PWM */
#define SERVO_PWM_INST                                                     TIMG8
#define SERVO_PWM_INST_IRQHandler                               TIMG8_IRQHandler
#define SERVO_PWM_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
#define SERVO_PWM_INST_CLK_FREQ                                          1000000
/* GPIO defines for channel 0 */
#define GPIO_SERVO_PWM_C0_PORT                                             GPIOB
#define GPIO_SERVO_PWM_C0_PIN                                     DL_GPIO_PIN_10
#define GPIO_SERVO_PWM_C0_IOMUX                                  (IOMUX_PINCM27)
#define GPIO_SERVO_PWM_C0_IOMUX_FUNC                 IOMUX_PINCM27_PF_TIMG8_CCP0
#define GPIO_SERVO_PWM_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_SERVO_PWM_C1_PORT                                             GPIOB
#define GPIO_SERVO_PWM_C1_PIN                                     DL_GPIO_PIN_11
#define GPIO_SERVO_PWM_C1_IOMUX                                  (IOMUX_PINCM28)
#define GPIO_SERVO_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM28_PF_TIMG8_CCP1
#define GPIO_SERVO_PWM_C1_IDX                                DL_TIMER_CC_1_INDEX

/* Defines for MAGNET_PWM */
#define MAGNET_PWM_INST                                                    TIMG7
#define MAGNET_PWM_INST_IRQHandler                              TIMG7_IRQHandler
#define MAGNET_PWM_INST_INT_IRQN                                (TIMG7_INT_IRQn)
#define MAGNET_PWM_INST_CLK_FREQ                                        80000000
/* GPIO defines for channel 1 */
#define GPIO_MAGNET_PWM_C1_PORT                                            GPIOA
#define GPIO_MAGNET_PWM_C1_PIN                                     DL_GPIO_PIN_2
#define GPIO_MAGNET_PWM_C1_IOMUX                                  (IOMUX_PINCM7)
#define GPIO_MAGNET_PWM_C1_IOMUX_FUNC                 IOMUX_PINCM7_PF_TIMG7_CCP1
#define GPIO_MAGNET_PWM_C1_IDX                               DL_TIMER_CC_1_INDEX

/* Defines for MECHANISM_PWM */
#define MECHANISM_PWM_INST                                                 TIMG6
#define MECHANISM_PWM_INST_IRQHandler                           TIMG6_IRQHandler
#define MECHANISM_PWM_INST_INT_IRQN                             (TIMG6_INT_IRQn)
#define MECHANISM_PWM_INST_CLK_FREQ                                     80000000
/* GPIO defines for channel 0 */
#define GPIO_MECHANISM_PWM_C0_PORT                                         GPIOB
#define GPIO_MECHANISM_PWM_C0_PIN                                 DL_GPIO_PIN_26
#define GPIO_MECHANISM_PWM_C0_IOMUX                              (IOMUX_PINCM57)
#define GPIO_MECHANISM_PWM_C0_IOMUX_FUNC             IOMUX_PINCM57_PF_TIMG6_CCP0
#define GPIO_MECHANISM_PWM_C0_IDX                            DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_MECHANISM_PWM_C1_PORT                                         GPIOB
#define GPIO_MECHANISM_PWM_C1_PIN                                 DL_GPIO_PIN_27
#define GPIO_MECHANISM_PWM_C1_IOMUX                              (IOMUX_PINCM58)
#define GPIO_MECHANISM_PWM_C1_IOMUX_FUNC             IOMUX_PINCM58_PF_TIMG6_CCP1
#define GPIO_MECHANISM_PWM_C1_IDX                            DL_TIMER_CC_1_INDEX



/* Defines for SYSTEM_TICK */
#define SYSTEM_TICK_INST                                                 (TIMG0)
#define SYSTEM_TICK_INST_IRQHandler                             TIMG0_IRQHandler
#define SYSTEM_TICK_INST_INT_IRQN                               (TIMG0_INT_IRQn)
#define SYSTEM_TICK_INST_LOAD_VALUE                                      (1249U)




/* Defines for EXPANSION_I2C */
#define EXPANSION_I2C_INST                                                  I2C1
#define EXPANSION_I2C_INST_IRQHandler                            I2C1_IRQHandler
#define EXPANSION_I2C_INST_INT_IRQN                                I2C1_INT_IRQn
#define EXPANSION_I2C_BUS_SPEED_HZ                                        400000
#define GPIO_EXPANSION_I2C_SDA_PORT                                        GPIOA
#define GPIO_EXPANSION_I2C_SDA_PIN                                DL_GPIO_PIN_30
#define GPIO_EXPANSION_I2C_IOMUX_SDA                              (IOMUX_PINCM5)
#define GPIO_EXPANSION_I2C_IOMUX_SDA_FUNC                IOMUX_PINCM5_PF_I2C1_SDA
#define GPIO_EXPANSION_I2C_SCL_PORT                                        GPIOA
#define GPIO_EXPANSION_I2C_SCL_PIN                                DL_GPIO_PIN_29
#define GPIO_EXPANSION_I2C_IOMUX_SCL                              (IOMUX_PINCM4)
#define GPIO_EXPANSION_I2C_IOMUX_SCL_FUNC                IOMUX_PINCM4_PF_I2C1_SCL


/* Defines for DEBUG_UART */
#define DEBUG_UART_INST                                                    UART0
#define DEBUG_UART_INST_IRQHandler                              UART0_IRQHandler
#define DEBUG_UART_INST_INT_IRQN                                  UART0_INT_IRQn
#define GPIO_DEBUG_UART_RX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_TX_PORT                                            GPIOA
#define GPIO_DEBUG_UART_RX_PIN                                    DL_GPIO_PIN_11
#define GPIO_DEBUG_UART_TX_PIN                                    DL_GPIO_PIN_10
#define GPIO_DEBUG_UART_IOMUX_RX                                 (IOMUX_PINCM22)
#define GPIO_DEBUG_UART_IOMUX_TX                                 (IOMUX_PINCM21)
#define GPIO_DEBUG_UART_IOMUX_RX_FUNC                  IOMUX_PINCM22_PF_UART0_RX
#define GPIO_DEBUG_UART_IOMUX_TX_FUNC                  IOMUX_PINCM21_PF_UART0_TX
#define DEBUG_UART_BAUD_RATE                                            (115200)
#define DEBUG_UART_IBRD_40_MHZ_115200_BAUD                                  (21)
#define DEBUG_UART_FBRD_40_MHZ_115200_BAUD                                  (45)
/* Defines for JY901S_UART */
#define JY901S_UART_INST                                                   UART2
#define JY901S_UART_INST_IRQHandler                             UART2_IRQHandler
#define JY901S_UART_INST_INT_IRQN                                 UART2_INT_IRQn
#define GPIO_JY901S_UART_RX_PORT                                           GPIOA
#define GPIO_JY901S_UART_TX_PORT                                           GPIOA
#define GPIO_JY901S_UART_RX_PIN                                   DL_GPIO_PIN_22
#define GPIO_JY901S_UART_TX_PIN                                   DL_GPIO_PIN_21
#define GPIO_JY901S_UART_IOMUX_RX                                (IOMUX_PINCM47)
#define GPIO_JY901S_UART_IOMUX_TX                                (IOMUX_PINCM46)
#define GPIO_JY901S_UART_IOMUX_RX_FUNC                 IOMUX_PINCM47_PF_UART2_RX
#define GPIO_JY901S_UART_IOMUX_TX_FUNC                 IOMUX_PINCM46_PF_UART2_TX
#define JY901S_UART_BAUD_RATE                                             (9600)
#define JY901S_UART_IBRD_40_MHZ_9600_BAUD                                  (260)
#define JY901S_UART_FBRD_40_MHZ_9600_BAUD                                   (27)
/* Defines for ZIGBEE_UART */
#define ZIGBEE_UART_INST                                                   UART1
#define ZIGBEE_UART_INST_IRQHandler                             UART1_IRQHandler
#define ZIGBEE_UART_INST_INT_IRQN                                 UART1_INT_IRQn
#define GPIO_ZIGBEE_UART_RX_PORT                                           GPIOB
#define GPIO_ZIGBEE_UART_TX_PORT                                           GPIOB
#define GPIO_ZIGBEE_UART_RX_PIN                                    DL_GPIO_PIN_5
#define GPIO_ZIGBEE_UART_TX_PIN                                    DL_GPIO_PIN_4
#define GPIO_ZIGBEE_UART_IOMUX_RX                                (IOMUX_PINCM18)
#define GPIO_ZIGBEE_UART_IOMUX_TX                                (IOMUX_PINCM17)
#define GPIO_ZIGBEE_UART_IOMUX_RX_FUNC                 IOMUX_PINCM18_PF_UART1_RX
#define GPIO_ZIGBEE_UART_IOMUX_TX_FUNC                 IOMUX_PINCM17_PF_UART1_TX
#define ZIGBEE_UART_BAUD_RATE                                            (38400)
#define ZIGBEE_UART_IBRD_40_MHZ_38400_BAUD                                  (65)
#define ZIGBEE_UART_FBRD_40_MHZ_38400_BAUD                                   (7)
/* Defines for CAMERA_UART */
#define CAMERA_UART_INST                                                   UART3
#define CAMERA_UART_INST_IRQHandler                             UART3_IRQHandler
#define CAMERA_UART_INST_INT_IRQN                                 UART3_INT_IRQn
#define GPIO_CAMERA_UART_RX_PORT                                           GPIOB
#define GPIO_CAMERA_UART_TX_PORT                                           GPIOB
#define GPIO_CAMERA_UART_RX_PIN                                   DL_GPIO_PIN_13
#define GPIO_CAMERA_UART_TX_PIN                                   DL_GPIO_PIN_12
#define GPIO_CAMERA_UART_IOMUX_RX                                (IOMUX_PINCM30)
#define GPIO_CAMERA_UART_IOMUX_TX                                (IOMUX_PINCM29)
#define GPIO_CAMERA_UART_IOMUX_RX_FUNC                 IOMUX_PINCM30_PF_UART3_RX
#define GPIO_CAMERA_UART_IOMUX_TX_FUNC                 IOMUX_PINCM29_PF_UART3_TX
#define CAMERA_UART_BAUD_RATE                                           (115200)
#define CAMERA_UART_IBRD_80_MHZ_115200_BAUD                                 (43)
#define CAMERA_UART_FBRD_80_MHZ_115200_BAUD                                 (26)





/* Defines for ADC12_VOLTAGE */
#define ADC12_VOLTAGE_INST                                                  ADC1
#define ADC12_VOLTAGE_INST_IRQHandler                            ADC1_IRQHandler
#define ADC12_VOLTAGE_INST_INT_IRQN                              (ADC1_INT_IRQn)
#define ADC12_VOLTAGE_ADCMEM_0                                DL_ADC12_MEM_IDX_0
#define ADC12_VOLTAGE_ADCMEM_0_REF               DL_ADC12_REFERENCE_VOLTAGE_VDDA
#define ADC12_VOLTAGE_ADCMEM_0_REF_VOLTAGE_V                                     3.3
#define GPIO_ADC12_VOLTAGE_C0_PORT                                         GPIOA
#define GPIO_ADC12_VOLTAGE_C0_PIN                                 DL_GPIO_PIN_15



/* Port definition for Pin Group OLED_RST */
#define OLED_RST_PORT                                                    (GPIOB)

/* Defines for PIN_RST: GPIOB.14 with pinCMx 31 on package pin 2 */
#define OLED_RST_PIN_RST_PIN                                    (DL_GPIO_PIN_14)
#define OLED_RST_PIN_RST_IOMUX                                   (IOMUX_PINCM31)
/* Port definition for Pin Group OLED_DC */
#define OLED_DC_PORT                                                     (GPIOB)

/* Defines for PIN_DC: GPIOB.15 with pinCMx 32 on package pin 3 */
#define OLED_DC_PIN_DC_PIN                                      (DL_GPIO_PIN_15)
#define OLED_DC_PIN_DC_IOMUX                                     (IOMUX_PINCM32)
/* Port definition for Pin Group OLED_SCL */
#define OLED_SCL_PORT                                                    (GPIOA)

/* Defines for PIN_SCL: GPIOA.28 with pinCMx 3 on package pin 35 */
#define OLED_SCL_PIN_SCL_PIN                                    (DL_GPIO_PIN_28)
#define OLED_SCL_PIN_SCL_IOMUX                                    (IOMUX_PINCM3)
/* Port definition for Pin Group OLED_SDA */
#define OLED_SDA_PORT                                                    (GPIOA)

/* Defines for PIN_SDA: GPIOA.31 with pinCMx 6 on package pin 39 */
#define OLED_SDA_PIN_SDA_PIN                                    (DL_GPIO_PIN_31)
#define OLED_SDA_PIN_SDA_IOMUX                                    (IOMUX_PINCM6)
/* Port definition for Pin Group USER_KEY */
#define USER_KEY_PORT                                                    (GPIOB)

/* Defines for PIN_KEY: GPIOB.21 with pinCMx 49 on package pin 20 */
#define USER_KEY_PIN_KEY_PIN                                    (DL_GPIO_PIN_21)
#define USER_KEY_PIN_KEY_IOMUX                                   (IOMUX_PINCM49)
/* Port definition for Pin Group BOARD_LED */
#define BOARD_LED_PORT                                                   (GPIOB)

/* Defines for PIN_LED: GPIOB.22 with pinCMx 50 on package pin 21 */
#define BOARD_LED_PIN_LED_PIN                                   (DL_GPIO_PIN_22)
#define BOARD_LED_PIN_LED_IOMUX                                  (IOMUX_PINCM50)
/* Port definition for Pin Group FLASH_GUARD */
#define FLASH_GUARD_PORT                                                 (GPIOB)

/* Defines for PIN_CS: GPIOB.6 with pinCMx 23 on package pin 58 */
#define FLASH_GUARD_PIN_CS_PIN                                   (DL_GPIO_PIN_6)
#define FLASH_GUARD_PIN_CS_IOMUX                                 (IOMUX_PINCM23)
/* Port definition for Pin Group BUZZER */
#define BUZZER_PORT                                                      (GPIOB)

/* Defines for PIN_BUZZER: GPIOB.23 with pinCMx 51 on package pin 22 */
#define BUZZER_PIN_BUZZER_PIN                                   (DL_GPIO_PIN_23)
#define BUZZER_PIN_BUZZER_IOMUX                                  (IOMUX_PINCM51)
/* Port definition for Pin Group GENERAL_EN */
#define GENERAL_EN_PORT                                                  (GPIOA)

/* Defines for PIN_EN: GPIOA.1 with pinCMx 2 on package pin 34 */
#define GENERAL_EN_PIN_EN_PIN                                    (DL_GPIO_PIN_1)
#define GENERAL_EN_PIN_EN_IOMUX                                   (IOMUX_PINCM2)
/* Port definition for Pin Group BIN */
#define BIN_PORT                                                         (GPIOA)

/* Defines for BIN1: GPIOA.16 with pinCMx 38 on package pin 9 */
#define BIN_BIN1_PIN                                            (DL_GPIO_PIN_16)
#define BIN_BIN1_IOMUX                                           (IOMUX_PINCM38)
/* Defines for BIN2: GPIOA.17 with pinCMx 39 on package pin 10 */
#define BIN_BIN2_PIN                                            (DL_GPIO_PIN_17)
#define BIN_BIN2_IOMUX                                           (IOMUX_PINCM39)
/* Port definition for Pin Group AIN */
#define AIN_PORT                                                         (GPIOA)

/* Defines for AIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define AIN_AIN1_PIN                                            (DL_GPIO_PIN_13)
#define AIN_AIN1_IOMUX                                           (IOMUX_PINCM35)
/* Defines for AIN2: GPIOA.14 with pinCMx 36 on package pin 7 */
#define AIN_AIN2_PIN                                            (DL_GPIO_PIN_14)
#define AIN_AIN2_IOMUX                                           (IOMUX_PINCM36)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOA)

/* Defines for E1A: GPIOA.26 with pinCMx 59 on package pin 30 */
// pins affected by this interrupt request:["E1A","E1B"]
#define ENCODERA_INT_IRQN                                       (GPIOA_INT_IRQn)
#define ENCODERA_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO26)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_26)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM59)
/* Defines for E1B: GPIOA.25 with pinCMx 55 on package pin 26 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO25)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_25)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM55)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.24 with pinCMx 52 on package pin 23 */
// pins affected by this interrupt request:["E2A","E2B"]
#define ENCODERB_INT_IRQN                                       (GPIOB_INT_IRQn)
#define ENCODERB_INT_IIDX                       (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO24)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_24)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM52)
/* Defines for E2B: GPIOB.20 with pinCMx 48 on package pin 19 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO20)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_20)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM48)
/* Port definition for Pin Group TRACK_A */
#define TRACK_A_PORT                                                     (GPIOA)

/* Defines for CH1: GPIOA.0 with pinCMx 1 on package pin 33 */
#define TRACK_A_CH1_PIN                                          (DL_GPIO_PIN_0)
#define TRACK_A_CH1_IOMUX                                         (IOMUX_PINCM1)
/* Defines for CH2: GPIOA.7 with pinCMx 14 on package pin 49 */
#define TRACK_A_CH2_PIN                                          (DL_GPIO_PIN_7)
#define TRACK_A_CH2_IOMUX                                        (IOMUX_PINCM14)
/* Defines for CH3: GPIOA.9 with pinCMx 20 on package pin 55 */
#define TRACK_A_CH3_PIN                                          (DL_GPIO_PIN_9)
#define TRACK_A_CH3_IOMUX                                        (IOMUX_PINCM20)
/* Defines for CH4: GPIOA.12 with pinCMx 34 on package pin 5 */
#define TRACK_A_CH4_PIN                                         (DL_GPIO_PIN_12)
#define TRACK_A_CH4_IOMUX                                        (IOMUX_PINCM34)
/* Defines for CH5: GPIOA.24 with pinCMx 54 on package pin 25 */
#define TRACK_A_CH5_PIN                                         (DL_GPIO_PIN_24)
#define TRACK_A_CH5_IOMUX                                        (IOMUX_PINCM54)
/* Defines for CH6: GPIOA.27 with pinCMx 60 on package pin 31 */
#define TRACK_A_CH6_PIN                                         (DL_GPIO_PIN_27)
#define TRACK_A_CH6_IOMUX                                        (IOMUX_PINCM60)
/* Port definition for Pin Group TRACK_B */
#define TRACK_B_PORT                                                     (GPIOB)

/* Defines for CH7: GPIOB.16 with pinCMx 33 on package pin 4 */
#define TRACK_B_CH7_PIN                                         (DL_GPIO_PIN_16)
#define TRACK_B_CH7_IOMUX                                        (IOMUX_PINCM33)
/* Defines for CH8: GPIOB.17 with pinCMx 43 on package pin 14 */
#define TRACK_B_CH8_PIN                                         (DL_GPIO_PIN_17)
#define TRACK_B_CH8_IOMUX                                        (IOMUX_PINCM43)
/* Port definition for Pin Group EXT_KEYS */
#define EXT_KEYS_PORT                                                    (GPIOB)

/* Defines for KEY1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define EXT_KEYS_KEY1_PIN                                        (DL_GPIO_PIN_0)
#define EXT_KEYS_KEY1_IOMUX                                      (IOMUX_PINCM12)
/* Defines for KEY2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define EXT_KEYS_KEY2_PIN                                        (DL_GPIO_PIN_1)
#define EXT_KEYS_KEY2_IOMUX                                      (IOMUX_PINCM13)



/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_0_init(void);
void SYSCFG_DL_SERVO_PWM_init(void);
void SYSCFG_DL_MAGNET_PWM_init(void);
void SYSCFG_DL_MECHANISM_PWM_init(void);
void SYSCFG_DL_SYSTEM_TICK_init(void);
void SYSCFG_DL_EXPANSION_I2C_init(void);
void SYSCFG_DL_DEBUG_UART_init(void);
void SYSCFG_DL_JY901S_UART_init(void);
void SYSCFG_DL_ZIGBEE_UART_init(void);
void SYSCFG_DL_CAMERA_UART_init(void);
void SYSCFG_DL_ADC12_VOLTAGE_init(void);

void SYSCFG_DL_SYSTICK_init(void);

bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
