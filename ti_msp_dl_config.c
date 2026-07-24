/*
 * Copyright (c) 2023, Texas Instruments Incorporated
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
 *  ============ ti_msp_dl_config.c =============
 *  Configured MSPM0 DriverLib module definitions
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */

#include "ti_msp_dl_config.h"

DL_TimerA_backupConfig gPWM_0Backup;
DL_TimerG_backupConfig gMAGNET_PWMBackup;
DL_TimerG_backupConfig gMECHANISM_PWMBackup;
DL_UART_Main_backupConfig gCAMERA_UARTBackup;

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform any initialization needed before using any board APIs
 */
SYSCONFIG_WEAK void SYSCFG_DL_init(void)
{
    SYSCFG_DL_initPower();
    SYSCFG_DL_GPIO_init();
    /* Module-Specific Initializations*/
    SYSCFG_DL_SYSCTL_init();
    SYSCFG_DL_PWM_0_init();
    SYSCFG_DL_SERVO_PWM_init();
    SYSCFG_DL_MAGNET_PWM_init();
    SYSCFG_DL_MECHANISM_PWM_init();
    SYSCFG_DL_SYSTEM_TICK_init();
    SYSCFG_DL_EXPANSION_I2C_init();
    SYSCFG_DL_DEBUG_UART_init();
    SYSCFG_DL_JY901S_UART_init();
    SYSCFG_DL_ZIGBEE_UART_init();
    SYSCFG_DL_CAMERA_UART_init();
    SYSCFG_DL_ADC12_VOLTAGE_init();
    SYSCFG_DL_SYSTICK_init();
    /* Ensure backup structures have no valid state */
	gPWM_0Backup.backupRdy 	= false;
	gMAGNET_PWMBackup.backupRdy 	= false;
	gMECHANISM_PWMBackup.backupRdy 	= false;

	gCAMERA_UARTBackup.backupRdy 	= false;

}
/*
 * User should take care to save and restore register configuration in application.
 * See Retention Configuration section for more details.
 */
SYSCONFIG_WEAK bool SYSCFG_DL_saveConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_saveConfiguration(PWM_0_INST, &gPWM_0Backup);
	retStatus &= DL_TimerG_saveConfiguration(MAGNET_PWM_INST, &gMAGNET_PWMBackup);
	retStatus &= DL_TimerG_saveConfiguration(MECHANISM_PWM_INST, &gMECHANISM_PWMBackup);
	retStatus &= DL_UART_Main_saveConfiguration(CAMERA_UART_INST, &gCAMERA_UARTBackup);

    return retStatus;
}


SYSCONFIG_WEAK bool SYSCFG_DL_restoreConfiguration(void)
{
    bool retStatus = true;

	retStatus &= DL_TimerA_restoreConfiguration(PWM_0_INST, &gPWM_0Backup, false);
	retStatus &= DL_TimerG_restoreConfiguration(MAGNET_PWM_INST, &gMAGNET_PWMBackup, false);
	retStatus &= DL_TimerG_restoreConfiguration(MECHANISM_PWM_INST, &gMECHANISM_PWMBackup, false);
	retStatus &= DL_UART_Main_restoreConfiguration(CAMERA_UART_INST, &gCAMERA_UARTBackup);

    return retStatus;
}

SYSCONFIG_WEAK void SYSCFG_DL_initPower(void)
{
    DL_GPIO_reset(GPIOA);
    DL_GPIO_reset(GPIOB);
    DL_TimerA_reset(PWM_0_INST);
    DL_TimerG_reset(SERVO_PWM_INST);
    DL_TimerG_reset(MAGNET_PWM_INST);
    DL_TimerG_reset(MECHANISM_PWM_INST);
    DL_TimerG_reset(SYSTEM_TICK_INST);
    DL_I2C_reset(EXPANSION_I2C_INST);
    DL_UART_Main_reset(DEBUG_UART_INST);
    DL_UART_Main_reset(JY901S_UART_INST);
    DL_UART_Main_reset(ZIGBEE_UART_INST);
    DL_UART_Main_reset(CAMERA_UART_INST);
    DL_ADC12_reset(ADC12_VOLTAGE_INST);


    DL_GPIO_enablePower(GPIOA);
    DL_GPIO_enablePower(GPIOB);
    DL_TimerA_enablePower(PWM_0_INST);
    DL_TimerG_enablePower(SERVO_PWM_INST);
    DL_TimerG_enablePower(MAGNET_PWM_INST);
    DL_TimerG_enablePower(MECHANISM_PWM_INST);
    DL_TimerG_enablePower(SYSTEM_TICK_INST);
    DL_I2C_enablePower(EXPANSION_I2C_INST);
    DL_UART_Main_enablePower(DEBUG_UART_INST);
    DL_UART_Main_enablePower(JY901S_UART_INST);
    DL_UART_Main_enablePower(ZIGBEE_UART_INST);
    DL_UART_Main_enablePower(CAMERA_UART_INST);
    DL_ADC12_enablePower(ADC12_VOLTAGE_INST);

    delay_cycles(POWER_STARTUP_DELAY);
}

SYSCONFIG_WEAK void SYSCFG_DL_GPIO_init(void)
{

    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C0_IOMUX,GPIO_PWM_0_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C0_PORT, GPIO_PWM_0_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_PWM_0_C1_IOMUX,GPIO_PWM_0_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_PWM_0_C1_PORT, GPIO_PWM_0_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO_PWM_C0_IOMUX,GPIO_SERVO_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO_PWM_C0_PORT, GPIO_SERVO_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_SERVO_PWM_C1_IOMUX,GPIO_SERVO_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_SERVO_PWM_C1_PORT, GPIO_SERVO_PWM_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MAGNET_PWM_C1_IOMUX,GPIO_MAGNET_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MAGNET_PWM_C1_PORT, GPIO_MAGNET_PWM_C1_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MECHANISM_PWM_C0_IOMUX,GPIO_MECHANISM_PWM_C0_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MECHANISM_PWM_C0_PORT, GPIO_MECHANISM_PWM_C0_PIN);
    DL_GPIO_initPeripheralOutputFunction(GPIO_MECHANISM_PWM_C1_IOMUX,GPIO_MECHANISM_PWM_C1_IOMUX_FUNC);
    DL_GPIO_enableOutput(GPIO_MECHANISM_PWM_C1_PORT, GPIO_MECHANISM_PWM_C1_PIN);

    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_EXPANSION_I2C_IOMUX_SDA,
        GPIO_EXPANSION_I2C_IOMUX_SDA_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(GPIO_EXPANSION_I2C_IOMUX_SCL,
        GPIO_EXPANSION_I2C_IOMUX_SCL_FUNC, DL_GPIO_INVERSION_DISABLE,
        DL_GPIO_RESISTOR_NONE, DL_GPIO_HYSTERESIS_DISABLE,
        DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_EXPANSION_I2C_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_EXPANSION_I2C_IOMUX_SCL);

    DL_GPIO_initPeripheralOutputFunction(
        GPIO_DEBUG_UART_IOMUX_TX, GPIO_DEBUG_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_DEBUG_UART_IOMUX_RX, GPIO_DEBUG_UART_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_JY901S_UART_IOMUX_TX, GPIO_JY901S_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_JY901S_UART_IOMUX_RX, GPIO_JY901S_UART_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_ZIGBEE_UART_IOMUX_TX, GPIO_ZIGBEE_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_ZIGBEE_UART_IOMUX_RX, GPIO_ZIGBEE_UART_IOMUX_RX_FUNC);
    DL_GPIO_initPeripheralOutputFunction(
        GPIO_CAMERA_UART_IOMUX_TX, GPIO_CAMERA_UART_IOMUX_TX_FUNC);
    DL_GPIO_initPeripheralInputFunction(
        GPIO_CAMERA_UART_IOMUX_RX, GPIO_CAMERA_UART_IOMUX_RX_FUNC);

    DL_GPIO_initDigitalOutput(OLED_RST_PIN_RST_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_DC_PIN_DC_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_SCL_PIN_SCL_IOMUX);

    DL_GPIO_initDigitalOutput(OLED_SDA_PIN_SDA_IOMUX);

    DL_GPIO_initDigitalInputFeatures(USER_KEY_PIN_KEY_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalOutput(BOARD_LED_PIN_LED_IOMUX);

    DL_GPIO_initDigitalOutput(FLASH_GUARD_PIN_CS_IOMUX);

    DL_GPIO_initDigitalOutput(BUZZER_PIN_BUZZER_IOMUX);

    DL_GPIO_initDigitalOutput(GENERAL_EN_PIN_EN_IOMUX);

    DL_GPIO_initDigitalOutput(BIN_BIN1_IOMUX);

    DL_GPIO_initDigitalOutput(BIN_BIN2_IOMUX);

    DL_GPIO_initDigitalOutput(AIN_AIN1_IOMUX);

    DL_GPIO_initDigitalOutput(AIN_AIN2_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERA_E1A_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERA_E1B_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERB_E2A_IOMUX);

    DL_GPIO_initDigitalInput(ENCODERB_E2B_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_A_CH1_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_A_CH2_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_A_CH3_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_A_CH4_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_A_CH5_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_A_CH6_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_B_CH7_IOMUX);

    DL_GPIO_initDigitalInput(TRACK_B_CH8_IOMUX);

    DL_GPIO_initDigitalInputFeatures(EXT_KEYS_KEY1_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_initDigitalInputFeatures(EXT_KEYS_KEY2_IOMUX,
		 DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
		 DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);

    DL_GPIO_clearPins(GPIOA, GENERAL_EN_PIN_EN_PIN |
		BIN_BIN1_PIN |
		BIN_BIN2_PIN |
		AIN_AIN1_PIN |
		AIN_AIN2_PIN);
    DL_GPIO_setPins(GPIOA, OLED_SCL_PIN_SCL_PIN |
		OLED_SDA_PIN_SDA_PIN);
    DL_GPIO_enableOutput(GPIOA, OLED_SCL_PIN_SCL_PIN |
		OLED_SDA_PIN_SDA_PIN |
		GENERAL_EN_PIN_EN_PIN |
		BIN_BIN1_PIN |
		BIN_BIN2_PIN |
		AIN_AIN1_PIN |
		AIN_AIN2_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOA, DL_GPIO_PIN_26_EDGE_RISE |
		DL_GPIO_PIN_25_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOA, ENCODERA_E1A_PIN |
		ENCODERA_E1B_PIN);
    DL_GPIO_enableInterrupt(GPIOA, ENCODERA_E1A_PIN |
		ENCODERA_E1B_PIN);
    DL_GPIO_clearPins(GPIOB, BOARD_LED_PIN_LED_PIN |
		BUZZER_PIN_BUZZER_PIN);
    DL_GPIO_setPins(GPIOB, OLED_RST_PIN_RST_PIN |
		OLED_DC_PIN_DC_PIN |
		FLASH_GUARD_PIN_CS_PIN);
    DL_GPIO_enableOutput(GPIOB, OLED_RST_PIN_RST_PIN |
		OLED_DC_PIN_DC_PIN |
		BOARD_LED_PIN_LED_PIN |
		FLASH_GUARD_PIN_CS_PIN |
		BUZZER_PIN_BUZZER_PIN);
    DL_GPIO_setUpperPinsPolarity(GPIOB, DL_GPIO_PIN_24_EDGE_RISE |
		DL_GPIO_PIN_20_EDGE_RISE);
    DL_GPIO_clearInterruptStatus(GPIOB, ENCODERB_E2A_PIN |
		ENCODERB_E2B_PIN);
    DL_GPIO_enableInterrupt(GPIOB, ENCODERB_E2A_PIN |
		ENCODERB_E2B_PIN);

}


static const DL_SYSCTL_SYSPLLConfig gSYSPLLConfig = {
    .inputFreq              = DL_SYSCTL_SYSPLL_INPUT_FREQ_16_32_MHZ,
	.rDivClk2x              = 1,
	.rDivClk1               = 0,
	.rDivClk0               = 0,
	.enableCLK2x            = DL_SYSCTL_SYSPLL_CLK2X_ENABLE,
	.enableCLK1             = DL_SYSCTL_SYSPLL_CLK1_DISABLE,
	.enableCLK0             = DL_SYSCTL_SYSPLL_CLK0_DISABLE,
	.sysPLLMCLK             = DL_SYSCTL_SYSPLL_MCLK_CLK2X,
	.sysPLLRef              = DL_SYSCTL_SYSPLL_REF_SYSOSC,
	.qDiv                   = 4,
	.pDiv                   = DL_SYSCTL_SYSPLL_PDIV_2
};
SYSCONFIG_WEAK void SYSCFG_DL_SYSCTL_init(void)
{

	//Low Power Mode is configured to be SLEEP0
    DL_SYSCTL_setBORThreshold(DL_SYSCTL_BOR_THRESHOLD_LEVEL_0);
    DL_SYSCTL_setFlashWaitState(DL_SYSCTL_FLASH_WAIT_STATE_2);

    
	DL_SYSCTL_setSYSOSCFreq(DL_SYSCTL_SYSOSC_FREQ_BASE);
	/* Set default configuration */
	DL_SYSCTL_disableHFXT();
	DL_SYSCTL_disableSYSPLL();
    DL_SYSCTL_configSYSPLL((DL_SYSCTL_SYSPLLConfig *) &gSYSPLLConfig);
    DL_SYSCTL_setULPCLKDivider(DL_SYSCTL_ULPCLK_DIV_2);
    DL_SYSCTL_setMCLKSource(SYSOSC, HSCLK, DL_SYSCTL_HSCLK_SOURCE_SYSPLL);
    /* INT_GROUP1 Priority */
    NVIC_SetPriority(GPIOB_INT_IRQn, 0);

}


/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerA_ClockConfig gPWM_0ClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerA_PWMConfig gPWM_0Config = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 8000,
    .isTimerWithFourCC = false,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_PWM_0_init(void) {

    DL_TimerA_setClockConfig(
        PWM_0_INST, (DL_TimerA_ClockConfig *) &gPWM_0ClockConfig);

    DL_TimerA_initPWMMode(
        PWM_0_INST, (DL_TimerA_PWMConfig *) &gPWM_0Config);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_0_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_0_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerA_setCaptureCompareOutCtl(PWM_0_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERA_CAPTURE_COMPARE_1_INDEX);

    DL_TimerA_setCaptCompUpdateMethod(PWM_0_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERA_CAPTURE_COMPARE_1_INDEX);
    DL_TimerA_setCaptureCompareValue(PWM_0_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerA_enableClock(PWM_0_INST);


    
    DL_TimerA_setCCPDirection(PWM_0_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1000000 Hz = 40000000 Hz / (1 * (39 + 1))
 */
static const DL_TimerG_ClockConfig gSERVO_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 39U
};

static const DL_TimerG_PWMConfig gSERVO_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 20000,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SERVO_PWM_init(void) {

    DL_TimerG_setClockConfig(
        SERVO_PWM_INST, (DL_TimerG_ClockConfig *) &gSERVO_PWMClockConfig);

    DL_TimerG_initPWMMode(
        SERVO_PWM_INST, (DL_TimerG_PWMConfig *) &gSERVO_PWMConfig);

    DL_TimerG_setCaptureCompareOutCtl(SERVO_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO_PWM_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(SERVO_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(SERVO_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(SERVO_PWM_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(SERVO_PWM_INST);


    
    DL_TimerG_setCCPDirection(SERVO_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gMAGNET_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gMAGNET_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 4000,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_MAGNET_PWM_init(void) {

    DL_TimerG_setClockConfig(
        MAGNET_PWM_INST, (DL_TimerG_ClockConfig *) &gMAGNET_PWMClockConfig);

    DL_TimerG_initPWMMode(
        MAGNET_PWM_INST, (DL_TimerG_PWMConfig *) &gMAGNET_PWMConfig);

    DL_TimerG_setCaptureCompareOutCtl(MAGNET_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(MAGNET_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(MAGNET_PWM_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(MAGNET_PWM_INST);


    
    DL_TimerG_setCCPDirection(MAGNET_PWM_INST , DL_TIMER_CC1_OUTPUT );


}
/*
 * Timer clock configuration to be sourced by  / 1 (80000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   80000000 Hz = 80000000 Hz / (1 * (0 + 1))
 */
static const DL_TimerG_ClockConfig gMECHANISM_PWMClockConfig = {
    .clockSel = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale = 0U
};

static const DL_TimerG_PWMConfig gMECHANISM_PWMConfig = {
    .pwmMode = DL_TIMER_PWM_MODE_EDGE_ALIGN_UP,
    .period = 8000,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_MECHANISM_PWM_init(void) {

    DL_TimerG_setClockConfig(
        MECHANISM_PWM_INST, (DL_TimerG_ClockConfig *) &gMECHANISM_PWMClockConfig);

    DL_TimerG_initPWMMode(
        MECHANISM_PWM_INST, (DL_TimerG_PWMConfig *) &gMECHANISM_PWMConfig);

    DL_TimerG_setCaptureCompareOutCtl(MECHANISM_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_0_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(MECHANISM_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_0_INDEX);
    DL_TimerG_setCaptureCompareValue(MECHANISM_PWM_INST, 0, DL_TIMER_CC_0_INDEX);

    DL_TimerG_setCaptureCompareOutCtl(MECHANISM_PWM_INST, DL_TIMER_CC_OCTL_INIT_VAL_LOW,
		DL_TIMER_CC_OCTL_INV_OUT_DISABLED, DL_TIMER_CC_OCTL_SRC_FUNCVAL,
		DL_TIMERG_CAPTURE_COMPARE_1_INDEX);

    DL_TimerG_setCaptCompUpdateMethod(MECHANISM_PWM_INST, DL_TIMER_CC_UPDATE_METHOD_IMMEDIATE, DL_TIMERG_CAPTURE_COMPARE_1_INDEX);
    DL_TimerG_setCaptureCompareValue(MECHANISM_PWM_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_enableClock(MECHANISM_PWM_INST);


    
    DL_TimerG_setCCPDirection(MECHANISM_PWM_INST , DL_TIMER_CC0_OUTPUT | DL_TIMER_CC1_OUTPUT );


}



/*
 * Timer clock configuration to be sourced by BUSCLK /  (40000000 Hz)
 * timerClkFreq = (timerClkSrc / (timerClkDivRatio * (timerClkPrescale + 1)))
 *   1250000 Hz = 40000000 Hz / (1 * (31 + 1))
 */
static const DL_TimerG_ClockConfig gSYSTEM_TICKClockConfig = {
    .clockSel    = DL_TIMER_CLOCK_BUSCLK,
    .divideRatio = DL_TIMER_CLOCK_DIVIDE_1,
    .prescale    = 31U,
};

/*
 * Timer load value (where the counter starts from) is calculated as (timerPeriod * timerClockFreq) - 1
 * SYSTEM_TICK_INST_LOAD_VALUE = (1 ms * 1250000 Hz) - 1
 */
static const DL_TimerG_TimerConfig gSYSTEM_TICKTimerConfig = {
    .period     = SYSTEM_TICK_INST_LOAD_VALUE,
    .timerMode  = DL_TIMER_TIMER_MODE_PERIODIC,
    .startTimer = DL_TIMER_START,
};

SYSCONFIG_WEAK void SYSCFG_DL_SYSTEM_TICK_init(void) {

    DL_TimerG_setClockConfig(SYSTEM_TICK_INST,
        (DL_TimerG_ClockConfig *) &gSYSTEM_TICKClockConfig);

    DL_TimerG_initTimerMode(SYSTEM_TICK_INST,
        (DL_TimerG_TimerConfig *) &gSYSTEM_TICKTimerConfig);
    DL_TimerG_enableInterrupt(SYSTEM_TICK_INST , DL_TIMERG_INTERRUPT_ZERO_EVENT);
	NVIC_SetPriority(SYSTEM_TICK_INST_INT_IRQN, 2);
    DL_TimerG_enableClock(SYSTEM_TICK_INST);





}


static const DL_I2C_ClockConfig gEXPANSION_I2CClockConfig = {
    .clockSel = DL_I2C_CLOCK_BUSCLK,
    .divideRatio = DL_I2C_CLOCK_DIVIDE_1,
};

SYSCONFIG_WEAK void SYSCFG_DL_EXPANSION_I2C_init(void) {

    DL_I2C_setClockConfig(EXPANSION_I2C_INST,
        (DL_I2C_ClockConfig *) &gEXPANSION_I2CClockConfig);
    DL_I2C_setAnalogGlitchFilterPulseWidth(EXPANSION_I2C_INST,
        DL_I2C_ANALOG_GLITCH_FILTER_WIDTH_50NS);
    DL_I2C_enableAnalogGlitchFilter(EXPANSION_I2C_INST);

    /* Configure Controller Mode */
    DL_I2C_resetControllerTransfer(EXPANSION_I2C_INST);
    /* Set frequency to 400000 Hz*/
    DL_I2C_setTimerPeriod(EXPANSION_I2C_INST, 9);
    DL_I2C_setControllerTXFIFOThreshold(EXPANSION_I2C_INST, DL_I2C_TX_FIFO_LEVEL_EMPTY);
    DL_I2C_setControllerRXFIFOThreshold(EXPANSION_I2C_INST, DL_I2C_RX_FIFO_LEVEL_BYTES_1);
    DL_I2C_enableControllerClockStretching(EXPANSION_I2C_INST);


    /* Enable module */
    DL_I2C_enableController(EXPANSION_I2C_INST);


}


static const DL_UART_Main_ClockConfig gDEBUG_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gDEBUG_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_DEBUG_UART_init(void)
{
    DL_UART_Main_setClockConfig(DEBUG_UART_INST, (DL_UART_Main_ClockConfig *) &gDEBUG_UARTClockConfig);

    DL_UART_Main_init(DEBUG_UART_INST, (DL_UART_Main_Config *) &gDEBUG_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(DEBUG_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(DEBUG_UART_INST, DEBUG_UART_IBRD_40_MHZ_115200_BAUD, DEBUG_UART_FBRD_40_MHZ_115200_BAUD);



    DL_UART_Main_enable(DEBUG_UART_INST);
}

static const DL_UART_Main_ClockConfig gJY901S_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gJY901S_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_JY901S_UART_init(void)
{
    DL_UART_Main_setClockConfig(JY901S_UART_INST, (DL_UART_Main_ClockConfig *) &gJY901S_UARTClockConfig);

    DL_UART_Main_init(JY901S_UART_INST, (DL_UART_Main_Config *) &gJY901S_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 9600
     *  Actual baud rate: 9599.81
     */
    DL_UART_Main_setOversampling(JY901S_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(JY901S_UART_INST, JY901S_UART_IBRD_40_MHZ_9600_BAUD, JY901S_UART_FBRD_40_MHZ_9600_BAUD);


    /* Configure Interrupts */
    DL_UART_Main_enableInterrupt(JY901S_UART_INST,
                                 DL_UART_MAIN_INTERRUPT_RX);
    /* Setting the Interrupt Priority */
    NVIC_SetPriority(JY901S_UART_INST_INT_IRQN, 1);


    DL_UART_Main_enable(JY901S_UART_INST);
}

static const DL_UART_Main_ClockConfig gZIGBEE_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gZIGBEE_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_ZIGBEE_UART_init(void)
{
    DL_UART_Main_setClockConfig(ZIGBEE_UART_INST, (DL_UART_Main_ClockConfig *) &gZIGBEE_UARTClockConfig);

    DL_UART_Main_init(ZIGBEE_UART_INST, (DL_UART_Main_Config *) &gZIGBEE_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 38400
     *  Actual baud rate: 38396.93
     */
    DL_UART_Main_setOversampling(ZIGBEE_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(ZIGBEE_UART_INST, ZIGBEE_UART_IBRD_40_MHZ_38400_BAUD, ZIGBEE_UART_FBRD_40_MHZ_38400_BAUD);



    DL_UART_Main_enable(ZIGBEE_UART_INST);
}

static const DL_UART_Main_ClockConfig gCAMERA_UARTClockConfig = {
    .clockSel    = DL_UART_MAIN_CLOCK_BUSCLK,
    .divideRatio = DL_UART_MAIN_CLOCK_DIVIDE_RATIO_1
};

static const DL_UART_Main_Config gCAMERA_UARTConfig = {
    .mode        = DL_UART_MAIN_MODE_NORMAL,
    .direction   = DL_UART_MAIN_DIRECTION_TX_RX,
    .flowControl = DL_UART_MAIN_FLOW_CONTROL_NONE,
    .parity      = DL_UART_MAIN_PARITY_NONE,
    .wordLength  = DL_UART_MAIN_WORD_LENGTH_8_BITS,
    .stopBits    = DL_UART_MAIN_STOP_BITS_ONE
};

SYSCONFIG_WEAK void SYSCFG_DL_CAMERA_UART_init(void)
{
    DL_UART_Main_setClockConfig(CAMERA_UART_INST, (DL_UART_Main_ClockConfig *) &gCAMERA_UARTClockConfig);

    DL_UART_Main_init(CAMERA_UART_INST, (DL_UART_Main_Config *) &gCAMERA_UARTConfig);
    /*
     * Configure baud rate by setting oversampling and baud rate divisors.
     *  Target baud rate: 115200
     *  Actual baud rate: 115190.78
     */
    DL_UART_Main_setOversampling(CAMERA_UART_INST, DL_UART_OVERSAMPLING_RATE_16X);
    DL_UART_Main_setBaudRateDivisor(CAMERA_UART_INST, CAMERA_UART_IBRD_80_MHZ_115200_BAUD, CAMERA_UART_FBRD_80_MHZ_115200_BAUD);



    DL_UART_Main_enable(CAMERA_UART_INST);
}

/* ADC12_VOLTAGE Initialization */
static const DL_ADC12_ClockConfig gADC12_VOLTAGEClockConfig = {
    .clockSel       = DL_ADC12_CLOCK_SYSOSC,
    .divideRatio    = DL_ADC12_CLOCK_DIVIDE_8,
    .freqRange      = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
};
SYSCONFIG_WEAK void SYSCFG_DL_ADC12_VOLTAGE_init(void)
{
    DL_ADC12_setClockConfig(ADC12_VOLTAGE_INST, (DL_ADC12_ClockConfig *) &gADC12_VOLTAGEClockConfig);
    DL_ADC12_configConversionMem(ADC12_VOLTAGE_INST, ADC12_VOLTAGE_ADCMEM_0,
        DL_ADC12_INPUT_CHAN_0, DL_ADC12_REFERENCE_VOLTAGE_VDDA, DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT, DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(ADC12_VOLTAGE_INST,24);
    /* Enable ADC12 interrupt */
    DL_ADC12_clearInterruptStatus(ADC12_VOLTAGE_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    DL_ADC12_enableInterrupt(ADC12_VOLTAGE_INST,(DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED));
    NVIC_SetPriority(ADC12_VOLTAGE_INST_INT_IRQN, 3);
    DL_ADC12_enableConversions(ADC12_VOLTAGE_INST);
}

SYSCONFIG_WEAK void SYSCFG_DL_SYSTICK_init(void)
{
    /* Initialize the period to 209.72 ms */
    DL_SYSTICK_init(16777216);
    /* Enable the SysTick and start counting */
    DL_SYSTICK_enable();
}

