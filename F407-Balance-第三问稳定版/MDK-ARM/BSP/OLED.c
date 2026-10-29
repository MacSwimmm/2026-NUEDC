/**
 ******************************************************************************
 * @file    OLED.c
 * @brief   SSD1306 128x64 I2C DMA 刷新、超时检测与自动总线恢复。
 *
 * 正常初始化命令和 1 KiB 显存都由 I2C1 TX DMA 发送。IRQ 中只记录故障并切断
 * 当前传输；HAL DeInit/Init、PB6/PB7 总线释放和 SSD1306 重初始化全部在主循环
 * OLED_Process() 中完成，避免在中断里做重初始化或影响滚球/循迹控制。
 ******************************************************************************
 */

#include "OLED.h"

#include "i2c.h"

#include <string.h>

#define OLED_POWER_STABILIZE_MS           100U
#define OLED_RECOVERY_SETTLE_MS            20U
#define OLED_RECOVERY_RETRY_MS            250U
#define OLED_DMA_DEADLINE_MS               80U

#define OLED_TRANSFER_NONE                  0U
#define OLED_TRANSFER_INIT                  1U
#define OLED_TRANSFER_FRAME                 2U

#define OLED_ERROR_BUS_BUSY        0x80000000UL
#define OLED_ERROR_START_FAILED    0x40000000UL
#define OLED_ERROR_TX_TIMEOUT      0x20000000UL
#define OLED_ERROR_BAD_PACKET      0x10000000UL
#define OLED_ERROR_BAD_CALLBACK    0x08000000UL
#define OLED_ERROR_HANDLE_STUCK    0x04000000UL
#define OLED_ERROR_RECOVERY_FAILED 0x02000000UL

volatile OLED_Debug_t g_oled_debug = {0};

/* 控制字节 0x00 后连续发送 SSD1306 128x64 初始化命令；必须位于 DMA SRAM。 */
static uint8_t s_init_packet[] = {
    0x00U,
    0xAEU,
    0xD5U, 0x80U,
    0xA8U, 0x3FU,
    0xD3U, 0x00U,
    0x40U,
    0x8DU, 0x14U,
    0x20U, 0x00U,
    0xA1U,
    0xC8U,
    0xDAU, 0x12U,
    0x81U, 0x7FU,
    0xD9U, 0xF1U,
    0xDBU, 0x40U,
    0xA4U,
    0xA6U,
    0x21U, 0x00U, 0x7FU,
    0x22U, 0x00U, 0x07U,
    0xAFU
};

static volatile uint8_t s_tx_busy;
static volatile uint8_t s_transfer_kind;
static volatile uint8_t s_fault_pending;
static volatile uint32_t s_pending_error_code;
static uint8_t s_recovering;
static uint32_t s_state_tick;
static uint32_t s_state_wait_ms;
static uint32_t s_tx_start_tick;

static void OLED_BusDelay(void)
{
    volatile uint32_t count;

    /* 只在故障恢复时调用；为开漏总线边沿留出数微秒稳定时间。 */
    for (count = 0U; count < 256U; count++) {
        __NOP();
    }
}

static void OLED_RequestFault(uint32_t error_code)
{
    if (s_fault_pending == 0U) {
        s_pending_error_code = error_code;
        s_fault_pending = 1U;
    }
}

/** @brief IRQ 和主循环均可调用：立即阻止当前故障传输继续产生中断。 */
static void OLED_QuiesceTransfer(void)
{
    uint32_t dma_flags;

    __HAL_I2C_DISABLE_IT(&hi2c1, I2C_IT_EVT | I2C_IT_ERR | I2C_IT_BUF);
    CLEAR_BIT(hi2c1.Instance->CR2, I2C_CR2_DMAEN | I2C_CR2_LAST);

    if (hi2c1.hdmatx != NULL) {
        __HAL_DMA_DISABLE_IT(hi2c1.hdmatx,
                             DMA_IT_TC | DMA_IT_HT | DMA_IT_TE |
                             DMA_IT_DME | DMA_IT_FE);
        __HAL_DMA_DISABLE(hi2c1.hdmatx);
        dma_flags = __HAL_DMA_GET_TC_FLAG_INDEX(hi2c1.hdmatx) |
                    __HAL_DMA_GET_HT_FLAG_INDEX(hi2c1.hdmatx) |
                    __HAL_DMA_GET_TE_FLAG_INDEX(hi2c1.hdmatx) |
                    __HAL_DMA_GET_DME_FLAG_INDEX(hi2c1.hdmatx) |
                    __HAL_DMA_GET_FE_FLAG_INDEX(hi2c1.hdmatx);
        __HAL_DMA_CLEAR_FLAG(hi2c1.hdmatx, dma_flags);
    }

    s_tx_busy = 0U;
    s_transfer_kind = OLED_TRANSFER_NONE;
    g_oled_debug.tx_busy = 0U;
    g_oled_debug.transfer_kind = OLED_TRANSFER_NONE;
}

static void OLED_EnterRecovery(uint32_t now,
                               uint32_t error_code,
                               uint8_t timeout)
{
    OLED_QuiesceTransfer();

    s_fault_pending = 0U;
    s_pending_error_code = 0U;
    s_recovering = 1U;
    s_state_tick = now;
    s_state_wait_ms = OLED_RECOVERY_RETRY_MS;

    g_oled_debug.initialized = 0U;
    g_oled_debug.state = (uint8_t)OLED_DRIVER_RECOVERY_WAIT;
    g_oled_debug.last_error_code = error_code;
    g_oled_debug.error_count++;
    g_oled_debug.recovery_count++;
    if (g_oled_debug.consecutive_fault_count < UINT8_MAX) {
        g_oled_debug.consecutive_fault_count++;
    }
    if (timeout != 0U) {
        g_oled_debug.timeout_count++;
    }
}

static void OLED_ConfigureRecoveryPins(void)
{
    GPIO_InitTypeDef gpio = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
}

/** @brief 释放被从机拉住的 SDA，并生成一个明确 STOP。总耗时有界且小于 1 ms。 */
static void OLED_ReleaseBusLines(void)
{
    uint8_t pulse;

    OLED_ConfigureRecoveryPins();
    OLED_BusDelay();
    for (pulse = 0U; pulse < 9U; pulse++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        OLED_BusDelay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        OLED_BusDelay();
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    OLED_BusDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    OLED_BusDelay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    OLED_BusDelay();
}

/** @retval 1=I2C1 和 DMA 已重新初始化；0=稍后继续周期性重试。 */
static uint8_t OLED_RecoverI2cBus(void)
{
    HAL_StatusTypeDef status;

    g_oled_debug.recovery_attempt_count++;
    g_oled_debug.state = (uint8_t)OLED_DRIVER_RECOVERY_BUS;

    HAL_NVIC_DisableIRQ(DMA1_Stream7_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
    HAL_NVIC_ClearPendingIRQ(DMA1_Stream7_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);

    (void)HAL_I2C_DeInit(&hi2c1);
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    hi2c1.State = HAL_I2C_STATE_RESET;
    hi2c1.ErrorCode = HAL_I2C_ERROR_NONE;
    hi2c1.Lock = HAL_UNLOCKED;

    OLED_ReleaseBusLines();
    status = I2C1_RecoverInit();
    g_oled_debug.last_hal_status = (uint8_t)status;
    if (status != HAL_OK) {
        HAL_NVIC_DisableIRQ(DMA1_Stream7_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
        HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
        g_oled_debug.recovery_fail_count++;
        g_oled_debug.last_error_code = OLED_ERROR_RECOVERY_FAILED |
                                         (uint32_t)status |
                                         HAL_I2C_GetError(&hi2c1);
        return 0U;
    }

    HAL_NVIC_ClearPendingIRQ(DMA1_Stream7_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_EV_IRQn);
    HAL_NVIC_ClearPendingIRQ(I2C1_ER_IRQn);
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 15U, 0U);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
    return 1U;
}

static uint8_t OLED_StartPacket(uint8_t *packet,
                                uint16_t length,
                                uint8_t transfer_kind)
{
    HAL_StatusTypeDef status;

    if ((g_oled_debug.enabled == 0U) || (s_fault_pending != 0U) ||
        (s_tx_busy != 0U)) {
        return 0U;
    }
    if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) {
        g_oled_debug.start_fail_count++;
        OLED_RequestFault(OLED_ERROR_HANDLE_STUCK | HAL_I2C_GetError(&hi2c1));
        return 0U;
    }
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET) {
        g_oled_debug.start_fail_count++;
        OLED_RequestFault(OLED_ERROR_BUS_BUSY | HAL_I2C_GetError(&hi2c1));
        return 0U;
    }

    s_transfer_kind = transfer_kind;
    s_tx_busy = 1U;
    s_tx_start_tick = HAL_GetTick();

    g_oled_debug.tx_busy = 1U;
    g_oled_debug.transfer_kind = transfer_kind;
    g_oled_debug.last_tx_start_tick = s_tx_start_tick;
    g_oled_debug.state = (transfer_kind == OLED_TRANSFER_INIT) ?
                         (uint8_t)OLED_DRIVER_INIT_TX :
                         (uint8_t)OLED_DRIVER_FRAME_TX;

    status = HAL_I2C_Master_Transmit_DMA(&hi2c1,
                                         (uint16_t)(OLED_I2C_ADDRESS_7BIT << 1U),
                                         packet,
                                         length);
    g_oled_debug.last_hal_status = (uint8_t)status;
    if (status != HAL_OK) {
        OLED_QuiesceTransfer();
        g_oled_debug.start_fail_count++;
        OLED_RequestFault(OLED_ERROR_START_FAILED |
                          (uint32_t)status |
                          HAL_I2C_GetError(&hi2c1));
        return 0U;
    }

    return 1U;
}

void OLED_Init(void)
{
    memset((void *)&g_oled_debug, 0, sizeof(g_oled_debug));

    s_tx_busy = 0U;
    s_transfer_kind = OLED_TRANSFER_NONE;
    s_fault_pending = 0U;
    s_pending_error_code = 0U;
    s_recovering = 0U;
    s_state_tick = HAL_GetTick();
    s_state_wait_ms = OLED_POWER_STABILIZE_MS;
    s_tx_start_tick = 0U;

    g_oled_debug.enabled = 1U;
    g_oled_debug.address_7bit = OLED_I2C_ADDRESS_7BIT;
    g_oled_debug.last_hal_status = (uint8_t)HAL_OK;
    g_oled_debug.state = (uint8_t)OLED_DRIVER_WAIT_POWER;
}

void OLED_Process(uint32_t now)
{
    if (g_oled_debug.enabled == 0U) {
        return;
    }

    if (s_fault_pending != 0U) {
        OLED_EnterRecovery(now, s_pending_error_code, 0U);
        return;
    }

    if ((s_tx_busy != 0U) &&
        ((uint32_t)(now - s_tx_start_tick) > OLED_DMA_DEADLINE_MS)) {
        OLED_EnterRecovery(now,
                           OLED_ERROR_TX_TIMEOUT | HAL_I2C_GetError(&hi2c1),
                           1U);
        return;
    }

    if ((g_oled_debug.state == (uint8_t)OLED_DRIVER_READY) &&
        (s_tx_busy == 0U) &&
        ((HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) ||
         (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) != RESET))) {
        OLED_EnterRecovery(now,
                           OLED_ERROR_HANDLE_STUCK |
                           HAL_I2C_GetError(&hi2c1),
                           0U);
        return;
    }

    if ((g_oled_debug.state == (uint8_t)OLED_DRIVER_WAIT_POWER) &&
        ((uint32_t)(now - s_state_tick) >= s_state_wait_ms)) {
        (void)OLED_StartPacket(s_init_packet,
                               (uint16_t)sizeof(s_init_packet),
                               OLED_TRANSFER_INIT);
    } else if ((g_oled_debug.state == (uint8_t)OLED_DRIVER_RECOVERY_WAIT) &&
               ((uint32_t)(now - s_state_tick) >= OLED_RECOVERY_RETRY_MS)) {
        if (OLED_RecoverI2cBus() != 0U) {
            s_state_tick = now;
            s_state_wait_ms = OLED_RECOVERY_SETTLE_MS;
            g_oled_debug.last_recovery_tick = now;
            g_oled_debug.state = (uint8_t)OLED_DRIVER_WAIT_POWER;
        } else {
            s_state_tick = now;
            g_oled_debug.state = (uint8_t)OLED_DRIVER_RECOVERY_WAIT;
        }
    }
}

uint8_t OLED_CanAcceptFrame(void)
{
    return (uint8_t)((g_oled_debug.enabled != 0U) &&
                     (g_oled_debug.initialized != 0U) &&
                     (g_oled_debug.state == (uint8_t)OLED_DRIVER_READY) &&
                     (s_fault_pending == 0U) &&
                     (s_tx_busy == 0U) &&
                     (HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY) &&
                     (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY) == RESET));
}

uint8_t OLED_StartFrame(uint8_t *packet, uint16_t length)
{
    if ((packet == NULL) ||
        (length != OLED_FRAME_PACKET_SIZE) ||
        (packet[0] != OLED_DATA_CONTROL_BYTE)) {
        g_oled_debug.start_fail_count++;
        g_oled_debug.error_count++;
        g_oled_debug.last_error_code = OLED_ERROR_BAD_PACKET;
        return 0U;
    }
    if (OLED_CanAcceptFrame() == 0U) {
        return 0U;
    }

    return OLED_StartPacket(packet, length, OLED_TRANSFER_FRAME);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    uint8_t completed_kind;

    if (hi2c->Instance != I2C1) {
        return;
    }
    if (s_fault_pending != 0U) {
        return;
    }

    completed_kind = s_transfer_kind;
    s_tx_busy = 0U;
    s_transfer_kind = OLED_TRANSFER_NONE;

    g_oled_debug.tx_busy = 0U;
    g_oled_debug.transfer_kind = OLED_TRANSFER_NONE;
    g_oled_debug.last_tx_complete_tick = HAL_GetTick();
    g_oled_debug.last_hal_status = (uint8_t)HAL_OK;

    if (completed_kind == OLED_TRANSFER_INIT) {
        g_oled_debug.initialized = 1U;
        g_oled_debug.init_count++;
        g_oled_debug.state = (uint8_t)OLED_DRIVER_READY;
        if (s_recovering != 0U) {
            g_oled_debug.recovery_success_count++;
            g_oled_debug.consecutive_fault_count = 0U;
            s_recovering = 0U;
        }
    } else if (completed_kind == OLED_TRANSFER_FRAME) {
        g_oled_debug.frame_count++;
        g_oled_debug.state = (uint8_t)OLED_DRIVER_READY;
    } else {
        OLED_RequestFault(OLED_ERROR_BAD_CALLBACK);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != I2C1) {
        return;
    }

    OLED_QuiesceTransfer();
    g_oled_debug.last_hal_status = (uint8_t)HAL_ERROR;
    OLED_RequestFault(HAL_I2C_GetError(hi2c));
}
