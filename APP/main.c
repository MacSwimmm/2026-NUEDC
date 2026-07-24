#include <stdio.h>

#include "ADC.h"
#include "Board.h"
#include "Car_Control.h"
#include "Display.h"
#include "JY901S.h"
#include "Key.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

static void ProcessKeyEvents(void)
{
    uint8_t events = Key_TakeEvents();

    if ((events & KEY_EVENT_USER) != 0U) {
        bool next_state = !Car_ControlIsArmed();
        if (!Car_ControlSetArmed(next_state)) {
            printf("ARM rejected: tracking sensors are not ready.\r\n");
        }
    }
    if ((events & KEY_EVENT_KEY1) != 0U) {
        Display_NextPage();
    }
    if ((events & KEY_EVENT_KEY2) != 0U) {
        Car_ControlCycleBaseSpeed();
    }
}

static void PrintStatus(void)
{
    CarTelemetry_t car;
    JY901S_Data_t imu;

    Car_ControlGetTelemetry(&car);
    JY901S_GetData(&imu);

    /* 全部使用整数输出，避免在 M0+ 上引入体积较大的浮点 printf。 */
    printf("ms=%lu arm=%u raw=%02X line=%02X err=%ld "
           "target=%ld,%ld speed=%ld,%ld pwm=%ld,%ld "
           "batt=%ldmV imu=%u yaw=%ld.%ld\r\n",
           (unsigned long)Board_Millis(),
           (unsigned int)car.armed,
           (unsigned int)car.raw_track_mask,
           (unsigned int)car.track_mask,
           (long)car.track_error,
           (long)(car.left_target_mps * 1000.0f),
           (long)(car.right_target_mps * 1000.0f),
           (long)(car.left_speed_mps * 1000.0f),
           (long)(car.right_speed_mps * 1000.0f),
           (long)car.left_pwm,
           (long)car.right_pwm,
           (long)(car.battery_voltage * 1000.0f),
           imu.online ? 1U : 0U,
           (long)imu.yaw_deg,
           (long)((imu.yaw_deg < 0.0f ? -imu.yaw_deg : imu.yaw_deg) * 10.0f) % 10L);
}

int main(void)
{
    uint32_t reset_cause = (uint32_t)DL_SYSCTL_getResetCause();
    uint32_t adc_tick;
    uint32_t display_tick;
    uint32_t log_tick;

    /* SysConfig 先初始化所有时钟、GPIO、PWM、UART、I2C、ADC 和定时器。 */
    SYSCFG_DL_init();

    /* 应用初始化会先写零电机/执行器，再把两个舵机置于 1500 us 中位。 */
    Car_ControlInit();
    Display_Init();

    /* 最后才开放中断，确保任何异步事件都看见完整的软件初始状态。 */
    Board_Init(Car_ControlStep5ms, APP_CONTROL_PERIOD_MS);

    printf("G3507-BASE boot, reset=%lu\r\n", (unsigned long)reset_cause);
    printf("PB21 arm/stop, PB0 display page, PB1 base speed.\r\n");
    printf("JY901S UART2 PA21/PA22 9600 8N1; motor STBY is tied to 3.3V.\r\n");
    printf("No serial motion-control command is implemented.\r\n");

    adc_tick = Board_Millis();
    display_tick = adc_tick;
    log_tick = adc_tick;

    while (1) {
        uint32_t now = Board_Millis();

        ProcessKeyEvents();

        if ((now - adc_tick) >= 100U) {
            adc_tick += 100U;
            Car_ControlSetBatteryVoltage(ADC_ReadBatteryVoltage());
        }
        if ((now - display_tick) >= 100U) {
            display_tick += 100U;
            Display_Update();
        }
        if ((now - log_tick) >= 500U) {
            log_tick += 500U;
            PrintStatus();
        }

        /* 任务全部完成后休眠，1 ms 时基、编码器、ADC 或 JY901S 都可唤醒 CPU。 */
        __WFI();
    }
}
