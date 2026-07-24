#include <stdio.h>

#include "Board.h"
#include "Car_Control.h"
#include "Display.h"
#include "JY901S.h"
#include "Key.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

/*
 * main.c 只负责裸机低频调度：处理按键事件、刷新 OLED、输出调试日志并休眠。
 * 5 ms 运动控制由 Board 注册到 TIMG0 中断，主循环绝不直接重复执行闭环。
 */

/** @brief 读取并清空按键事件，再映射为整车应用动作。 */
static void ProcessKeyEvents(void)
{
    uint8_t events = Key_TakeEvents();

    if ((events & KEY_EVENT_USER) != 0U) {
        /* PB21 在“使能/停车”间切换；循迹未就绪时 SetArmed 会安全拒绝。 */
        bool next_state = !Car_ControlIsArmed();
        if (!Car_ControlSetArmed(next_state)) {
            printf("ARM rejected: tracking sensors are not ready.\r\n");
        }
    }
    if ((events & KEY_EVENT_KEY1) != 0U) {
        /* PB0 只切换显示页，不影响控制器状态。 */
        Display_NextPage();
    }
    if ((events & KEY_EVENT_KEY2) != 0U) {
        /* PB1 循环调整基础速度，实际左右目标仍由循迹误差和限幅决定。 */
        Car_ControlCycleBaseSpeed();
    }
}

static void PrintStatus(void)
{
    CarTelemetry_t car;
    JY901S_Data_t imu;

    Car_ControlGetTelemetry(&car);
    JY901S_GetData(&imu);

    /*
     * 全部使用整数输出，避免在 Cortex-M0+ 上引入体积较大的浮点 printf。
     * target/speed 统一换算为 mm/s，yaw 保留 1 位小数，便于串口终端直接观察。
     */
    printf("ms=%lu arm=%u raw=%02X line=%02X err=%ld "
           "target=%ld,%ld speed=%ld,%ld pwm=%ld,%ld "
           "imu=%u yaw=%ld.%ld\r\n",
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
           imu.online ? 1U : 0U,
           (long)imu.yaw_deg,
           (long)((imu.yaw_deg < 0.0f ? -imu.yaw_deg : imu.yaw_deg) * 10.0f) % 10L);
}

int main(void)
{
    uint32_t reset_cause = (uint32_t)DL_SYSCTL_getResetCause();
    uint32_t display_tick;
    uint32_t log_tick;

    /*
     * SysConfig 先初始化时钟、GPIO、PWM、UART、I2C 和定时器寄存器。
     * 此时 NVIC 尚未由 Board_Init 开放，因此各 BSP 可以在无并发环境下清零状态。
     */
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

    /* 两个低频任务使用各自时间戳，uint32_t 相减写法可以自然跨越毫秒计数回绕。 */
    display_tick = Board_Millis();
    log_tick = display_tick;

    while (1) {
        uint32_t now = Board_Millis();

        /* 按键事件由 5 ms 中断产生，在主循环中取走并执行较重的应用动作。 */
        ProcessKeyEvents();

        if ((now - display_tick) >= 100U) {
            /* 固定累加周期而不是赋值 now，可减少偶发延迟造成的长期节拍漂移。 */
            display_tick += 100U;
            Display_Update();
        }
        if ((now - log_tick) >= 500U) {
            log_tick += 500U;
            PrintStatus();
        }

        /* 任务全部完成后休眠，1 ms 时基、编码器或 JY901S 中断都可唤醒 CPU。 */
        __WFI();
    }
}
