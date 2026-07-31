#include <stdint.h>

#include "Board.h"
#include "Car_Control.h"
#include "Display.h"
#include "Key.h"
#include "Race_Control.h"
#include "Telemetry_Stream.h"
#include "app_config.h"
#include "ti_msp_dl_config.h"

/**
 * @file main.c
 * @brief 赛题第 2 项第一版入口：PB21 启动、一圈循迹、A 点停车与时间显示。
 *
 * 主循环只承担低频流程编排：
 * - 取走已经在 5 ms 中断中完成消抖的 PB21 事件；
 * - 推进 A 点离开/返回判断、1 ms 计时和圈末预减速；
 * - 每 10 ms 把最新只读控制快照压入 UART3 非阻塞发送队列；
 * - 每 50 ms 刷新一次 OLED；
 * - 空闲时执行 WFI 等待下一次中断。
 *
 * 真正的循迹、编码器测速、左右轮 PI 和 PWM 写入全部位于
 * Car_ControlStep5ms()，主循环绝不直接碰电机寄存器。
 *
 * JY901S/UART2 按用户要求保留并初始化，但第一版不读取其加速度或姿态参与控制。
 * PID_Tuner.c/.h 也继续保留供以后架空调速；比赛入口不调用 PIDTuner_Init() 或
 * PIDTuner_Process()，因此不会在上电时抢占 PB21 流程或调用停车接口。
 */

int main(void)
{
    uint32_t oled_tick;

    /*
     * SysConfig 必须最先执行：配置 80 MHz 时钟、底盘 PWM、GPIO、UART 和 1 ms
     * 时基定时器。电磁铁、舵机及机构 PWM 已从配置源移除，相应引脚保持未分配。
     */
    SYSCFG_DL_init();

    /*
     * Car_ControlInit 先把两路电机写成零 PWM，再初始化编码器、循迹、按键、
     * JY901S 和控制器；Race_ControlInit 进一步统一到 WAITING/巡航请求状态。
     */
    Car_ControlInit();
    Race_ControlInit();

    /* OLED 初始化包含硬件复位延时，完成后显示唯一的比赛状态页。 */
    Display_Init();
    Display_Update();

    /*
     * 最后注册 5 ms 控制回调并开放编码器、JY901S 和 TIMG0 中断，再初始化
     * UART3 遥测队列。UART3 TX 优先级最低且不在控制 ISR 中组帧或等待发送。
     */
    Board_Init(Car_ControlStep5ms, APP_CONTROL_PERIOD_MS);
    Telemetry_StreamInit();
    oled_tick = Board_Millis();

    while (1) {
        uint8_t key_events = Key_TakeEvents();
        uint32_t now = Board_Millis();

        /* 本题只使用板载 PB21；PB0/PB1 保留硬件配置，但第一版不分配比赛动作。 */
        if ((key_events & KEY_EVENT_USER) != 0U) {
            Race_ControlHandleUserKey();
        }

        /*
         * 高频推进比赛流程，函数内部只在新的 5 ms 控制样本到达时累计横线计数；
         * 即使主循环一毫秒内运行多次，也不会把同一个 A 点样本重复算作三次。
         */
        Race_ControlProcess();

        /*
         * 只读取本周期已经完成的 Car/Race 快照。若 UART3 队列暂时满，遥测模块
         * 会整帧丢弃并累计计数，绝不阻塞 A 点识别、OLED 或 5 ms 控制闭环。
         */
        Telemetry_StreamProcess();

        /*
         * OLED 以 20 Hz 更新。刷新软件 SPI 显存属于主循环低优先级任务，
         * 期间 1 ms 时基和 5 ms 控制中断仍可抢占，因而不会拖慢循迹闭环。
         */
        if ((now - oled_tick) >= 50U) {
            oled_tick = now;
            Display_Update();
        }

        /*
         * WFI 只暂停 CPU 内核，TIMA1 电机 PWM 和各硬件外设继续运行；TIMG0
         * 最迟 1 ms 后唤醒主循环。这样既不忙等，也不改变固定控制周期。
         */
        __WFI();
    }
}
