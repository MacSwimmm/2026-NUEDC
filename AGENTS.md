# F407-Balance 工程约束

## 工程用途

本工程是 STM32F407VET6 车载平衡滚球控制器。机构水平上电后，小球可位于任意有效视觉位置；经视觉、通信和零点验证后，把当前摆杆位置建为水平零点，并持续闭环把小球拉回并稳定在 `0 mm`。在 `HOLD_ORIGIN` 连续确认小球稳定后，PA0 每次上电最多触发一次 `0 -> +50 -> -50 mm` 轨迹，最终持续保持在 `-50 mm`。

## 固定硬件基线

- MCU：STM32F407VET6，HSE 8 MHz，系统主频 168 MHz。
- USART1：张大头闭环步进电机总线，PA9/PA10，115200 8N1，RX/TX DMA。
- USART2：摄像头坐标链路，PA2/PA3，115200 8N1，RX/TX DMA。
- USART3：JY901S 或备用链路，PB10/PB11，115200 8N1，RX/TX DMA。
- PA1：板载 LED 输出。
- SWD：PA13/PA14。

张大头协议函数完整保留在 `MDK-ARM/Algorithm/Emm_V5.c/.h`，其发送口固定为 `huart1`。自动闭环只能经过 `Motor_Command` 的所有权、ACK 和右端 `-340 pulse` / 左端 `+330 pulse` 硬限位；正脉冲使小球向左、负脉冲使小球向右，首次上板仍必须确认摄像头坐标、真实水平零点和急停。

## 目录职责

| 路径 | 职责 |
| --- | --- |
| `F407-Balance.ioc` | CubeMX 引脚、时钟、DMA 和 UART 配置源文件 |
| `Core/` | CubeMX/HAL 生成代码，只在 USER CODE 区域加入应用内容 |
| `Drivers/` | ST HAL/CMSIS，不手工修改 |
| `MDK-ARM/Algorithm/Emm_V5.*` | 张大头 V5 完整串口协议 |
| `MDK-ARM/Algorithm/PID.*` | 通用位置式 PID 数学模块 |
| `MDK-ARM/APP/Balance_Control.*` | 目标坐标和未来平衡控制编排 |
| `MDK-ARM/BSP/Camera.*` | MaixCam 文本坐标流解析和 Watch 数据 |
| `MDK-ARM/BSP/JY901S.*` | WIT 0x55/11 字节协议解析和 Watch 数据 |
| `MDK-ARM/BSP/UART_DMA.*` | 三路 ReceiveToIdle DMA 启动、分发和错误恢复 |

## 当前安全行为

- 上电先完成 HAL、GPIO、DMA、USART1/2/3、三路 RX DMA 和目标状态初始化，此阶段不驱动电机。
- PA0 稳定松开、Camera 连续 3 帧识别到任意位置的小球且坐标新鲜后，自动启动一次安全建零流程；机构水平由上电前人工保证。
- 启动流程依次查询电机位置、失能、把当前水平位置清零、读回验证、使能并发送 `0 pulse`。
- 随后 `Balance_ControlLoop()` 进入 `HOLD_ORIGIN`，目标固定为 `0 mm`；视觉丢失只进入 `VISION_HOLD` 冻结新修正并无限等待，收到新有效帧后自动恢复，不因视觉超时故障或停止。
- `HOLD_ORIGIN` 中连续 3 个正常帧满足 `|x|<=10 mm`、`|v|<=20 mm/s` 后，PA0 才可触发一次 `MOVE_POSITIVE(+50) -> MOVE_NEGATIVE(-50) -> HOLD_NEGATIVE(-50)`；拒绝条件不锁存，轨迹不重新建零、不重新使能、不重新获取电机所有权。
- 轨迹完成标准为 `-50 mm` 稳定 300 ms；评分计时从 PA0 消抖接受开始，要求总时间不超过 5 s，+50 折返点/正峰、-50 负谷/最终稳定窗均不越过 ±10 mm 误差边界，且全过程无故障。
- `HOLD_ORIGIN`、任意点稳球和评分轨迹复用 F4-MIX 串级位置/速度 PID；轨迹倍率默认全部为 1.0，若以后单独调倍率，不能无意改变已验证的回 O 默认行为。
- `HOLD_CAPTURED_TARGET` 中 PE3 短按启动慢速前馈循迹；选点页 PE2 长按始终先请求底盘停车，再退出任意点任务并返回滚球菜单。
- `STOP_RETURN_ZERO`、`DISABLE` 和机械硬限位必须保留，不能用低层 `Emm_V5_Pos_Control()` 绕过。
- OLED 继续只读 `Balance_GetTelemetrySnapshot()`；I2C1、DMA1 Stream7/Channel1、100 ms 刷新和主循环最后调用顺序不得因轨迹功能改变。

## Keil 构建

在 `MDK-ARM` 目录运行：

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -r '.\F407-Balance.uvprojx' -o '.\build_output.log'
```

UV4 可能立即返回，必须检查 `build_output.log`、AXF 和 HEX 的时间戳；只有日志明确显示 `0 Error(s)` 才算构建成功。
