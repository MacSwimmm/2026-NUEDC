# 2026 年电子设计竞赛代码

**竞赛成绩：省一等奖。**

## 最终版工程

- [F407-Balance 第三问稳定版（最终版）](F407-Balance-第三问稳定版/)：STM32F407 平衡滚球控制工程，包含固件源码、CubeMX/Keil 工程和电脑端遥测工具。
- 版本标记：**最终版 · 省一等奖**。
- Keil 工程入口：`F407-Balance-第三问稳定版/MDK-ARM/F407-Balance.uvprojx`。

以下为原有 MSPM0G3507 循迹小车工程说明。

## G3507-BASE 循迹小车基础工程

本工程面向嘉立创·天猛星 MSPM0G3507，按照 APP / BSP / Algorithm 三层组织。
当前比赛入口提供三个互斥按键模式：PB21 保持现有参数循迹一圈；PB0 使用独立
3.50 倍速度档缓启动后循迹一圈；PB1 使用同一 3.50 倍速度档直线循迹，运行
6 s 后再用 2 s 平滑减速至停止。循迹控制已完整替换为 GitHub 工程
`argen1234-code/ti_mspm0g3507` 提交 `962cdabf36dae1f842d3a552c347d00d8fb631e8`
的八路权重、动态基础速度、差速 PD、最后方向原地搜线和 400 ms 丢线保护逻辑。
源工程的 FreeRTOS、GPIO、电机驱动和轮速 PID 未复制，继续使用本工程现有硬件层、
5 ms 裸机调度、双轮增量式速度 PI 与 3200 PWM 软件安全上限。
PB21 的 `TRACK_FORWARD_SPEED_SCALE=5.08` 保持不变；PB0/PB1 使用独立的
`AUX_TRACK_FORWARD_SPEED_SCALE=3.50`。三种模式共用原有直线/入弯/弯中 P、D、
丢线恢复和双轮速度 PI（150/300），辅助速度档不会反向覆盖 PB21 参数。

> **TB6612 的 STBY 直接接 3.3 V。PA8 在 SysConfig 中没有分配，也没有任何软件控制。**
>
> **工程不再配置 ADC 或电池电压采样，PA15 已解除复用并保持未分配。**

## 1. 为什么一个 PWM 实例可以控制两个电机

SysConfig 中的 PWM_0 是一个 TIMA1 定时器实例，但该定时器同时开放两个独立
比较通道：

~~~text
TIMA1
├─ C0 / PB2 -> TB6612 PWMA -> 左电机
└─ C1 / PB3 -> TB6612 PWMB -> 右电机
~~~

两个通道共用 10 kHz 周期，但比较值可以分别修改，所以左右轮占空比完全独立。
软件也不是用一个编码器估算两轮速度，而是分别接入两台编码器电机的 A/B 相：

~~~text
左编码器 A/B -> 左轮计数 -> 左轮速度 -> 左轮增量式 PI -> TIMA1_C0
右编码器 A/B -> 右轮计数 -> 右轮速度 -> 右轮增量式 PI -> TIMA1_C1
~~~

因此该工程具备真正的双轮独立速度闭环。

## 2. 工程架构

~~~text
G3507-BASE/
├─ APP/
│  ├─ main.c                 裸机任务调度、按键事件和调试状态输出
│  ├─ Car_Control.c/.h       整车使能、5 ms 控制链和遥测快照
│  ├─ Race_Control.c/.h      三按键模式、A 点识别、缓启停和计时
│  ├─ Telemetry_Stream.c/.h  UART3 固定帧循迹诊断遥测
│  ├─ PID_Tuner.c/.h         保留的架空轮速调参协议，比赛入口不初始化
│  ├─ Display.c/.h           OLED 按键模式、速度档、时间和循迹遥测
│  └─ app_config.h           车辆机械参数、方向、PI 和循迹参数
├─ BSP/
│  ├─ bsp_config.h           PWM、传感器极性和执行器硬件参数
│  ├─ Board.c/.h             时基、LED、延时和临界区
│  ├─ Motor.c/.h             TB6612 双路电机驱动
│  ├─ Encoder.c/.h           两套 A/B 相编码器计数
│  ├─ TrackSensor.c/.h       八路数字循迹采样
│  ├─ Key.c/.h               三个按键消抖
│  ├─ Buzzer.c/.h            PB23 有源蜂鸣器
│  ├─ Serial.c/.h            UART0/1 基础接口与 UART3 非阻塞 TX 队列
│  ├─ JY901S.c/.h            JY901S 标准 0x55 帧解析
│  └─ OLED.c/.h              SSD1306 128×64 软件 SPI 驱动
├─ Algorithm/
│  ├─ PID.c/.h               左右轮独立增量式速度 PI
│  └─ LineTrack.c/.h         源工程动态速度、PD、丢线搜线和保护逻辑
├─ keil/                     Keil 工程、启动文件和链接脚本
├─ tools/                    SysConfig、构建和静态检查脚本
├─ G3507-BASE.syscfg         唯一的外设配置源
└─ ti_msp_dl_config.c/.h     SysConfig 自动生成文件
~~~

分层规则：

- BSP 只处理 GPIO、定时器、UART、软件 SPI 等硬件细节。
- Algorithm 只进行 PI 和循迹差速计算。
- APP 负责把传感器、算法、执行器和显示组合成完整车辆功能。
- ti_msp_dl_config.c/.h 由 SysConfig 生成，不手工修改。

## 3. 基础控制逻辑

TIMG0 每 1 ms 产生一次中断，每 5 次执行一轮底盘闭环：

1. 扫描按键并采样八路循迹输入。
2. 原子读取左右编码器增量并清零。
3. 按轮径、减速比和编码器线数换算两轮实际速度。
4. 八路传感器从左到右按 `-350,-250,-150,-50,+50,+150,+250,+350`
   求有效探头平均误差；负值表示线偏左，正值表示线偏右。连续相邻 3/4 路黑
   仍分别生成 A 点弱/强标志，但不会绕过普通循迹公式。
5. 有线时执行移植后的源公式（下列速度均已由 RPM 换算为 m/s）：

~~~text
base       = clamp(20*scale RPM - abs(error)*0.02*scale RPM,
                   12*scale RPM, 20*scale RPM)
derivative = 当前误差 - 上次误差（启动或重捕获首拍为 0）
kp         = 0.035（abs(error)<100）
             0.100（100<=abs(error)<200）
             0.150（abs(error)>=200）
correction = clamp(kp*error + 0.020*derivative, -20 RPM, +20 RPM)
left       = clamp(base + correction, -35*scale RPM, +35*scale RPM)
right      = clamp(base - correction, -35*scale RPM, +35*scale RPM)
~~~

其中 PB21 的 `scale=5.08`，PB0/PB1 的 `scale=3.50`；转向 P/D 与恢复参数不随
速度档切换。

6. 误差绝对值超过 25 才更新最后真实转向。全白后立即向其反方向以双轮
   `±24 RPM` 快速打 60 ms，随后沿同一反方向以 `±12 RPM` 慢速搜线；连续
   全白到 400 ms 后清 PI、零 PWM 并报告故障。
7. 左右目标直接送入现有两套增量式速度 PI，再分别写 TIMA1_C0 和 TIMA1_C1。

旧版入弯/弯中/出弯引导、三阶段全白恢复、误差滤波、D 项滤波、差速斜坡和
左右轮目标斜坡均已移出控制路径。源工程原周期为 10 ms；当前周期为 5 ms，
因此离散 Kd 按时间语义乘 2，其余速度、P、限幅按轮周长直接换算。重新使能前，
传感器仍必须连续 20 ms 检测到至少一路黑线，A 点横线允许启动。

中断优先级按实时性排列：

| 优先级 | 中断 | 用途 |
| --- | --- | --- |
| 0 | GPIOA/GPIOB Group1 | 两套编码器边沿计数 |
| 1 | UART2 | JY901S 字节接收 |
| 2 | TIMG0 | 1 ms 时基与 5 ms 控制 |
| 3 | UART3 | 无线 DAP 遥测 TX FIFO 后台发送 |

## 4. 接线表

所有模块必须与 MSPM0G3507 共地。MCU GPIO 只接受 3.3 V 逻辑，编码器或模块若
输出 5 V，必须先做电平转换。

### 4.1 TB6612 与两台编码器电机

| 功能 | MSPM0 引脚 | 配置 |
| --- | --- | --- |
| 左电机 PWMA | PB2 | TIMA1_C0，约 10 kHz |
| 左电机 AIN1 | PA13 | GPIO 输出 |
| 左电机 AIN2 | PA14 | GPIO 输出 |
| 右电机 PWMB | PB3 | TIMA1_C1，约 10 kHz |
| 右电机 BIN1 | PA16 | GPIO 输出 |
| 右电机 BIN2 | PA17 | GPIO 输出 |
| TB6612 STBY | 不接 MCU | **固定接 3.3 V** |
| 左编码器 A | PA26 | GPIO 上升沿中断 |
| 左编码器 B | PA25 | GPIO 上升沿中断 |
| 右编码器 A | PB24 | GPIO 上升沿中断 |
| 右编码器 B | PB20 | GPIO 上升沿中断 |

TB6612 的电机电源按电机额定电压单独供电；逻辑电源、MCU 和电机电源必须共地。

### 4.2 八路循迹

| 位置 | 通道 | MSPM0 引脚 | 软件位 |
| --- | --- | --- | --- |
| 最左 | CH1 | PA0 | bit7 |
| 左侧 | CH2 | PA7 | bit6 |
| 左侧 | CH3 | PA9 | bit5 |
| 中间偏左 | CH4 | PA12 | bit4 |
| 中间偏右 | CH5 | PA24 | bit3 |
| 右侧 | CH6 | PA27 | bit2 |
| 右侧 | CH7 | PB16 | bit1 |
| 最右 | CH8 | PB17 | bit0 |

上述顺序按实车安装固定为从左到右 CH1～CH8。参考辅助板例程的数字输出定义，
白色为 1、黑色为 0，因此 `TRACK_ACTIVE_LOW_MASK = 0xFF`；极性统一后的
`line_mask` 仍保持 1 表示检测到黑线，便于上层算法直接使用。

### 4.3 128×64 OLED

OLED 使用参考工程已经验证的软件四线 SPI 参数，不占用硬件 SPI 外设。

| OLED 信号 | MSPM0 引脚 |
| --- | --- |
| SCL/CLK | PA28 |
| SDA/DIN | PA31 |
| RST/RES | PB14 |
| DC | PB15 |
| CS | 固定接低电平 |

当前驱动按 SSD1306、128×64、页寻址模式初始化。若实际模块为带列偏移的其他
控制器，需要按模块型号调整列地址，不能只修改显示坐标。

### 4.4 JY901S

| JY901S 信号 | MSPM0 引脚 | 参数 |
| --- | --- | --- |
| JY901S RX | PA21 / UART2_TX | 9600 8N1 |
| JY901S TX | PA22 / UART2_RX | 9600 8N1，RX 中断 |

解析标准 11 字节 0x55 帧：

- 0x51：三轴加速度和温度
- 0x52：三轴角速度和温度
- 0x53：横滚、俯仰、航向和温度

若传感器当前波特率不是 9600，必须先统一传感器设置或修改 SysConfig。

### 4.5 其他已配置接口

| 功能 | 引脚 | 配置/默认状态 |
| --- | --- | --- |
| 板载 CH340 调试 | PA10 TX / PA11 RX | UART0，115200 8N1，轮速 PI 调参与遥测 |
| 透明串口模块 | PB4 TX / PB5 RX | UART1，38400 8N1 |
| 无线 DAP 遥测 | PB12 TX | UART3，115200 8N1，TX-only；PB13 未分配 |
| 扩展 I2C | PA29 SCL / PA30 SDA | I2C1，400 kHz |
| 扩展按键 1/2 | PB0 / PB1 | 内部上拉；PB0 一圈缓启动，PB1 直线循迹 6 s + 2 s 缓停 |
| 有源蜂鸣器 | PB23 | 默认关闭 |
| 通用使能 | PA1 | 默认低电平，暂未提供 BSP 接口 |
| 板载用户按键 | PB21 | 内部上拉，低电平按下 |
| 板载 LED | PB22 | 高电平点亮 |
| W25Q128 CS 保护 | PB6 | 始终保持高电平 |

`APP/PID_Tuner.c` 和电脑端工具继续保留，但当前比赛 `main.c` 不调用
`PIDTuner_Init()`/`PIDTuner_Process()`，UART0 调参协议不会上电运行，也不会与
三按键运行模式争用控制权。UART3 已改为无线 DAP 专用只发送遥测口，不接收任何
电脑命令；UART1 仍只保留基础透明发送接口。
I2C1 只完成 SysConfig 外设和引脚配置，具体扩展器件协议应在后续 BSP 中按题目添加。

### 4.6 明确保留、未分配

| 引脚 | 当前处理 |
| --- | --- |
| PA8 | 不分配；TB6612 STBY 固定接 3.3 V |
| PA15 | 不分配；ADC 功能已从 SysConfig 和工程源码移除 |
| PA2 | 不分配；电磁铁 PWM 已移除 |
| PB10 / PB11 | 不分配；两路舵机 PWM 已移除 |
| PB18 | 不分配，保留模拟扩展 |
| PB19 | 不分配，保留模拟扩展 |
| PB25 | 不分配，保留模拟扩展 |
| PB26 / PB27 | 不分配；机构 PWM 已移除 |

PA18 保留给 BSL，PA19/PA20 保留给 SWD，PA23 保留给 VREF，PB7～PB9 是板载
W25Q128 信号，也不作为普通扩展 GPIO 使用。

## 5. 当前 main.c、比赛流程和 OLED

上电后底盘保持零 PWM。三个低有效按键均经过 20 ms 消抖，同一窗口多键冲突时
只执行停车，不自动选择模式：

1. **PB21**：保持原 `5.08` 倍速度档和原状态机。`LEAVING_A` 连续 3 拍离开
   起点横线后进入 `RUNNING`；超过 8 s 后，第二次强 A 点连续确认 6 拍即停车。
2. **PB0**：使用 `3.50` 倍速度档，前 4 s 将左右轮完整循迹目标从 0 线性放大
   到 100%，随后按与 PB21 相同的 A 点条件完成一圈。
3. **PB1**：同样使用 `3.50` 倍速度档并继续读取八路误差进行循迹，不是开环
   同速直走。前 1.5 s 缓启动；第 6 s 起将完整左右轮目标线性缩小，至第 8 s
   归零、清 PI、写零 PWM，并冻结 `8.000 s`。
4. 任一模式运行中再按任一模式键都先停车；持续丢线则显示 `LOST` 并停车。

OLED 顶行显示 `PB21 LAP 5.08`、`PB0 LAP 3.50` 或 `PB1 LINE 3.50`；第二行
显示状态和 `SSS.mmm` 时间；第三行显示算法实际使用的 CH1~CH8 位图与缓启停
输出百分比；末两行保留左右轮目标/反馈速度。

### 5.1 Keil Watch 推荐变量

进入 Debug 后，直接在 Watch 窗口添加下列表达式并展开结构：

| Watch 表达式 | 用途 |
| --- | --- |
| `g_race_watch` | 比赛状态、终止原因、实时/冻结时间、A 点离开与终点连续计数 |
| `g_car_watch` | 八路输入、源误差/离散微分、P/D、搜线方向、动态速度、轮速和 PWM |
| `g_pid_watch` | 左右轮 PI 参数与目标/反馈/输出；该结构中的参数字段可在线修改 |
| `g_motor_watch` | Motor BSP 最终写入的左右带符号 PWM |
| `g_jy901s_watch` | 保留的 JY901S 解析量与在线状态 |

`g_car_watch` 和 `g_race_watch` 是只读镜像：真实控制代码不读取它们，调试器误写
不会改变车辆状态，下一次 5 ms 控制或主循环刷新时会被真实值覆盖。重点观察：

- 正常循迹：`filtered_error` 是为兼容旧遥测名称保留的未滤波误差，
  `filtered_derivative` 是相邻有效帧差分；正修正表示左轮快、车辆向右转。
- 全白恢复：`recovering=1`，`curve_direction=-1/+1` 分别表示实际向左/向右原地
  搜线；方向固定与最后真实转向相反，持续 400 ms 后 `line_fault=1`。
- A 点停车：`g_race_watch.state` 依次为 0、1、2、3，成功时 `result=1`，
  `final_time_ms` 保持不再变化。

## 6. 需要按实物校准的参数

车辆机械与控制参数集中修改 APP/app_config.h：

| 参数 | 当前基线 | 说明 |
| --- | --- | --- |
| ENCODER_MOTOR_SHAFT_PPR | 13 | 电机轴每转每相脉冲数 |
| ENCODER_COUNT_MULTIPLIER | 2 | 当前 A/B 两相上升沿计数 |
| MOTOR_GEAR_RATIO | 28 | 减速比 |
| WHEEL_CIRCUMFERENCE_M | 0.204203519 | 轮胎实测周长 |
| LEFT/RIGHT_ENCODER_SIGN | +1 / -1 | 编码器正方向 |
| LEFT/RIGHT_MOTOR_OUTPUT_SIGN | -1 / -1 | 电机输出正方向，按当前实车方向确认 |
| SPEED_PI_KP/KI | 150 / 300 | 当前两轮速度 PI 初值，未随循迹移植替换 |
| SPEED_PI_OUTPUT_LIMIT_COUNTS | 3200 | 速度闭环 PWM 硬限幅，Watch 只能调低不能突破 |
| WHEEL_SPEED_TARGET_LIMIT_MPS | 0.45 | SetSpeed_Left/Right 目标速度绝对值上限 |
| TRACK_FORWARD_SPEED_SCALE | 5.08 | PB21 小误差基础速度和目标速度上限倍率 |
| AUX_TRACK_FORWARD_SPEED_SCALE | 3.50 | PB0/PB1 独立循迹速度倍率 |
| TRACK_BASE_SPEED/MAX/MIN_MPS | 0.345785 / 0.345785 / 0.207471 | PB21 的 101.6/101.6/60.96 RPM |
| AUX_TRACK_BASE_SPEED/MIN_MPS | 0.238237 / 0.142942 | PB0/PB1 的 70/42 RPM |
| TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR | 0.000345785 | PB21 每误差单位降速系数 |
| AUX_TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR | 0.000238237 | PB0/PB1 每误差单位降速系数 |
| TRACK_STEER_KP_MPS | 0.000119119 | 直线小误差 Kp=0.035，保持上一版 |
| TRACK_STEER_ENTRY_KP_MPS | 0.000340339 | 入弯 Kp=0.100，abs(error)>=100 |
| TRACK_STEER_CURVE_KP_MPS | 0.000510509 | 弯中 Kp=0.150，abs(error)>=200 |
| TRACK_STEER_KD_MPS | 0.000068068 | 5 ms 离散 D=0.020，保持上一版 |
| TRACK_CORRECTION_LIMIT_MPS | 0.068068 | 三种模式共用差速修正上限 20 RPM |
| TRACK_TARGET_LIMIT_MPS | 0.605123 | PB21 单轮目标上限 177.8 RPM |
| AUX_TRACK_TARGET_LIMIT_MPS | 0.416916 | PB0/PB1 单轮目标上限 122.5 RPM |
| TRACK_RECOVERY_KICK_SPEED_MPS | 0.081681 | 丢线前 60 ms 反向快速打 24 RPM |
| TRACK_RECOVERY_SPEED_MPS | 0.040841 | 后续反向慢速搜线 12 RPM |
| TRACK_LOST_PROTECT_MS | 400 | 连续丢线故障停车时间 |
| TRACK_DIRECTION_DEADBAND | 25 | 更新最后转向所需误差绝对值 |
| RACE_CRUISE/APPROACH_SPEED_MPS | 0.345785 / 0.345785 | PB21 现有请求保持不变 |
| PB0_LAP_LAUNCH_RAMP_MS | 4000 | PB0 从零目标缓慢放大至完整 3.50 档 |
| PB1_STRAIGHT_RUN/DECEL_MS | 6000 / 2000 | PB1 第 6 s 开始缓停，第 8 s 停止 |

硬件与模块参数集中修改 BSP/bsp_config.h：

| 参数 | 当前基线 | 说明 |
| --- | --- | --- |
| MOTOR_PWM_PERIOD_COUNTS | 8000 | TIMA1 双路 PWM 周期 |
| MOTOR_PWM_LIMIT_COUNTS | 7800 | Motor BSP 原始 PWM 最终保护；速度闭环另限 3200 |
| TRACK_ACTIVE_LOW_MASK | 0xFF | 八路均为低电平检测到黑线 |
| TRACK_READY_SAMPLES | 4 | 使能前连续正常采样次数 |
| TRACK_MARKER_WEAK_MIN_CONTIGUOUS | 3 | A 点弱标志：连续相邻黑色探头数 |
| TRACK_MARKER_STRONG_MIN_CONTIGUOUS | 4 | A 点强标志：连续相邻黑色探头数 |
| JY901S_OFFLINE_TIMEOUT_MS | 500 | IMU 离线判定时间 |

电机自带编码器并不代表这些参数对所有型号都相同。铭牌线数可能按单相、双相或
输出轴标注，必须通过手转一圈计数和实际行驶距离校准。

## 7. 生成和编译

本机已经验证的工具：

- MSPM0 SDK：C:\ti\mspm0_sdk_2_01_00_03
- SysConfig：C:\ti\sysconfig_1.20.0
- Keil：C:\Keil_v5\UV4\UV4.exe
- Python：3.13，已安装 pyserial 3.5，Tkinter 8.6

PowerShell：

~~~powershell
cd C:\Users\han_z\Desktop\TEST_TI\G3507-BASE-OFFICIAL
.\tools\run_sysconfig_keil.bat "$PWD" G3507-BASE.syscfg
.\tools\run_control_logic_tests.ps1
.\tools\run_telemetry_tests.ps1
.\tools\static_check.ps1
.\tools\build.cmd
~~~

也可以直接打开 keil/G3507-BASE.uvprojx。Keil 每次构建前会自动运行 SysConfig。
成功构建后生成：

~~~text
keil/Objects/G3507_BASE.axf
keil/Objects/G3507_BASE.hex
~~~

## 8. UART3 无线 DAP 循迹实时监视与 CSV

比赛固件每 10 ms 从只读的 Car/Race 快照生成一个固定 74 字节帧，通过
`UART3_TX=PB12` 发送。UART3 使用 115200 8N1，PB13 不配置为 RX。连接方式：

| MSPM0 | 无线 DAP |
| --- | --- |
| PB12 / UART3_TX | UART RX |
| GND | GND |

不要连接无线 DAP 的 TX 或供电脚；必须确认其串口逻辑电平为 3.3 V TTL。
UART3 的 512 字节软件环形缓冲由最低优先级 TX 中断消费。队列空间不足时整帧
丢弃并递增序号/丢帧计数，绝不在 5 ms 控制中断中打印，也不会等待串口、修改
电机目标、切换比赛状态或调用停车接口。

电脑端工具只读接收，不会像 PID Tuner 一样发送 `KEEP`、`STOP` 或参数命令：

~~~powershell
cd C:\Users\han_z\Desktop\TEST_TI\G3507-BASE
.\tools\run_track_monitor.cmd
# 也可以预先指定无线 DAP 的串口
.\tools\run_track_monitor.cmd --port COM6
~~~

界面持续显示八路探头、比赛状态、原始/源 PD 误差、离散微分、P/D 项、差速修正、
动态基础速度、左右轮目标/反馈、编码器、PWM、丢线状态和无线链路统计，并绘制
最近 10 秒曲线。默认勾选“随比赛自动记录”：检测到任一按键模式启动后清空旧
数据并开始记录；正常完成、持续丢线或再次按模式键人工停止时结束记录并保留样本。
点击“导出 CSV”后默认保存到 `Documents\G3507_Telemetry`，编码为 UTF-8 with BOM，
可直接用 Excel 打开或把文件交给 Codex 分析。

固定帧使用 `A5 5A` 同步头、版本/类型/负载长度、16 位序号、64 字节负载和
CRC16-CCITT-FALSE。CSV 同时记录电脑时间、板端毫秒、控制周期、序号缺口和 MCU
丢帧累计，因此可以区分控制算法问题、无线链路丢包和板端发送队列拥塞。

## 9. Type-C/CH340 动态轮速 PI 调参

本节工具仅作为后续底盘标定入口保留。当前比赛 `main.c` 没有初始化调参协议；
需要重新启用时，应在架空车轮的专用调试入口中显式调用 PID Tuner，而不要与
PB21/PB0/PB1 任一运行模式同时运行。

电脑端工具为 `tools/pid_tuner.py`。它复用板上的 UART0/CH340，实时绘制左右轮
目标速度、编码器反馈和 PI 输出 PWM，并把全部遥测保存到
`tools/pid_logs/pid_时间.csv`。启动方式：

~~~powershell
cd C:\Users\han_z\Desktop\TEST_TI\G3507-BASE-OFFICIAL
.\tools\run_pid_tuner.cmd
# 也可以直接指定端口
.\tools\run_pid_tuner.cmd --port COM6
~~~

若另一台电脑缺少 pyserial：

~~~powershell
python -m pip install pyserial
~~~

建议按以下顺序首次测试：

1. 先断开电机主电源，核对 STBY=3.3 V、全部共地和电机/编码器接线。
2. 烧录当前固件，用 Type-C 连接电脑；设备管理器中应出现 `USB-SERIAL CH340 (COMx)`。
3. 车轮完全架空，再接通电机电源；打开工具并选择 CH340 对应 COM 口。
4. 先手转轮子观察 OLED：车辆前进方向应为正，后退方向应为负。
5. 连接后先将 PWM 限幅保持在 800～1200、目标设为 80 mm/s，应用 PI 参数。
6. 点击“确认架空并 ARM”，再点击“发送目标”；随时可按“紧急停止”。
7. 先分别测试左轮和右轮，再测试两轮相同目标，观察上升时间、超调、稳态误差、
   PWM 是否长期饱和以及左右轮差异。
8. 需要自动尝试时选择单轮并点击“开始受限自动整定”。工具最多运行 8 轮，
   每轮约 2.2 秒，轮间先停车；结束后把评分最佳的 Kp/Ki 写回 MCU RAM 并保持停车。

串口命令均为 ASCII 文本，以换行结束：

| 命令 | 作用 |
| --- | --- |
| `ARM LIFTED` | 明确确认车轮架空并允许速度目标 |
| `STOP` | 立即停车并解除 ARM |
| `KEEP` / `PING` | 喂 1 秒通信看门狗；PING 还返回 PONG |
| `TARGET L R` | 设置左右目标速度，单位 mm/s，范围 -450～+450 |
| `KP L R` / `KI L R` | 在线修改左右 PI 参数，范围 0～5000 |
| `LIMIT L R` | 设置左右闭环 PWM 限幅，不能超过 3200 |
| `RESET` | 清空两侧 PI 历史状态 |
| `STREAM 0/1` | 关闭/打开 20 ms 遥测流 |
| `STATUS` | 返回 ARM、Kp、Ki 和限幅 |

遥测格式如下，速度单位为 mm/s，PWM 单位为定时器计数：

~~~text
T,ms,Ltarget,Lfeedback,Lpwm,Rtarget,Rfeedback,Rpwm,Lcount,Rcount
~~~

安全边界：必须收到 `ARM LIFTED` 才接受速度目标；上位机超过 1 秒无命令自动停车；
目标硬限幅为 ±450 mm/s；速度闭环 PWM 硬限幅为 3200；自动整定还会把被调轮
限制到不高于 1500。关闭窗口、断开串口和取消整定都会尝试先发送 `STOP`。

自动整定是基于阶跃响应的受限启发式：它根据稳态误差、超调、振荡次数和 PWM
饱和比例逐轮调整参数，不保证一次就得到最终值。机械摩擦、供电电压、轮胎负载、
编码器比例和测试目标都会影响结果，因此应在实际常用速度附近再次人工确认。
工具写回的是 RAM 参数，复位后仍恢复 `APP/app_config.h` 中的编译默认值；确认参数后
再把最终结果固化到源码并重新构建。

当前结论是 SysConfig、静态源码和 Keil 构建可在电脑端验证；CH340 实际枚举、
串口收发、真实阶跃曲线和自动整定结果必须接上这块实物板后确认。
