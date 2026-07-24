# G3507-BASE 循迹小车基础工程

本工程面向嘉立创·天猛星 MSPM0G3507，按照 APP / BSP / Algorithm 三层组织，
只保留循迹小车基础闭环、参数显示和必要扩展接口。工程采用裸机定时调度，
没有弯道、路口等复杂赛道状态机。

> **TB6612 的 STBY 直接接 3.3 V。PA8 在 SysConfig 中没有分配，也没有任何软件控制。**

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
│  ├─ Display.c/.h           OLED 三页参数显示
│  └─ app_config.h           车辆机械参数、方向、PI 和循迹参数
├─ BSP/
│  ├─ bsp_config.h           PWM、传感器极性、分压和执行器硬件参数
│  ├─ Board.c/.h             时基、LED、延时和临界区
│  ├─ Motor.c/.h             TB6612 双路电机驱动
│  ├─ Encoder.c/.h           两套 A/B 相编码器计数
│  ├─ TrackSensor.c/.h       八路数字循迹采样
│  ├─ Key.c/.h               三个按键消抖
│  ├─ ADC.c/.h               电池电压采样
│  ├─ Servo.c/.h             两路 50 Hz 舵机
│  ├─ Actuator.c/.h          电磁铁、机构 PWM、蜂鸣器和通用使能
│  ├─ Serial.c/.h            调试、透明传输和相机串口发送接口
│  ├─ JY901S.c/.h            JY901S 标准 0x55 帧解析
│  └─ OLED.c/.h              SSD1306 128×64 软件 SPI 驱动
├─ Algorithm/
│  ├─ PID.c/.h               左右轮独立增量式速度 PI
│  └─ LineTrack.c/.h         加权误差与简单 PD 差速
├─ keil/                     Keil 工程、启动文件和链接脚本
├─ tools/                    SysConfig、构建和静态检查脚本
├─ G3507-BASE.syscfg         唯一的外设配置源
└─ ti_msp_dl_config.c/.h     SysConfig 自动生成文件
~~~

分层规则：

- BSP 只处理 GPIO、定时器、UART、ADC 等硬件细节。
- Algorithm 只进行 PI 和循迹差速计算。
- APP 负责把传感器、算法、执行器和显示组合成完整车辆功能。
- ti_msp_dl_config.c/.h 由 SysConfig 生成，不手工修改。

## 3. 基础控制逻辑

TIMG0 每 1 ms 产生一次中断，每 5 次执行一轮底盘闭环：

1. 扫描按键并采样八路循迹输入。
2. 原子读取左右编码器增量并清零。
3. 按轮径、减速比和编码器线数换算两轮实际速度。
4. 八路传感器按 +70 到 -70 加权求横向误差。
5. 简单 PD 生成左右轮目标速度。
6. 左右轮各自执行一套增量式 PI。
7. 分别写入 TIMA1_C0 和 TIMA1_C1。

没有检测到任何黑线时，左右目标、两套 PI 和两个 PWM 会立即清零。重新使能前，
传感器必须连续 20 ms 得到既非全白、也非全黑的正常图案。

中断优先级按实时性排列：

| 优先级 | 中断 | 用途 |
| --- | --- | --- |
| 0 | GPIOA/GPIOB Group1 | 两套编码器边沿计数 |
| 1 | UART2 | JY901S 字节接收 |
| 2 | TIMG0 | 1 ms 时基与 5 ms 控制 |
| 3 | ADC1 | 电池转换完成 |

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
| 板载 CH340 调试 | PA10 TX / PA11 RX | UART0，115200 8N1 |
| 透明串口模块 | PB4 TX / PB5 RX | UART1，38400 8N1 |
| 相机/视觉串口 | PB12 TX / PB13 RX | UART3，115200 8N1 |
| 扩展 I2C | PA29 SCL / PA30 SDA | I2C1，400 kHz |
| 舵机 1 | PB10 | TIMG8_C0，50 Hz，默认 1500 μs |
| 舵机 2 | PB11 | TIMG8_C1，50 Hz，默认 1500 μs |
| 电磁铁功率控制 | PA2 | TIMG7_C1，20 kHz，默认 0% |
| 机构 PWM 0/1 | PB26 / PB27 | TIMG6_C0/C1，默认 0 |
| 电池电压 | PA15 | ADC1_CH0 |
| 扩展按键 1/2 | PB0 / PB1 | 内部上拉，低电平按下 |
| 有源蜂鸣器 | PB23 | 默认关闭 |
| 通用使能 | PA1 | 默认低电平 |
| 板载用户按键 | PB21 | 内部上拉，低电平按下 |
| 板载 LED | PB22 | 高电平点亮 |
| W25Q128 CS 保护 | PB6 | 始终保持高电平 |

UART1 和 UART3 当前只保留基础发送接口，不接收车辆运动指令。I2C1 只完成
SysConfig 外设和引脚配置，具体扩展器件协议应在后续 BSP 中按题目添加。

### 4.6 明确保留、未分配

| 引脚 | 当前处理 |
| --- | --- |
| PA8 | 不分配；TB6612 STBY 固定接 3.3 V |
| PB18 | 不分配，保留模拟扩展 |
| PB19 | 不分配，保留模拟扩展 |
| PB25 | 不分配，保留模拟扩展 |

PA18 保留给 BSL，PA19/PA20 保留给 SWD，PA23 保留给 VREF，PB7～PB9 是板载
W25Q128 信号，也不作为普通扩展 GPIO 使用。

## 5. 按键和 OLED 页面

| 按键 | 功能 |
| --- | --- |
| PB21 板载按键 | 整车使能/停车；循迹输入未就绪时拒绝使能 |
| PB0 KEY1 | 切换 OLED 页面 |
| PB1 KEY2 | 在 0.08～0.30 m/s 间循环调整基础速度 |

OLED 页面：

1. 循迹位图、误差、左右目标速度、实际速度和 PWM。
2. JY901S 在线状态、姿态角、角速度、加速度和温度。
3. 电池电压、基础速度和两路舵机脉宽。

UART0 每 500 ms 输出一行诊断数据，便于架空调试编码器方向、速度和循迹输入。

## 6. 需要按实物校准的参数

车辆机械与控制参数集中修改 APP/app_config.h：

| 参数 | 当前基线 | 说明 |
| --- | --- | --- |
| ENCODER_MOTOR_SHAFT_PPR | 13 | 电机轴每转每相脉冲数 |
| ENCODER_COUNT_MULTIPLIER | 2 | 当前 A/B 两相上升沿计数 |
| MOTOR_GEAR_RATIO | 28 | 减速比 |
| WHEEL_CIRCUMFERENCE_M | 0.204203519 | 轮胎实测周长 |
| LEFT/RIGHT_ENCODER_SIGN | +1 / -1 | 编码器正方向 |
| LEFT/RIGHT_MOTOR_OUTPUT_SIGN | -1 / +1 | 电机输出正方向 |
| SPEED_PI_KP/KI | 400 / 400 | 两轮速度 PI 初值 |
| TRACK_STEER_KP/KD_MPS | 0.0022 / 0.0008 | 循迹差速初值 |

硬件与模块参数集中修改 BSP/bsp_config.h：

| 参数 | 当前基线 | 说明 |
| --- | --- | --- |
| MOTOR_PWM_PERIOD_COUNTS | 8000 | TIMA1 双路 PWM 周期 |
| MOTOR_PWM_LIMIT_COUNTS | 7800 | 电机 PWM 安全上限 |
| TRACK_ACTIVE_LOW_MASK | 0xFF | 八路均为低电平检测到黑线 |
| TRACK_READY_SAMPLES | 4 | 使能前连续正常采样次数 |
| BATTERY_DIVIDER_RATIO | 11 | 10:1 外部分压的总恢复倍率 |
| SERVO_MIN/NEUTRAL/MAX_PULSE_US | 1000/1500/2000 | 舵机脉宽范围 |
| JY901S_OFFLINE_TIMEOUT_MS | 500 | IMU 离线判定时间 |

电机自带编码器并不代表这些参数对所有型号都相同。铭牌线数可能按单相、双相或
输出轴标注，必须通过手转一圈计数和实际行驶距离校准。

## 7. 生成和编译

本机已经验证的工具：

- MSPM0 SDK：C:\ti\mspm0_sdk_2_01_00_03
- SysConfig：C:\ti\sysconfig_1.20.0
- Keil：C:\Keil_v5\UV4\UV4.exe

PowerShell：

~~~powershell
cd C:\Users\han_z\Desktop\TEST_TI\G3507-BASE
.\tools\run_sysconfig_keil.bat "$PWD" G3507-BASE.syscfg
.\tools\static_check.ps1
.\tools\build.cmd
~~~

也可以直接打开 keil/G3507-BASE.uvprojx。Keil 每次构建前会自动运行 SysConfig。
成功构建后生成：

~~~text
keil/Objects/G3507_BASE.axf
keil/Objects/G3507_BASE.hex
~~~

## 8. 首次上板顺序

1. 先断开电机主电源，核对 STBY=3.3 V、全部共地和 GPIO 电平。
2. 将车轮架空，烧录程序；上电默认未使能，方向脚和双路 PWM 都为 0。
3. 手转左右轮，观察 UART0/OLED 的左右速度符号是否与前进方向一致。
4. 从最左 CH1 到最右 CH8 逐个遮挡，确认 OLED 位图从 bit7 到 bit0 依次变化，且遮挡黑线时对应位为 1。
5. 检查 JY901S 页面是否显示 ONLINE，静止时姿态和加速度是否合理。
6. 接通电机电源，把基础速度先保持在最低档，再按 PB21 短时使能。
7. 若轮子方向错误，先核对接线，再修改输出符号；若速度符号错误，修改编码器符号。
8. 完成架空验证后再落地，依次校准轮径/编码器比例、速度 PI 和循迹 PD。

当前结论是 SysConfig、静态源码和 Keil 构建已经通过；电机方向、传感器极性、
JY901S 实际波特率以及控制参数仍必须以实车测试为准。
