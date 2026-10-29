# F407-Balance

> **版本：最终版（第三问稳定版）**
>
> **竞赛成绩：2026 年电子设计竞赛省一等奖。**
>
> 本目录为最终版工程归档，包含 STM32F407 固件、CubeMX/Keil 工程及电脑端遥测工具。

STM32F407VET6 车载平衡滚球控制器。机构保持水平上电，小球可位于任意有效视觉
位置；安全建零并使能后先持续把小球闭环拉回并强力稳定在 `0 mm`。此后 PA0 或评分
轨迹页短按 PE2，仅在 O 点稳定门控通过时触发一次 `0 -> +50 -> -50 mm` 轨迹，完成后
持续保持在 `-50 mm`。

## 上电安全行为

- 上电先初始化 HAL、GPIO、DMA、USART1/2/3、I2C1、接收解析器、遥测、OLED 和状态结构，
  此阶段不运动；OLED 上电等待和初始化由主循环状态机异步推进；
- PA0 已稳定松开、Camera 连续 3 帧识别到任意位置的小球且坐标新鲜后，自动进入
  一次安全启动流程；机构的真实水平由上电前人工保证；
- PA0 为内部下拉输入，高电平按下；上电后必须先稳定检测到松开；
- PE3、PE2 组成两键 OLED 菜单，均为内部上拉、低电平按下；按键只接对应 GPIO
  与 GND，不要外接 5 V。PE3 短按循环下一项/下一页，PE2 短按确认或执行动作，
  PE2 长按约 `750 ms` 进入菜单或返回。主页短按 PE2 仍保留原目标锁定快捷
  动作；上电自动进入 `HOLD_ORIGIN` 后，第一次短按 PE2 会经 `Motor_Command` 把电机轴
  回到已验证的水平 `0 pulse`，并等待位置查询确认；OLED 显示 `SET POS` 后，手动把球
  放到任意有效位置，再按一次 PE2，固件采集当前新鲜摄像头 X 坐标并持续锁定该目标。
  OLED 显示 `HOLD T`；以后再按 PE2 可重复“回水平→重新放球→重新记录”。目标只写入
  RAM，掉电后仍按默认流程锁 `0 mm`。PE2 在启动、PA0 轨迹进行中或故障状态下不强行
  插入普通闭环；完成的 `HOLD_NEGATIVE` 也可用 PE2 重新进入目标记录流程；
- 自动启动或 IDLE 中原有 PA0/Watch `START` 通过视觉有效、连续帧、参数和 USART1
  所有权门控后，才依次查询位置、明确失能、清零、再次验证零位、使能并进入闭环；
- 进入 `HOLD_ORIGIN` 后，只有连续 3 个正常视觉样本同时满足 `|x|<=10 mm`、
  `|v|<=20 mm/s`，PA0 或评分轨迹页短按 PE2 才被接受；两者共用同一安全门控，
  不满足时立即拒绝且不锁存，后续稳定后可重按；
- 轨迹按 `HOLD_ORIGIN` → `MOVE_POSITIVE(+50)` → `MOVE_NEGATIVE(-50)` →
  `HOLD_NEGATIVE` 执行；触发时不重新建零、不重新使能、不重新获取电机所有权，每次
  进入评分任务只接受一次；长按 PE2 退出后会取消本次记录并回到 0 点，重新满足稳定
  门控后可以再次进入评分任务；
- 机械最终发送硬限位固定为右端 `-340 pulse` 和左端 `+330 pulse`，不能被 Watch、PID
  或标定镜像修改；默认工作范围与最终发送硬限位一致。
- 小球误差和速度经 P/I/D 形成电机绝对位置目标，再经过目标跟随和两级限速执行。
  协议使用绝对位置以保证 ACK 重试幂等，并保留可靠的软/硬限位。
- `HOLD_ORIGIN` 与 PE2 记录目标共用强锁档：近端等效 `Kp=2.40, Ki=0.45, Kd=1.25`、
  目标跟随 `25 1/s`、`max_slew=4200 pulse/s`、`max_step=210 pulse/帧`，并用约
  `60 ms`、最多 `±6 mm` 的速度前视提前反向压回外扰；PA0 轨迹仍使用独立
  `P/I/D=1.20/1.00/0.50`、跟随 `0.90`、slew/step `1.00/1.00` 倍率，不继承强锁参数；
- 轨迹计时从 PA0/PE2 启动请求被安全门接受开始，到 `-50 mm` 连续满足 `|x-target|<=5 mm`、
  `|v|<=12 mm/s` 并稳定 `300 ms` 结束（参考工程使用的较宽速度门槛，便于 OLED 更早
  停止计时）。固件同时检查
  总时间 `<=5 s`、+50 折返点误差 `<=10 mm`、正向峰值 `<=+60 mm`、负向最低点
  `>=-60 mm`、最终稳定窗最大误差 `<=10 mm` 且全过程无故障；
- 连续 8 帧丢球或有效坐标超过 `vision_continue_ms` 未更新时，只进入
  `VISION_HOLD` 冻结新的位置修正；不会报视觉丢失故障，也不会自动停止。等待任意时长
  后收到下一帧有效坐标，会先重建速度历史，再自动恢复丢失前的 O 点、PE2 目标或轨迹状态。

软件硬限位依赖正确的人工水平零位，不能替代物理断电开关和现场监护。第一次
上板必须不放球、准备物理断电，并按执行指南从小脉冲和小目标逐级验证。

## 串口与协议

| 串口 | 引脚 | 波特率 | 用途 |
| --- | --- | ---: | --- |
| USART1 | PA9 TX / PA10 RX | 115200 8N1 | 张大头闭环步进电机 |
| USART2 | PA2 TX / PA3 RX | 115200 8N1 | MaixCam 毫米坐标 |
| USART3 | PB10 TX | 115200 8N1 | 向电脑单向发送闭环遥测；PB11 当前不接收 |

Camera 接受一行一个有符号整数毫米：`50\r\n`、`-50\r\n`、`0\r\n`；没有
真实检测结果时发送 `N\r\n`。STM32 对 X 固定执行 `-120..+120 mm` 物理检查。

两键接线均为“GPIO 一端、GND 一端”，固件已打开内部上拉，松开为高电平、按下为
低电平，不需要再接上拉电阻：`PE3=循环下一项/下一页`，
`PE2=短按确认/动作，长按约 750 ms 返回/进入菜单`。PE4、PE5 不参与菜单。

张大头商家完整协议保留在 `MDK-ARM/Algorithm/Emm_V5.c/.h`。闭环实际命令
使用新增 Async 串口入口，由 `MDK-ARM/APP/Motor_Command.c/.h` 统一完成 USART1
所有权、TX、ACK 30 ms 超时、一次重试、最新位置目标合并和最终机械硬门。

USART3 通过 TX DMA 每 `50 ms` 发送一帧 v2 ASCII CSV，含视觉位置/速度、
P/I/D/回中分项、限速前后输出、请求/发送/ACK 脉冲、状态、故障、轨迹端点和自动
评分；每 `5 s` 补发当前参数与评分阈值。电脑端脚本同时兼容旧 v1 日志，接线说明见
`PC_Tools/balance_telemetry.py`、`PC_Tools/README.md`。该链路只读，不接收控制命令。
电脑端脚本不带参数运行时默认打开 Tkinter 实时 UI，也可双击
`PC_Tools/启动遥测界面.bat`。

## OLED 显示与接线

默认使用 `SSD1306 128x64` 四针 I2C 的 0.96 英寸 OLED，7 位地址为 `0x3C`：

| OLED 引脚 | STM32F407VET6 | 说明 |
| --- | --- | --- |
| GND | GND | 必须与控制板共地 |
| VCC | 3.3V | 不要接 5V |
| SCL / SCK | PB6 | I2C1 SCL，400 kHz |
| SDA | PB7 | I2C1 SDA，地址 `0x3C` |

多数四针模块已带上拉；若模块没有，上拉电阻建议在 PB6、PB7 各接约 `4.7 kΩ` 到
`3.3 V`。OLED 近端建议并联 `0.1 uF` 去耦，SCL/SDA 尽量短并远离电机线。

屏幕按 128x64、8 个文本页设计，每行最多约 21 个 5x7 ASCII 字符。主页短按 PE3
在滚球实时、循迹实时、系统状态三页间循环；长按 PE2 进入主菜单。主菜单包含：

- `BALL CONTROL`：实时稳球、PE2/PA0 评分轨迹、PE2 目标记录；
- `LINE TRACK`：`FAST LAP 30`、`STRAIGHT 20`、`SLOW LAP 20` 三种模式；
- `SYSTEM STATUS`：Camera、电机、OLED、循迹和编码器摘要。

| 所在页面 | PE3 短按 | PE2 短按 | PE2 长按约 750 ms |
| --- | --- | --- | --- |
| 主页 | 循环下一张详情页 | 目标记录快捷动作 | 进入主菜单 |
| 普通菜单 | 循环下一项 | 确认/进入 | 返回上一级 |
| 评分轨迹页 | 无动作 | 与 PA0 共用安全门控启动 | 取消任务、回 0 点并返回滚球菜单 |
| 循迹准备页 | 无动作 | 启动所选模式 | 返回模式选择 |
| 任意点记录页 | 目标锁定后启动慢速循迹 | 回水平/记录目标 | 停止循迹、取消任务、回 0 点并返回滚球菜单 |
| 循迹运行中 | 切换详情 1/2 | 无动作 | 立即停车、退出任务并返回模式选择 |
| 循迹已完成 | 无动作 | 无动作 | 清除运行态并返回模式选择 |

菜单中短按 PE3 循环下一项，短按 PE2 确认，长按 PE2 返回。评分轨迹页短按 PE2 或
按 PA0 都可请求启动，并继续执行完全相同的安全门控。循迹准备页短按 PE2 启动；
循迹任务运行时 PE3 只切换两张详情页；长按 PE2 会在下一次控制周期清零轮速目标、
PWM 和运行状态，再返回模式菜单。任意点目标锁定后，短按 PE3 可在保持该目标稳球的
同时启动慢速前馈循迹，页面仍停留在任意点记录页；循迹完成后长按 PE2 仍返回滚球菜单，
若循迹尚未结束则先请求底盘停车。评分轨迹或任意点记录页长按 PE2 会取消对应任务，
清除本次评分占用并把滚球目标恢复为 `0 mm`；视觉丢失时仍先冻结，视觉恢复后再回 0。
滚球首页继续显示 `LOCK/LOST/CAM OFF`、控制
状态、轨迹计时、目标/误差以及 X/V；目标流程显示 `LEVEL`、`SET POS`、`HOLD T`。
Camera 失锁或离线时实时误差、X、V 改为横线，避免把旧坐标当成当前测量值。循迹
主页和运行主详情页顶部用大号 `T:xxx.x` 显示计时，八路电平和偏差压缩在下一行；
终点确认停车时 `final_time_ms` 锁存，OLED 随后保持最终时间不再继续增加。

OLED 每 `100 ms` 最多生成一帧，初始化命令和完整 1 KiB 显存由低优先级
`I2C1 TX DMA1_Stream7` 非阻塞发送，单帧 DMA 超时门限为 `80 ms`。NACK、BUSY、
DMA/HAL 错误或超时不再永久停用 OLED：驱动等待约 `250 ms` 后在主循环中 DeInit
I2C1，将 PB6/PB7 临时改为开漏 GPIO，输出 9 个 SCL 脉冲和 STOP，再重新初始化
I2C1 与 SSD1306；若仍失败会继续周期性重试。恢复过程不在 IRQ 中执行，也不改变
电机 ACK、滚球闭环、循迹状态或控制参数。

## 控制状态与 Watch

推荐观察：

- `g_balance_tuning`：唯一实时控制参数快照；F4-MIX 串级默认位置环为
  `1.05/0.008/0.08`，输出/I限幅 `50/10 cm/s`，速度环为
  `2.25/0.0003/2.10`，输出/I限幅 `150/50 degree`，速度滤波旧值权重 `0.30`；
- 轨迹门控默认 `±10 mm / 20 mm/s / 3帧`；`trajectory_kp_scale`、
  `trajectory_ki_scale`、`trajectory_kd_scale`、
  `trajectory_follow_gain_scale`、`trajectory_slew_scale`、`trajectory_step_scale`
  当前全部为 `1.00`，因此评分轨迹与静止/任意点稳球使用同一套基础参数；
- `g_balance_debug`：已精简为现场关键摘要，只保留启动、PA0/PE2、Camera、PID、
  电机通信摘要和故障字段；不要再展开几十项内部计数；
- `g_balance_command`：`1 START`、`2 STOP_RETURN_ZERO`、`3 RESET_RUN`、
  `4 CLEAR_TRACE`、`5 DISABLE`；
- `g_balance_trace`：128 点、不循环覆盖的前 5 秒轨迹；
- `g_motor_command_debug`：USART1 owner、请求 ID、ACK、重试、合并目标和硬限位；
- `g_motor_feedback`、`g_camera_debug`、`g_emm_debug`、`g_uart_dma_debug`；
- `g_telemetry_debug`：USART3 遥测帧数、字节数、忙状态、最近 HAL 状态和构建错误；
- `g_oled_debug`：OLED I2C 状态、帧数、错误/超时、恢复次数、恢复尝试和成功次数；
- `g_oled_display_debug`：当前菜单页、主页详情、`LOCK/LOST/CAM OFF`、显示位置/速度
  和提交计数；
- `g_motor_calibration`：只保留人工 Watch 标定，不再自动执行。

不要在电机使能或小球运动时设置断点、单步。Watch STOP/DISABLE 依赖 MCU、
调试链路和 USART1 正常，不能替代物理断电。

推荐分三个 Watch 窗口观察：

- 启动：`state`、`fault`、`start_reject_reason`、`start_ready`、`trajectory_ready`、
  `trajectory_ready_count`、PA0 三个 button 字段、`target_button_raw_level`、
  `target_button_debounced_level`、`target_button_press_count`、
  `target_button_rejected_count`、`camera_online`、`target_found`、
  `last_valid_age_ms`、`position_used_mm`；
- 闭环：`target_mm`、`position_used_mm`、`error_mm`、`velocity_filtered_mm_s`、
  `drive_rate_pulse_s`、`command_position_pulse`、`final_requested_pulse`、
  `last_transmitted_pulse`、`last_acked_pulse`；
- 安全：`tuning_error_mask`、`motor_owner`、`motor_command_state`、三个通信错误计数、
  `hard_limit_reject_count`。
- 评分：`trajectory_started`、`trajectory_completed`、`completion_time_ms`、
  `positive_turn_position_mm`、`positive_peak_position_mm`、`negative_min_position_mm`、
  `negative_stable_window_max_abs_error_mm`、`score_pass`。

## 构建与证据边界

在 `MDK-ARM` 目录执行：

```powershell
& 'C:\Keil_v5\UV4\UV4.exe' -r '.\F407-Balance.uvprojx' -o '.\build_output.log'
```

必须检查日志明确显示 `0 Error(s)`，并确认 AXF/HEX/MAP 时间戳刷新。构建通过只
证明编译和链接成功；实际 UART/ACK、电机与摄像头正负方向、水平零位、回 O 点效果、
轨迹时间/误差、视觉等待恢复和急停必须在可断电、低风险条件下分阶段上板验证。建议
先把球放在 O 点做 `±20 mm` 小目标方向验证，再开放完整 `±50 mm`，最后记录至少
10 次完整运行确认重复性。
