# F407-Balance 平衡与循迹串口遥测

## 接线

使用 `3.3 V TTL` USB 转串口模块：

- STM32 `PB10 / USART3_TX` -> USB-TTL `RX`；
- STM32 `GND` -> USB-TTL `GND`；
- 当前为单向遥测，USB-TTL `TX` 不需要接 `PB11`；
- 不要接传统正负电平 RS-232，也不要把 USB-TTL 的 `5 V` 接到 STM32。

串口固定为 `115200 8N1`。USART3 只发送遥测，不接收电脑命令，电脑端无法通过
该脚本改变 PID、状态机或电机目标。

## 安装与运行

推荐直接启动图形界面：

```powershell
py .\PC_Tools\balance_telemetry.py
```

不带参数运行时会自动弹出 UI，也可以双击 `PC_Tools\启动遥测界面.bat`。界面中：

- 在下拉框选择 `WCH-Link SERIAL` 对应的 COM 口；
- 点击“连接并开始记录”；
- “平衡监控”页实时查看状态、位置、速度、误差、视觉、请求/ACK、PA0 轨迹
  READY/运行/评分状态和两张趋势曲线；
- “循迹监控”页解析独立 `TRK` 帧，显示 C1~C8、加权误差、左右轮目标/反馈、
  PWM、编码器故障和终点状态；下位机尚未加入 `TRK` 发送时该页保持等待；
- “参数与诊断”页显示 Kp/Ki/Kd、轨迹独立倍率、评分阈值、通信计数和自动建议；
- 点击“停止并生成报告”后自动关闭平衡/循迹两个 CSV，并生成平衡 `_summary.txt`。

如果同一块 WCH-Link 出现两个 COM 口，可以逐个尝试。选错的端口会显示“已连接但无
遥测”，不会向 STM32 下发任何命令。

命令行模式仍然保留。注意 `--port COM24` 不能单独作为 PowerShell 命令执行，必须
跟在 Python 和脚本路径后面：

```powershell
py -m pip install pyserial
py .\PC_Tools\balance_telemetry.py --list
py .\PC_Tools\balance_telemetry.py --port COM6
```

默认持续采集到 `Ctrl+C`，并在当前目录生成：

- `balance_telemetry_时间.csv`：可直接用 Excel 打开；
- `balance_telemetry_时间_track.csv`：循迹独立记录，不改变原平衡 CSV 字段；
- `balance_telemetry_时间_summary.txt`：本次数据摘要和保守调参建议。

固定采集 60 秒并指定文件：

```powershell
py .\PC_Tools\balance_telemetry.py --port COM6 --duration 60 `
  --output .\run_left_40mm.csv
```

重新分析已有 CSV：

```powershell
py .\PC_Tools\balance_telemetry.py --analyze .\run_left_40mm.csv
```

## 建议采集方法

1. 先准备物理断电，确认真实水平零点、摄像头坐标正负和电机方向；
2. 正常上电并等状态进入 `HOLD_ORIGIN`；
3. 先用原有方法分别轻拨 `20~40 mm`，确认任意位置回 O、视觉等待恢复和急停未退化；
4. 等界面显示 `PA0 轨迹 READY` 后按下 PA0，记录到 `HOLD_NEGATIVE`；首次上板应先
   临时用 `±20 mm` 目标确认方向，再恢复完整 `±50 mm`；
5. 报告会自动给出 PA0 接受到最终稳定的时间、+50 折返点、正峰、负谷、最终稳定窗
   最大误差和 PASS/FAIL；完整目标建议连续记录至少 10 次；
6. 如果脚本提示球多数时间远离目标、ACK 异常、硬限位或视觉异常，先停止 PID 调参。

脚本会优先检查方向、USART1 ACK、硬限位、视觉有效率和参数校验，再判断输出限速、
回零时间、过零振荡、稳定静差及速度估计噪声。建议来自遥测统计，不代表实物已经安全，
也不会自动改写 `g_balance_tuning`。

## 协议

MCU 每 `50 ms` 发送一帧 v2 `TEL`，每 `5 s` 发送一帧 v2 `CFG`，上电先发送
`HELLO`。当前脚本仍可解析旧 v1 串口帧和旧 CSV 日志。
所有帧都是 ASCII CSV，以 `\r\n` 结束。为避免固件引入浮点格式化，浮点量在线上
使用 `x100` 或 `x1000` 整数表示；接收脚本写 CSV 前会恢复为真实单位。

融合上位机另支持 `TRK v1` 行帧；该能力目前先在电脑端实现，只有下位机后续开始
发送 `TRK` 后循迹页面和 `_track.csv` 才会出现数据。`TEL/CFG` 的字段数量、CSV 和
分析逻辑均保持不变。循迹轮速按当前 F407 实现显示为“编码器计数/10 ms”，不是 m/s。

主要 CSV 字段包括：

- 状态与安全：`state`、`fault`、`start_reject_reason`、`tuning_error_mask`；
- 视觉：`position_mm`、`velocity_mm_s`、`last_valid_age_ms`、帧序号及错误计数；
- 控制：`error_mm`、P/I/D/回中分项、raw/limited rate、目标位置；
- 电机：最终请求、最近发送/ACK、owner/state/retry 及三类通信错误、硬限位计数；
- 轨迹：READY/已触发/已完成、计时、折返点、正峰、负谷、稳定窗最大误差和评分；
- 参数：Kp/Ki/Kd、回中和速度滤波、`max_slew`、`max_step`、轨迹倍率、目标、
  启动门控、评分阈值及软件/硬件边界。
