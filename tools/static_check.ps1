[CmdletBinding()]
param()

# 本脚本用于构建前的工程边界检查：
# 1. 先根据 .syscfg 重新生成代码，避免检查过期文件；
# 2. 验证保留/释放引脚、双轮闭环、循迹位序以及无关外设已彻底解除；
# 3. 验证 APP/BSP/Algorithm 分层和 Keil 源文件清单；
# 4. 阻止被明确排除的功能说明重新混入基础工程。
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$syscfgPath = Join-Path $projectRoot 'G3507-BASE.syscfg'
$generatedSourcePath = Join-Path $projectRoot 'ti_msp_dl_config.c'
$generatedHeaderPath = Join-Path $projectRoot 'ti_msp_dl_config.h'
$keilProjectPath = Join-Path $projectRoot 'keil\G3507-BASE.uvprojx'

Write-Host '[1/5] 重新生成并校验 SysConfig'
& (Join-Path $PSScriptRoot 'run_sysconfig_keil.bat') $projectRoot 'G3507-BASE.syscfg'
if ($LASTEXITCODE -ne 0) {
    throw "SysConfig 失败，退出码：$LASTEXITCODE"
}

$syscfgText = Get-Content -LiteralPath $syscfgPath -Raw
$generatedSource = Get-Content -LiteralPath $generatedSourcePath -Raw
$generatedHeader = Get-Content -LiteralPath $generatedHeaderPath -Raw
$keilProject = Get-Content -LiteralPath $keilProjectPath -Raw

Write-Host '[2/5] 检查保留引脚和无关外设移除边界'
foreach ($pin in @(
    'PA2', 'PA8', 'PA15',
    'PB10', 'PB11', 'PB13', 'PB18', 'PB19', 'PB25', 'PB26', 'PB27'
)) {
    $assignmentPattern = '\$assign\s*=\s*"' + [regex]::Escape($pin) + '"'
    if ($syscfgText -match $assignmentPattern) {
        throw "保留引脚被 SysConfig 分配：$pin"
    }
}

foreach ($adcFile in @('BSP\ADC.c', 'BSP\ADC.h')) {
    if (Test-Path -LiteralPath (Join-Path $projectRoot $adcFile)) {
        throw "ADC 驱动文件仍然存在：$adcFile"
    }
}

foreach ($removedActuatorFile in @(
    'BSP\Actuator.c', 'BSP\Actuator.h',
    'BSP\Servo.c', 'BSP\Servo.h',
    'BSP\Magnet.c', 'BSP\Magnet.h'
)) {
    if (Test-Path -LiteralPath (Join-Path $projectRoot $removedActuatorFile)) {
        throw "赛题循迹固件不应保留该执行器文件：$removedActuatorFile"
    }
}
foreach ($configuredText in @($syscfgText, $generatedSource, $generatedHeader, $keilProject)) {
    if (($configuredText -match 'ADC12_VOLTAGE') -or
        ($configuredText -match '/ti/driverlib/ADC12') -or
        ($configuredText -match '<FileName>ADC\.(c|h)</FileName>')) {
        throw '工程仍包含 ADC 外设配置或 ADC 驱动条目。'
    }
    if (($configuredText -match 'SERVO_PWM') -or
        ($configuredText -match 'MAGNET_PWM') -or
        ($configuredText -match 'MECHANISM_PWM') -or
        ($configuredText -match '<FileName>(Servo|Magnet)\.(c|h)</FileName>')) {
        throw '工程仍包含舵机、电磁铁或机构 PWM 配置/工程条目。'
    }
}

Write-Host '[3/5] 检查按键、串口、电机、编码器、循迹位序和比赛流程'
$requiredSyscfgFragments = @(
    'GPIO5.$name                                = "USER_KEY"',
    'GPIO5.associatedPins[0].pin.$assign        = "PB21"',
    'UART1.$name                    = "DEBUG_UART"',
    'UART1.targetBaudRate           = 115200',
    'UART1.enabledInterrupts        = ["RX"]',
    'UART1.interruptPriority        = "2"',
    'UART1.peripheral.$assign       = "UART0"',
    'UART1.peripheral.rxPin.$assign = "PA11"',
    'UART1.peripheral.txPin.$assign = "PA10"',
    'PWM1.peripheral.$assign         = "TIMA1"',
    'PWM1.peripheral.ccp0Pin.$assign = "PB2"',
    'PWM1.peripheral.ccp1Pin.$assign = "PB3"',
    'GPIO10.associatedPins[0].pin.$assign       = "PA26"',
    'GPIO10.associatedPins[1].pin.$assign       = "PA25"',
    'GPIO11.associatedPins[0].pin.$assign       = "PB24"',
    'GPIO11.associatedPins[1].pin.$assign       = "PB20"',
    'GPIO12.associatedPins[0].$name = "CH1"',
    'GPIO12.associatedPins[0].internalResistor = "NONE"',
    'GPIO12.associatedPins[0].pin.$assign = "PA0"',
    'GPIO12.associatedPins[1].internalResistor = "NONE"',
    'GPIO12.associatedPins[2].internalResistor = "NONE"',
    'GPIO12.associatedPins[3].internalResistor = "NONE"',
    'GPIO12.associatedPins[4].internalResistor = "NONE"',
    'GPIO12.associatedPins[5].internalResistor = "NONE"',
    'GPIO13.associatedPins[0].internalResistor = "NONE"',
    'GPIO13.associatedPins[1].$name = "CH8"',
    'GPIO13.associatedPins[1].internalResistor = "NONE"',
    'GPIO13.associatedPins[1].pin.$assign = "PB17"',
    'UART2.$name                    = "JY901S_UART"',
    'UART2.peripheral.$assign       = "UART2"',
    'UART2.peripheral.rxPin.$assign = "PA22"',
    'UART2.peripheral.txPin.$assign = "PA21"',
    'UART4.$name                    = "TELEMETRY_UART"',
    'UART4.targetBaudRate           = 115200',
    'UART4.direction                = "TX"',
    'UART4.peripheral.$assign       = "UART3"',
    'UART4.peripheral.txPin.$assign = "PB12"'
)
foreach ($fragment in $requiredSyscfgFragments) {
    if (-not $syscfgText.Contains($fragment)) {
        throw "缺少关键 SysConfig 配置：$fragment"
    }
}

foreach ($iomux in @(
    'TRACK_A_CH1_IOMUX',
    'TRACK_A_CH2_IOMUX',
    'TRACK_A_CH3_IOMUX',
    'TRACK_A_CH4_IOMUX',
    'TRACK_A_CH5_IOMUX',
    'TRACK_A_CH6_IOMUX',
    'TRACK_B_CH7_IOMUX',
    'TRACK_B_CH8_IOMUX'
)) {
    $basicInputPattern = 'DL_GPIO_initDigitalInput\(\s*' + [regex]::Escape($iomux) + '\s*\);'
    $featureInputPattern = 'DL_GPIO_initDigitalInputFeatures\(\s*' + [regex]::Escape($iomux)
    if (($generatedSource -notmatch $basicInputPattern) -or
        ($generatedSource -match $featureInputPattern)) {
        throw "循迹输入必须保持浮空数字输入：$iomux"
    }
}
$trackSource = Get-Content -LiteralPath (Join-Path $projectRoot 'BSP\TrackSensor.c') -Raw
$bspConfig = Get-Content -LiteralPath (Join-Path $projectRoot 'BSP\bsp_config.h') -Raw
$appConfig = Get-Content -LiteralPath (Join-Path $projectRoot 'APP\app_config.h') -Raw
$carSource = Get-Content -LiteralPath (Join-Path $projectRoot 'APP\Car_Control.c') -Raw
$lineTrackSource = Get-Content -LiteralPath (Join-Path $projectRoot 'Algorithm\LineTrack.c') -Raw
$raceSource = Get-Content -LiteralPath (Join-Path $projectRoot 'APP\Race_Control.c') -Raw
$telemetrySource = Get-Content -LiteralPath (Join-Path $projectRoot 'APP\Telemetry_Stream.c') -Raw
$serialSource = Get-Content -LiteralPath (Join-Path $projectRoot 'BSP\Serial.c') -Raw
$mainSource = Get-Content -LiteralPath (Join-Path $projectRoot 'APP\main.c') -Raw
if ($trackSource -notmatch 'TRACK_A_CH1_PIN\)\s*!=\s*0U\)\s*raw\s*\|=\s*0x80U') {
    throw '循迹位序错误：最左 CH1 必须映射到 bit7。'
}
if ($trackSource -notmatch 'TRACK_B_CH8_PIN\)\s*!=\s*0U\)\s*raw\s*\|=\s*0x01U') {
    throw '循迹位序错误：最右 CH8 必须映射到 bit0。'
}
if ($bspConfig -notmatch '#define\s+TRACK_ACTIVE_LOW_MASK\s+0xFFU') {
    throw '辅助板八路循迹应配置为低电平检测到黑线。'
}
if ($trackSource -notmatch
    '\{\s*-350\s*,\s*-250\s*,\s*-150\s*,\s*-50\s*,\s*50\s*,\s*150\s*,\s*250\s*,\s*350\s*\}') {
    throw '八路循迹权重必须完整保持源工程的 -350 到 +350 顺序。'
}
if (($bspConfig -notmatch
     '#define\s+TRACK_MARKER_WEAK_MIN_CONTIGUOUS\s+3U') -or
    ($bspConfig -notmatch
     '#define\s+TRACK_MARKER_STRONG_MIN_CONTIGUOUS\s+4U')) {
    throw 'A 点必须保持连续相邻 3 路弱标志、4 路强标志的集中配置。'
}
if (($carSource -match '#include\s+"(Servo|Magnet)\.h"') -or
    ($mainSource -match '#include\s+"(Servo|Magnet)\.h"')) {
    throw '比赛控制路径仍依赖已删除的舵机或电磁铁 BSP。'
}
if (($mainSource -match 'PIDTuner_Init\s*\(\s*\)\s*;') -or
    ($mainSource -match 'PIDTuner_Process\s*\(\s*\)\s*;')) {
    throw '比赛 main.c 不得初始化或运行 PID 调参协议。'
}
if (($syscfgText -match 'CAMERA_UART') -or
    ($generatedSource -match 'CAMERA_UART') -or
    ($generatedHeader -match 'CAMERA_UART') -or
    ($serialSource -match 'Serial_CameraWrite')) {
    throw 'UART3 已专用于无线 DAP 遥测，不应继续保留相机串口命名或接口。'
}
if ($generatedHeader -notmatch '#define\s+TELEMETRY_UART_INST\s+UART3') {
    throw '生成代码中 UART3 遥测实例缺失。'
}
if ($generatedHeader -match 'GPIO_TELEMETRY_UART_RX') {
    throw 'UART3 遥测必须保持 TX-only，PB13 不应生成 RX 配置。'
}
if ($generatedSource -notmatch
    'gTELEMETRY_UARTConfig\s*=\s*\{[\s\S]*?\.direction\s*=\s*DL_UART_MAIN_DIRECTION_TX') {
    throw 'UART3 遥测生成配置不是 TX-only。'
}
foreach ($telemetryFragment in @(
    'Serial_TelemetryWrite',
    'TELEMETRY_UART_INST_IRQHandler',
    'DL_UART_MAIN_INTERRUPT_TX',
    'TELEMETRY_FRAME_LENGTH',
    'CalculateCrc16',
    'Car_ControlGetWatchSnapshot',
    'TELEMETRY_EVENT_MANUAL_STOP'
)) {
    if ((-not $serialSource.Contains($telemetryFragment)) -and
        (-not $telemetrySource.Contains($telemetryFragment)) -and
        (-not $carSource.Contains($telemetryFragment))) {
        throw "UART3 遥测链路缺少关键实现：$telemetryFragment"
    }
}
if (($mainSource -notmatch 'Telemetry_StreamInit\s*\(\s*\)\s*;') -or
    ($mainSource -notmatch 'Telemetry_StreamProcess\s*\(\s*\)\s*;')) {
    throw '比赛 main.c 未初始化或调度只读 UART3 遥测。'
}
foreach ($raceFragment in @(
    'RACE_STATE_WAITING',
    'RACE_STATE_LEAVING_A',
    'RACE_STATE_RUNNING',
    'RACE_STATE_FINISHED',
    'RACE_MARKER_CLEAR_SAMPLES',
    'RACE_MARKER_STRONG_CONFIRM_SAMPLES',
    'RACE_FINISH_MIN_TIME_MS',
    'RACE_APPROACH_START_MS',
    'g_race_watch',
    'UpdateRaceWatch'
)) {
    if (-not $raceSource.Contains($raceFragment)) {
        throw "比赛流程缺少关键状态或门限：$raceFragment"
    }
}
$trackingControlSource = $carSource + "`n" + $lineTrackSource
foreach ($sourceTrackingParameter in @(
    'TRACK_FORWARD_SPEED_SCALE',
    'TRACK_ERROR_SLOWDOWN_MPS_PER_ERROR',
    'TRACK_STEER_KP_MPS',
    'TRACK_STEER_ENTRY_KP_MPS',
    'TRACK_STEER_CURVE_KP_MPS',
    'TRACK_STEER_ENTRY_ERROR_ABS',
    'TRACK_STEER_CURVE_ERROR_ABS',
    'TRACK_STEER_KD_MPS',
    'TRACK_CORRECTION_LIMIT_MPS',
    'TRACK_TARGET_LIMIT_MPS',
    'TRACK_RECOVERY_KICK_SPEED_MPS',
    'TRACK_RECOVERY_SPEED_MPS',
    'TRACK_RECOVERY_KICK_MS',
    'TRACK_LOST_PROTECT_MS',
    'TRACK_DIRECTION_DEADBAND'
)) {
    if (-not $appConfig.Contains($sourceTrackingParameter)) {
        throw "缺少移植后的源循迹参数：$sourceTrackingParameter"
    }
}
if ($appConfig -notmatch '#define\s+TRACK_FORWARD_SPEED_SCALE\s+5\.0f') {
    throw '当前正常行驶速度倍率必须保持为 5.0。'
}
if (($appConfig -notmatch '#define\s+SPEED_PI_KP\s+150\.0f') -or
    ($appConfig -notmatch '#define\s+SPEED_PI_KI\s+300\.0f')) {
    throw '双轮速度 PI 参数必须保持稳定版本的 Kp=150、Ki=300。'
}
foreach ($unchangedTrackingParameter in @(
    '#define\s+TRACK_STEER_KP_MPS\s+\(0\.035f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_STEER_ENTRY_KP_MPS\s+\(0\.100f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_STEER_CURVE_KP_MPS\s+\(0\.150f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_STEER_ENTRY_ERROR_ABS\s+100\.0f',
    '#define\s+TRACK_STEER_CURVE_ERROR_ABS\s+200\.0f',
    '#define\s+TRACK_STEER_KD_MPS\s+\(0\.020f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_CORRECTION_LIMIT_MPS\s+\(20\.0f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_RECOVERY_KICK_SPEED_MPS\s+\(24\.0f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_RECOVERY_SPEED_MPS\s+\(12\.0f\s*\*\s*TRACK_RPM_TO_MPS\)',
    '#define\s+TRACK_RECOVERY_KICK_MS\s+60U',
    '#define\s+TRACK_LOST_PROTECT_MS\s+400U'
)) {
    if ($appConfig -notmatch $unchangedTrackingParameter) {
        throw '循迹分段 P、D、差速上限或搜线速度配置不符合当前版本。'
    }
}
foreach ($controlFragment in @(
    'LineTrack_Calculate',
    'error_slowdown_mps_per_error',
    'line_was_visible',
    'last_direction',
    'recovery_kick_speed_mps',
    'recovery_kick_ms',
    'recovery_speed_mps',
    'lost_protect_ms',
    'g_line_controller.line_fault',
    'g_car_watch',
    'steering_correction_mps',
    'phase_bias_mps',
    'filtered_derivative'
)) {
    if (-not $trackingControlSource.Contains($controlFragment)) {
        throw "源循迹移植缺少关键实现：$controlFragment"
    }
}
if (($lineTrackSource -notmatch
     'direction\s*=\s*\(int8_t\)\(\s*-last_direction\s*\)') -or
    ($lineTrackSource -notmatch
     'lost_time_ms\s*<=\s*controller->recovery_kick_ms')) {
    throw '丢线恢复必须从第一拍起向最后真实转向的反方向快速打一把。'
}
foreach ($removedTrackingFragment in @(
    'LineTrackGuide',
    'LineTrack_GetRecoveryPhase',
    'LineTrack_MakeRecoveryError',
    'TRACK_ARC_FOLLOW',
    'TRACK_ARC_STRAIGHT',
    'TRACK_CURVE_ENTRY',
    'TRACK_WHEEL_TARGET_STEP_MPS',
    'TRACK_ERROR_FILTER_ALPHA'
)) {
    if (($trackingControlSource.Contains($removedTrackingFragment)) -or
        ($appConfig.Contains($removedTrackingFragment))) {
        throw "旧循迹逻辑仍残留在控制路径：$removedTrackingFragment"
    }
}
if ($carSource -match 'if\s*\(\s*track\.junction') {
    throw 'A 点横线不得再绕过源工程的八路平均权重循迹公式。'
}

Write-Host '[4/5] 检查分层和 Keil 源文件清单'
foreach ($legacyDirectory in @('Control', 'Display', 'Drivers')) {
    if (Test-Path -LiteralPath (Join-Path $projectRoot $legacyDirectory)) {
        throw "发现旧架构目录：$legacyDirectory"
    }
}

$reverseDependency = Get-ChildItem -LiteralPath (Join-Path $projectRoot 'BSP') -File |
    Select-String -Pattern '#include\s+"app_config\.h"' |
    Select-Object -First 1
if ($null -ne $reverseDependency) {
    throw "BSP 不应反向依赖 APP：$($reverseDependency.Path)"
}

$layerPaths = @(
    (Join-Path $projectRoot 'APP'),
    (Join-Path $projectRoot 'BSP'),
    (Join-Path $projectRoot 'Algorithm')
)
$sourceFiles = foreach ($layerPath in $layerPaths) {
    Get-ChildItem -LiteralPath $layerPath -Filter '*.c' -File
}
foreach ($sourceFile in $sourceFiles) {
    $projectRelativePath = $sourceFile.FullName.Substring($projectRoot.Length + 1)
    $keilRelativePath = '..\' + $projectRelativePath
    if (-not $keilProject.Contains($keilRelativePath)) {
        throw "源文件未加入 Keil 工程：$keilRelativePath"
    }
}

Write-Host '[5/5] 检查说明边界和工程关键文件'
$textFiles = @((Join-Path $projectRoot 'README.md'))
foreach ($layerPath in $layerPaths) {
    $textFiles += Get-ChildItem -LiteralPath $layerPath -File |
        Select-Object -ExpandProperty FullName
}

$disallowedPatterns = @(
    ('\b' + [regex]::Escape('Blue' + 'tooth') + '\b'),
    ('\b' + [regex]::Escape('B' + 'LE') + '\b'),
    [regex]::Escape([string][char]0x84DD + [char]0x7259),
    [regex]::Escape([string][char]0x9065 + [char]0x63A7)
)
foreach ($file in $textFiles) {
    $content = Get-Content -LiteralPath $file -Raw
    foreach ($pattern in $disallowedPatterns) {
        if ([regex]::IsMatch(
                $content,
                $pattern,
                [System.Text.RegularExpressions.RegexOptions]::IgnoreCase)) {
            throw "发现不需要的相关内容：$file"
        }
    }
}

foreach ($requiredFile in @(
    'README.md',
    'G3507-BASE.syscfg',
    'APP\Race_Control.c',
    'APP\Race_Control.h',
    'APP\Telemetry_Stream.c',
    'APP\Telemetry_Stream.h',
    'keil\G3507-BASE.uvprojx',
    'tools\build.cmd',
    'tools\run_control_logic_tests.ps1',
    'tools\run_telemetry_tests.ps1',
    'tools\tests\control_logic_test.c',
    'tools\tests\telemetry_stream_test.c',
    'tools\tests\test_track_telemetry_protocol.py',
    'tools\tests\stubs\ti_msp_dl_config.h',
    'tools\pid_tuner.py',
    'tools\run_pid_tuner.cmd',
    'tools\track_telemetry_protocol.py',
    'tools\track_monitor.py',
    'tools\run_track_monitor.cmd'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $projectRoot $requiredFile))) {
        throw "缺少工程文件：$requiredFile"
    }
}

Write-Host '[OK] SysConfig、引脚、分层和工程文件静态检查通过。'
