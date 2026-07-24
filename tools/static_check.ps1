[CmdletBinding()]
param()

$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$syscfgPath = Join-Path $projectRoot 'G3507-BASE.syscfg'
$generatedHeaderPath = Join-Path $projectRoot 'ti_msp_dl_config.h'
$keilProjectPath = Join-Path $projectRoot 'keil\G3507-BASE.uvprojx'

Write-Host '[1/5] 重新生成并校验 SysConfig'
& (Join-Path $PSScriptRoot 'run_sysconfig_keil.bat') $projectRoot 'G3507-BASE.syscfg'
if ($LASTEXITCODE -ne 0) {
    throw "SysConfig 失败，退出码：$LASTEXITCODE"
}

$syscfgText = Get-Content -LiteralPath $syscfgPath -Raw
$generatedHeader = Get-Content -LiteralPath $generatedHeaderPath -Raw
$keilProject = Get-Content -LiteralPath $keilProjectPath -Raw

Write-Host '[2/5] 检查保留引脚没有被分配'
foreach ($pin in @('PA8', 'PB18', 'PB19', 'PB25')) {
    $assignmentPattern = '\$assign\s*=\s*"' + [regex]::Escape($pin) + '"'
    if ($syscfgText -match $assignmentPattern) {
        throw "保留引脚被 SysConfig 分配：$pin"
    }
}

Write-Host '[3/5] 检查双电机 PWM、双编码器和舵机时基'
$requiredSyscfgFragments = @(
    'PWM1.peripheral.$assign         = "TIMA1"',
    'PWM1.peripheral.ccp0Pin.$assign = "PB2"',
    'PWM1.peripheral.ccp1Pin.$assign = "PB3"',
    'GPIO10.associatedPins[0].pin.$assign       = "PA26"',
    'GPIO10.associatedPins[1].pin.$assign       = "PA25"',
    'GPIO11.associatedPins[0].pin.$assign       = "PB24"',
    'GPIO11.associatedPins[1].pin.$assign       = "PB20"'
)
foreach ($fragment in $requiredSyscfgFragments) {
    if (-not $syscfgText.Contains($fragment)) {
        throw "缺少关键 SysConfig 配置：$fragment"
    }
}
if ($generatedHeader -notmatch '#define\s+SERVO_PWM_INST_CLK_FREQ\s+1000000') {
    throw '舵机定时器不是 1 MHz，脉宽微秒值将不正确。'
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
    'keil\G3507-BASE.uvprojx',
    'tools\build.cmd'
)) {
    if (-not (Test-Path -LiteralPath (Join-Path $projectRoot $requiredFile))) {
        throw "缺少工程文件：$requiredFile"
    }
}

Write-Host '[OK] SysConfig、引脚、分层和工程文件静态检查通过。'
