[CmdletBinding()]
param()

# 使用本机 MinGW GCC 编译并运行纯算法/状态机测试，不链接任何 MSPM0 寄存器代码。
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$gcc = (Get-Command gcc -ErrorAction Stop).Source
$testSource = Join-Path $projectRoot 'tools\tests\control_logic_test.c'
$testExe = Join-Path ([System.IO.Path]::GetTempPath()) (
    'g3507_control_logic_test_' + $PID + '.exe'
)

try {
    & $gcc `
        -std=c11 -Wall -Wextra -Werror `
        "-I$(Join-Path $projectRoot 'tools\tests\stubs')" `
        "-I$(Join-Path $projectRoot 'APP')" `
        "-I$(Join-Path $projectRoot 'Algorithm')" `
        "-I$(Join-Path $projectRoot 'BSP')" `
        $testSource `
        (Join-Path $projectRoot 'Algorithm\LineTrack.c') `
        (Join-Path $projectRoot 'APP\Race_Control.c') `
        (Join-Path $projectRoot 'BSP\TrackSensor.c') `
        -o $testExe
    if ($LASTEXITCODE -ne 0) {
        throw "主机侧控制测试编译失败，退出码：$LASTEXITCODE"
    }

    & $testExe
    if ($LASTEXITCODE -ne 0) {
        throw "主机侧控制测试失败，退出码：$LASTEXITCODE"
    }
}
finally {
    if (Test-Path -LiteralPath $testExe) {
        Remove-Item -LiteralPath $testExe -Force
    }
}
