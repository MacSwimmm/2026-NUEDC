[CmdletBinding()]
param()

# Compile and run the production UART3 telemetry encoder with host-side stubs.
$ErrorActionPreference = 'Stop'
$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$gcc = (Get-Command gcc -ErrorAction Stop).Source
$testSource = Join-Path $projectRoot 'tools\tests\telemetry_stream_test.c'
$testExe = Join-Path ([System.IO.Path]::GetTempPath()) (
    'g3507_telemetry_stream_test_' + $PID + '.exe'
)

try {
    & $gcc `
        -std=c11 -Wall -Wextra -Werror `
        "-I$(Join-Path $projectRoot 'APP')" `
        "-I$(Join-Path $projectRoot 'BSP')" `
        $testSource `
        (Join-Path $projectRoot 'APP\Telemetry_Stream.c') `
        -o $testExe
    if ($LASTEXITCODE -ne 0) {
        throw "UART3 telemetry host test compile failed: $LASTEXITCODE"
    }

    & $testExe
    if ($LASTEXITCODE -ne 0) {
        throw "UART3 telemetry host test failed: $LASTEXITCODE"
    }
}
finally {
    if (Test-Path -LiteralPath $testExe) {
        Remove-Item -LiteralPath $testExe -Force
    }
}

# Validate the independent Python stream parser, resynchronization and packet gaps.
& python (Join-Path $projectRoot 'tools\tests\test_track_telemetry_protocol.py')
if ($LASTEXITCODE -ne 0) {
    throw "Python telemetry protocol test failed: $LASTEXITCODE"
}
