@echo off
setlocal

rem Launch the read-only UART3 wireless-DAP telemetry monitor.
where python >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Python was not found in PATH.
  exit /b 1
)

python "%~dp0track_monitor.py" %*
exit /b %ERRORLEVEL%
