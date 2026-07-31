@echo off
setlocal

rem Launch the CH340 wheel-speed PI tuner with the current Python runtime.
where python >nul 2>nul
if errorlevel 1 (
  echo [ERROR] Python was not found in PATH.
  exit /b 1
)

python "%~dp0pid_tuner.py" %*
exit /b %ERRORLEVEL%
