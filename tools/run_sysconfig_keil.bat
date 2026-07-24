@echo off
setlocal

rem Use the locally verified SysConfig and MSPM0 SDK versions.
rem Argument 1 is the project root; argument 2 is the .syscfg file name.
set "SYSCFG=C:\ti\sysconfig_1.20.0\sysconfig_cli.bat"
set "SDK_ROOT=C:\ti\mspm0_sdk_2_01_00_03"
set "PROJECT_DIR=%~1"
set "SYSCFG_FILE=%~2"

if not exist "%SYSCFG%" (
  echo [ERROR] SysConfig CLI not found: %SYSCFG%
  exit /b 1
)

if not exist "%SDK_ROOT%\.metadata\product.json" (
  echo [ERROR] MSPM0 SDK metadata not found: %SDK_ROOT%\.metadata\product.json
  exit /b 1
)

if not exist "%PROJECT_DIR%\%SYSCFG_FILE%" (
  echo [ERROR] SysConfig file not found: %PROJECT_DIR%\%SYSCFG_FILE%
  exit /b 1
)

echo [SysConfig] Generating %SYSCFG_FILE%
rem Generate ti_msp_dl_config.c/.h for the Keil compiler backend.
call "%SYSCFG%" -o "%PROJECT_DIR%" -s "%SDK_ROOT%\.metadata\product.json" --compiler keil "%PROJECT_DIR%\%SYSCFG_FILE%"
exit /b %ERRORLEVEL%
