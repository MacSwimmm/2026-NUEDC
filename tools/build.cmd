@echo off
setlocal

rem Rebuild with the verified local Keil installation and keep artifacts in the project.
rem Check the UV4 result, build log, AXF and HEX instead of trusting process exit alone.
set "UV4=C:\Keil_v5\UV4\UV4.exe"
set "PROJECT_ROOT=%~dp0.."
set "PROJECT=%PROJECT_ROOT%\keil\G3507-BASE.uvprojx"
set "LOG=%PROJECT_ROOT%\keil\build.log"
set "AXF=%PROJECT_ROOT%\keil\Objects\G3507_BASE.axf"
set "HEX=%PROJECT_ROOT%\keil\Objects\G3507_BASE.hex"

if not exist "%UV4%" (
  echo [ERROR] Keil uVision not found: %UV4%
  exit /b 1
)

if not exist "%PROJECT%" (
  echo [ERROR] Keil project not found: %PROJECT%
  exit /b 1
)

pushd "%PROJECT_ROOT%\keil"
rem -r performs a full rebuild; the Keil pre-build step regenerates SysConfig output.
"%UV4%" -r "G3507-BASE.uvprojx" -o "build.log"
set "UV4_RESULT=%ERRORLEVEL%"
popd

if exist "%LOG%" type "%LOG%"
if not "%UV4_RESULT%"=="0" exit /b %UV4_RESULT%

rem Treat the Keil log and output artifacts as the actual build evidence.
findstr /C:"0 Error(s)" "%LOG%" >nul
if errorlevel 1 (
  echo [ERROR] Keil log does not contain a successful build result.
  exit /b 1
)

if not exist "%AXF%" (
  echo [ERROR] AXF was not generated: %AXF%
  exit /b 1
)

if not exist "%HEX%" (
  echo [ERROR] HEX was not generated: %HEX%
  exit /b 1
)

echo [OK] Keil build completed and AXF/HEX files were generated.
exit /b 0
