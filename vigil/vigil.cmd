@echo off
where py >nul 2>&1
if %errorlevel% equ 0 (
  py -3 "%~dp0master_launcher\vigil_launcher.py" %*
) else (
  python "%~dp0master_launcher\vigil_launcher.py" %*
)
