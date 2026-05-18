@echo off
setlocal
chcp 65001 >nul
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
"%USERPROFILE%\.platformio\penv\Scripts\pio.exe" device monitor -p COM6 -b 115200
