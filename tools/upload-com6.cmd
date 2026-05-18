@echo off
setlocal
chcp 65001 >nul
set PYTHONUTF8=1
set PYTHONIOENCODING=utf-8
"%USERPROFILE%\.platformio\penv\Scripts\pio.exe" run -e esp32s3_n16r16v -t upload --upload-port COM6
