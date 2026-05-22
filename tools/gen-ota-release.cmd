@echo off
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0gen-ota-release.ps1" %*
