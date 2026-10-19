@echo off
rem Mini BD-1 remote: double-click this file.
"%APPDATA%\uv\tools\platformio\Scripts\python.exe" "%~dp0pult.py" %*
if errorlevel 1 pause
