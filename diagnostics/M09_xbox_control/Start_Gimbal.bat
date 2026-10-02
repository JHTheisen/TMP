@echo off
cd /d "C:\Users\jhthe\Documents\Arduino\TMP\diagnostics\M09_xbox_control"

set PYTHONPATH=%CD%\.pio\python_deps

"%USERPROFILE%\.platformio\penv\Scripts\python.exe" xbox_control.py ^
	--port COM9 ^
	--speed-scale 1.0 ^
	--latitude 43.9425 ^
	--longitude -86.0394 ^
	--elevation 0

pause
