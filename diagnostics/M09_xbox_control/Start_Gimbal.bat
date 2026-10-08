@echo off
cd /d "C:\Users\jhthe\Documents\Arduino\TMP\diagnostics\M09_xbox_control"

set PYTHONPATH=%CD%\.pio\python_deps

REM Signed output-axis revolutions per encoder revolution.
REM Yaw 1:1; pitch 1:15. Use a negative value if raw angle increases in reverse.
set YAW_ENCODER_RATIO=1
set PITCH_ENCODER_RATIO=0.06666667

"%USERPROFILE%\.platformio\penv\Scripts\python.exe" xbox_control.py ^
	--port COM9 ^
	--speed-scale 1.0 ^
	--latitude 43.9425 ^
	--longitude -86.0394 ^
	--elevation 0 ^
	--yaw-encoder-ratio %YAW_ENCODER_RATIO% ^
	--pitch-encoder-ratio %PITCH_ENCODER_RATIO% ^
	--north-reference true

pause
