@echo off
setlocal

cd /d "%~dp0"

set "SCRIPT=v04_Add Key Frames.py"

where python >nul 2>nul
if %errorlevel%==0 (
    C:\Users\jhthe\AppData\Local\Programs\Python\Python36-32\python.exe "%SCRIPT%"
    goto done
)

py -3 --version >nul 2>nul
if %errorlevel%==0 (
    C:\Users\jhthe\AppData\Local\Programs\Python\Python36-32\pyton.exe "%SCRIPT%"
    goto done
)

echo Python was not found from this shortcut.
echo.
echo If this runs in VS Code, copy the Python interpreter path from VS Code:
echo   Ctrl+Shift+P ^> Python: Select Interpreter
echo.
echo Then edit this .bat and replace the next line with:
echo   "C:\path\to\python.exe" "%SCRIPT%"
echo.

:done
pause
