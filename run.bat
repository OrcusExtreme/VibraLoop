@echo off
rem Start ProjectLab_System with the project virtual environment (.venv created by setup.bat).
rem Keep this file ASCII only (cmd.exe reads it with the system code page).
setlocal EnableExtensions
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "VENV_PY=.venv\Scripts\python.exe"

if not exist "%VENV_PY%" goto need_setup
"%VENV_PY%" -c "import flask_socketio, sklearn, serial, watchdog" >nul 2>&1
if errorlevel 1 goto need_setup

echo Starting ProjectLab_System. Open http://localhost:5000 in a web browser.
echo Press Ctrl+C to stop the server.
"%VENV_PY%" cnc_app.py
set "RESULT=%ERRORLEVEL%"
if not "%RESULT%"=="0" pause
exit /b %RESULT%

:need_setup
echo [ERROR] The virtual environment .venv is missing or was copied from another PC.
echo Run setup.bat first.
pause
exit /b 1
