@echo off
rem ProjectLab_System environment setup for Windows.
rem Creates .venv with 64-bit Python 3.12 - 3.14, installs requirements.txt and checks the result.
rem Usage: setup.bat [--recreate] [--download-wheels] [--no-pause]
rem Keep this file ASCII only (cmd.exe reads it with the system code page).
setlocal EnableExtensions
cd /d "%~dp0"
set "PYTHONUTF8=1"
set "PIP_DISABLE_PIP_VERSION_CHECK=1"
set "NO_PAUSE="
for %%A in (%*) do if /i "%%~A"=="--no-pause" set "NO_PAUSE=1"

rem Prefer the py launcher (newest supported version first), then python on PATH.
set "BASE_PY="
for %%V in (3.14 3.13 3.12) do (
    if not defined BASE_PY (
        py -%%V -c "import sys; sys.exit(0 if sys.maxsize > 2**32 else 1)" >nul 2>&1 && set "BASE_PY=py -%%V"
    )
)
if not defined BASE_PY (
    python -c "import sys; sys.exit(0 if (3, 12) <= sys.version_info[:2] <= (3, 14) and sys.maxsize > 2**32 else 1)" >nul 2>&1 && set "BASE_PY=python"
)
if not defined BASE_PY goto no_python

echo Using %BASE_PY%
%BASE_PY% scripts\setup_env.py %*
set "RESULT=%ERRORLEVEL%"
goto finish

:no_python
echo.
echo [ERROR] 64-bit Python 3.12 - 3.14 was not found.
echo Install Python 3.13 or 3.14 from https://www.python.org/downloads/windows/
echo with the "py launcher" option, then run setup.bat again.
set "RESULT=1"

:finish
echo.
if "%RESULT%"=="0" (echo Setup finished. Start the program with run.bat) else (echo Setup FAILED. Read the messages above.)
if not defined NO_PAUSE pause
exit /b %RESULT%
