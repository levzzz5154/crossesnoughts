@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" run.py %*
    exit /b
)
where py >nul 2>nul
if %errorlevel%==0 (
    py -3 run.py %*
) else (
    python run.py %*
)
