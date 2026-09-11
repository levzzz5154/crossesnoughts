@echo off
setlocal
cd /d "%~dp0"
py -3 -m pip install -r requirements-build.txt
py -3 packaging\build_portable.py --clean --with-lidar
