# Running on Windows

## Simulation and replay

1. Install 64-bit Python 3.11 or 3.12.
2. In PowerShell, from this repository, run:

   ```powershell
   py -3 -m venv .venv
   .venv\Scripts\Activate.ps1
   python -m pip install -r requirements.txt
   .\run.bat --lidar sim
   ```

The settings screen, separate game/preview windows, native file chooser,
simulation, and replay mode do not require the YDLidar native SDK.

## Real YDLidar X3

The checked-in `_ydlidar.so` is a Linux binary and cannot load on Windows.
Build the bundled `YDLidar-SDK` with 64-bit Visual Studio, CMake, Python, and
SWIG, ensuring that the Python extension and DLLs use the same architecture
and Python version as the virtual environment. Place the resulting
`ydlidar.py`, `_ydlidar.pyd`, and required DLLs in one of:

- `YDLidar-SDK\build\python`
- `YDLidar-SDK\build\python\Release`
- `YDLidar-SDK\build\Release`

The launcher searches those locations and passes them to game/preview child
processes. Connect the lidar, select **Real lidar**, and choose or enter its
`COM` port. The settings screen discovers Windows serial ports automatically.
