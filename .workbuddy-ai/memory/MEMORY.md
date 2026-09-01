# Project memory — crossesnoughts (YDLidar X3 tic-tac-toe)

## Hardware (verified 2026-08-31)

- YDLidar X3 on **`/dev/ttyUSB0`**, **115200 baud** (NOT the SDK default 230400).
  Stable path: `/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0`
- Auto-streams on power-up; no SCAN command required.
- Measured: 12.1 Hz rotation, 331 pts/rev (1.09 deg/pt), ~4.0 kHz ranging, 100% checksum.
- Permissions OK: device is `root:uucp 660`, user `levzzz` is in `uucp`.

## Interpreter — IMPORTANT

**Use `/home/levzzz/miniconda3/bin/python` (3.12), not `/usr/bin/python3` (3.14).**
`YDLidar-SDK/build/python/_ydlidar.so` links `libpython3.12.so.1.0`; under 3.14
`ydlidar.CYdLidar()` segfaults (SIGSEGV). The conda interpreter has all deps
(numpy, fastapi, uvicorn, pygame, pytest, serial).

Run real mode as:
```
PYTHONPATH=<repo>/YDLidar-SDK/build/python /home/levzzz/miniconda3/bin/python ...
```

## Known gaps in real mode

1. **FIXED (2026-08-31):** `core/real.py` now applies the canonical tri_test
   option block (baud 115200, TYPE_TRIANGLE, YDLIDAR_TYPE_SERIAL, SampleRate 3,
   SingleChannel True, FixedResolution True) before `initialize()`.
2. `core/scan.py` constants don't match hardware:
   SCAN_POINT_COUNT 300 vs real 331; SCAN_FREQ 10.0 vs 12.1 Hz;
   SCAN_SAMPLE_RATE 3.0 kHz vs ~4.0 kHz.
3. SDK reports `scan.moduleNum == 0`, not `MODULE_NUM_X3` (6); `real.py` patches
   it in `doProcessSimple`.

## App architecture (2026-08-31)

Two-phase pygame app: **Settings screen** → **Game phase** (ESC returns to settings).

- `game/pipeline.py` — shared scan thread across both phases; control ops for
  source switch / board-size / yaw / calibrate / new_game.
- `game/settings_screen.py` — device status (live probe), source picker
  (Sim/Real/Replay), calibration radar (360° view + yaw slider + board-size
  slider + background recal + tracking check), team config (names + image upload).
- `game/render.py` — team names in status bar, win overlay (winner + logo +
  "N — play again"), F = fullscreen, resizable window.
- `game/devstatus.py` — stdlib serial probe (termios): ports + Hz/pts/checksums.
- `core/yaw.py` — `RotatedLidarSource`: yaw offset on angles + background.
- `core/settings.py` — `GameSettings` dataclass + JSON persistence.
- Game rules: overwrite opponent's cell = allowed + flips turn; own-cell tap =
  no-op (no move, no turn flip).
- Run: `/home/levzzz/miniconda3/bin/python -m game.run`
- 118 tests pass. Headless UI smoke test passes (SDL dummy driver).

## Protocol reference (decoding raw scans)

From `YDLidar-SDK`:
- angle  = `(raw >> 1) / 64.0` degrees  — `src/CYdLidar.cpp:659`
- range  = `raw / 4000.0` metres        — `src/CYdLidar.cpp:685` (triangle branch)
- checksum = XOR of four header u16s, then XOR of each node u16 —
  `src/YDlidarDriver.cpp:1472`
- Packet: `AA 55 | ct | count | firstAngle:u16 | lastAngle:u16 | cs:u16 | u16[count]`
- `ct & 0x01` marks the zero/start-of-revolution packet.

## Tools

- `tools/lidar_check.py` — stdlib-only hardware health check, works with any
  Python (useful precisely when the SWIG module is broken).
  `python3 tools/lidar_check.py /dev/ttyUSB0 115200 4`
- `tools/lidar_view.py` — pygame live viewer: polar radar + top-down map,
  distance-coloured points, fading trails, auto-zoom, HUD.
  Needs the conda interpreter (pygame is not in system python3.14):
  `/home/levzzz/miniconda3/bin/python tools/lidar_view.py /dev/ttyUSB0 115200`

## Gotchas

- Shell is **fish**: no POSIX `for ... do ... done`, and bare `--include=*.h`
  style globs fail. Use the Grep/Glob tools rather than shell grep/find.
- `stty` can hang on a continuously transmitting tty (it waits to drain);
  prefer Python `termios` with `TCSANOW`.

## Bundled SDK — what applies to the X3 (v1.2.20)

- `doc/Dataset.md` row **X3/X3 Pro = model 6, 115200, SampleRate 3K,
  0.10-8.0 m, 4-8 Hz, SingleChannel true**. The docs' 4-8 Hz / 3K disagree with
  the measured 12.1 Hz / ~4.0 kHz; baud and single-channel do match.
- Docs: "run the tri_test" for triangle units. X3-correct samples are only
  `examples/tri_test.cpp`, `tri_restart.cpp`, `tri_and_gs_test.cpp` and
  `python/examples/tri_test.py`. Everything else targets other models.
- The bundled matplotlib viewers (`plot_ydlidar_test.py` 230400/SR 9,
  `plot_tof_test.py` 512000/TYPE_TOF) are wrong for this device as shipped.
- **No ROS/ROS2 driver in the tree** — only `sensor_msgs::LaserScan` mentions
  in doc comments in `core/common/ydlidar_def.h`.
- Bindings: C++, C, Python (SWIG, `pip install .`), C# (SWIG, needs
  `cmake -DBUILD_CSHARP=ON`). Officially Ubuntu + Windows only.
- Stable-symlink recipe: `doc/howto/how_to_create_a_udev_rules.md` → `/dev/ydlidar`.
