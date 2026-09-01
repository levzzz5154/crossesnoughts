# HANDOFF — YDLidar X3 Emulator + 3D Scene + Crosses & Noughts

**For:** next agent picking up this repo.
**Date:** 2026-08-31
**Status:** 118/118 tests passing. Settings screen + calibration + device probe + teams + win overlay all live, headless-verified.
**Added this session (2026-08-31):**
- **Settings screen** (`game/settings_screen.py`): pre-game screen with live device status, source picker (Sim/Real/Replay), 360° radar calibration view with yaw slider + board-size slider, background recalibration, tracking check (live track ring + cell highlight + confidence), team name fields + image upload, Start Game button.
- **Pipeline** (`game/pipeline.py`): shared scan thread for both settings and game phases, with control ops for source switch, board-size/yaw/calibrate/new_game. Replaces the old `game/run.py` scan loop. Fixed a scan-then-move ordering bug that caused tracker re-acquisition to latch onto stale position after a pointer jump.
- **Device probe** (`game/devstatus.py`): stdlib-only serial probe (termios), lists candidate ports, sniffs 1.5 s for streaming + Hz + points/rev + checksums. No SWIG needed. Verified live: 12.05 Hz, 332 pts, 0 bad checksums.
- **Yaw wrapper** (`core/yaw.py`): `RotatedLidarSource` applies a runtime-adjustable yaw offset to scan angles in `doProcessSimple` AND shifts the background table by the same yaw in `calibrate_background`, so the 90° board window can align to any physical lidar orientation.
- **Settings persistence** (`core/settings.py`): `GameSettings` dataclass (board_size, yaw_deg, source kind/params, team X/O name+image, window size) with JSON load/save.
- **Real lidar fix** (`core/real.py`): now applies the canonical tri_test option block (baud 115200, TYPE_TRIANGLE, YDLIDAR_TYPE_SERIAL, SampleRate 3, SingleChannel True, FixedResolution True) before `initialize()`.
- **Own-cell no-op rule** (`core/game.py`): tapping your own cell is a no-op (no move, no turn flip). Overwriting the opponent's cell is still allowed and flips the turn.
- **Game renderer** (`game/render.py`): team names in status bar, win overlay (dimmed board + winner name + big logo image + "N — play again"), F key toggles fullscreen, dynamic canvas sizing for resizable window.
**Previous sessions:** record/replay (`core/recording.py`); lidar pointer mode; nearest-cluster acquisition fix.

---

## 1. What this project is

An emulator + visualization + game stack for a YDLidar X3 (non-Pro) lidar mounted at the corner of a square board:

- **`core/`** — web-free Python: a pure-Python lidar emulator (`SimLidarSource`) that duck-types the installed `ydlidar.CYdLidar` SWIG module, plus a real-lidar wrapper, a noise model, a tracker, and the game state machine.
- **`web/`** — FastAPI + WebSocket server serving a three.js 3D scene (testing/visualization tool, contains NO game code).
- **`game/`** — standalone 2D pygame app (crosses & noughts), imports ONLY `core/`, needs no web server.
- **`tests/`** — 96 tests across 10 files.

Full design rationale, coordinate conventions, noise model, tracking math, and acceptance criteria live in **`PLAN.md`** (read it before touching core math). `design_webgl.md` is the pre-plan design candidate (superseded).

---

## 2. How to run

```bash
cd /home/levzzz/Documents/EAI-X3-X3ProLidar/crossesnoughts
/home/levzzz/miniconda3/bin/python -m pytest tests/ -q   # 118 passed
/home/levzzz/miniconda3/bin/python -m game.run            # settings screen → game
/home/levzzz/miniconda3/bin/python -m game.run --lidar sim  # skip source picker, start with sim
# F = toggle fullscreen in game phase; ESC = back to settings; N = new game
```

CLI flags:

| Flag | App | Values |
|---|---|---|
| `--lidar` | both | `sim` (default) \| `real` \| `replay` |
| `--record FILE` | both | record every scan to a .npz (works with sim or real) |
| `--record-note` | both | free-text note stored in the recording's `meta` |
| `--replay-file FILE` | both | .npz to play back (requires `--lidar replay`) |
| `--replay-loop` / `--no-replay-loop` | both | loop forever (default) or play once and stop |
| `--replay-speed` | both | rate multiplier (default 1.0); `0` = free-run, no pacing |
| `--pointer` | game | `lidar` (default, commit from the tracked position) \| `mouse` (raw mouse) |
| `--board-size` | both | float, **1.0–2.0** (default 2.0) |
| `--image-x / --image-o` | game | PNG/JPG glyph files; omit → vector fallback |
| `--resolution` | game | 480/720/1080/1440/2160 (default 1080) |
| `--seed` | both | sim RNG seed |
| `--port-dev` | both | real lidar serial port (default `/dev/ttyUSB0`) |
| `--port / --host` | web | default 8000 / 127.0.0.1 |

---

## 3. Architecture (both apps: ONE process, TWO threads)

- **Main thread**: uvicorn (web) / pygame event loop (game).
- **Scan thread**: owns the LidarSource **exclusively** — `doProcessSimple(scan)` → `Tracker.process(scan)` → publish; drains a control queue (`reset`, `set_object`, `new_game`, `quit`) BETWEEN scans; shuts the source down in its own `finally` (single-writer shutdown — the main thread NEVER touches the source).
- **Thread boundary**: latest-wins store (SceneModel in web; pointer dict in game) + control queue.
- **Data flow**: `LidarSource → LaserScan → Tracker (filter → background subtraction → cluster → arc-centroid bias → alpha-beta) → Frame → WS JSON → three.js` (latest-wins rAF apply, zero trig in browser).
- **Coordinates**: L→B is identity swap (`x_B = y_L, y_B = x_L`); board origin = lidar-mount corner (front-right); θ=0 forward, CW-positive; B→W pure translation `x_W = x_B − S/2, z_W = y_B − S/2`. Pinned in `tests/test_transforms.py`.

## 4. Module map

```
core/config.py        SceneConfig (board_size ∈ [1.0,2.0], object_radius 0.10), NoiseConfig
core/scan.py          LaserPoint/LaserConfig/LaserScan (SWIG field names) + LidarProp enum (header-verified)
core/transforms.py    polar<->board<->world, pure numpy
core/raycast.py       vectorized raycast_ranges(angles, scene, object_center) — board edges, box walls, object circle
core/noise.py         apply_noise: jitter→range σ(a+b·d²)→dropout→ghost→blind spot→mixed pixels
core/lidar_source.py  LidarSource ABC (SWIG surface) + BackgroundTable (angle interpolation) + make_lidar_source factory
core/sim.py           SimLidarSource — SWIG twin, 10 Hz pacing inside doProcessSimple, raycast+noise, set_object_pose/reset/calibrate_background
core/real.py          RealLidarSource — lazy ydlidar import, canonical option block + FixedResolution=True, 30-scan median calibration
core/recording.py     RecordingLidarSource (wraps ANY source, dumps scans to .npz) + ReplayLidarSource (plays .npz back through the same surface) + record() + `python -m core.recording`
core/tracking.py      Tracker — BG subtraction (0.10 m), clustering (3-bin / 0.25 m), range-consistency (median+3σ), inverse-variance centroid, arc-centroid bias r_obj(1+cos β)/2, alpha-beta (α=0.6, β=0.3, gate 0.5 m, missed>10 → re-acquire)
core/game.py          GameState (3×3, overwrite opponent, own-cell no-op, per-move win, no draw) + TapDetector (0.3 s dwell)
core/yaw.py           RotatedLidarSource — yaw wrapper, rotates angles + background for board-window alignment
core/settings.py      GameSettings dataclass + JSON load/save (board_size, yaw, source, teams, window)
game/pipeline.py      Shared scan thread (settings→game), control ops, sim-mouse-hand integration
game/devstatus.py     Stdlib serial device probe (list ports, sniff Hz/pts/checksums)
game/settings_screen.py  Pre-game settings: device status, source picker, calibration radar, team config
web/frames.py         Frame.to_json, SceneModel (latest-wins + control queue), hello/scene messages
web/main.py           create_app, /ws endpoint, ScanLoop (thread + control handling)
web/run.py            CLI
web/static/           index.html, main.js (three.js scene + click/drag + grid overlay), style.css, vendor/ (three.module.js + three.core.js + OrbitControls.js, r185)
game/render.py        pygame renderer (grid, glyphs, hover, status, team names, win overlay, fullscreen)
game/run.py           CLI + two-phase loop (settings → game → ESC → settings)
tests/                test_transforms, test_raycast, test_noise, test_sim_api, test_swap, test_tracking, test_wire, test_game, test_e2e, test_recording, test_yaw, test_settings, test_pipeline
requirements.txt      fastapi, uvicorn[standard], numpy>=2,<3, pygame-ce, pytest
```

## 5. Web viz features (current state)

- Box walls + board + lidar marker + cyan point cloud (preallocated Float32Array, setDrawRange) + red track marker + green true-pose marker (sim only).
- **Click/drag on the board places the sim object** — raycast against invisible board plane, sends `{"type":"set_object","x","y"}` (board coords) over WS; server moves the object at the next scan boundary AND calls `Tracker.reset()` so it re-acquires (a jumped object exceeds the 0.5 m gate).
- **3×3 amber grid overlay + nearest-cell highlight + HUD "cell (cx, cy)"** — cell = `floor(x·3/S), floor(y·3/S)` clamped, updated per frame from the tracked output.
- Camera at `(1.0S, 0.9S, 1.0S)` looking at `(0, 0.1, 0)` (zoomed for clickability).
- WS: `hello` + `scene` on connect; frames only when seq changes; client controls `reset` (re-seed + re-calibrate) and `set_object`.
- `window.__viz = { boardPoint, sendObject, getClickPlane, getWs }` — debug hook used by automated browser verification; safe to keep.

## 6. Game (2D pygame)

- **Two-phase app**: Settings screen → Game phase. ESC returns to settings; the scan thread runs continuously across both phases.
- **Settings screen** (`game/settings_screen.py`):
  - Device status: lists serial ports, shows streaming/Hz/points/checksums via `game/devstatus.py` probe (stdlib termios, no SWIG).
  - Source picker: Sim / Real / Replay — each with live status. Selecting a source reconfigures the pipeline via a control op.
  - Calibration: 360° top-down radar view with the 90° board-quadrant overlay (yaw-aligned), board-size slider (1.0–2.0 m), yaw slider (0–360°), recalibrate background button, tracking check (live track ring + cell highlight + confidence readout).
  - Teams: X/O name text fields + image upload (tkinter file dialog, PNG/JPG).
  - Start Game button (or Enter key).
- **Game phase** (`game/render.py` + `game/run.py`):
  - Tap = pointer dwell 0.3 s in a cell. Turn alternates X/O. Overwrite opponent's cell allowed and flips the turn. **Own-cell tap = no-op** (no move, no turn flip). Win checked per move. No draw. `N` = New Game.
  - Team names shown in status bar ("Red's turn (X)" / "Blue wins!").
  - Win overlay: dimmed board, winner name + big team logo image, "N — play again".
  - F = toggle fullscreen (remembers monitor). Window is resizable; size persisted in settings.json.
  - `--pointer lidar` (default): moves from tracked position. `--pointer mouse`: raw mouse bypass.
  - Sim mode: mouse drives the simulated hand through the full raycast → noise → tracking pipeline.

## 7. Key decisions & deviations from PLAN.md

1. **Board size capped at 2.0 m** (user override of plan's 3.0). Background threshold 0.10 m = 5σ_max at S=2.0 far corner (σ_r(2.83)=0.020 m). `SceneConfig(board_size=3.0)` raises ValueError.
2. **Nearest-cluster selection** within the gate (plan's r_pred anchor could grab mixed-pixel clusters during fast jumps).
3. **`Tracker.reset()`** added (plan didn't have it) — required for click-drag object placement.
4. **three.core.js vendored** — r185 splits three.module.js; without it the browser 404s.
5. E2E game test uses continuous pointer motion (~0.3 m/s); instant cell jumps exceed the 0.5 m gate by design.
6. Angle grid: N=300, θ_i = −π + i·2π/299 inclusive; increment 2π/299 (SDK FOV/(count−1), CYdLidar.cpp:634-635).

## 8. Verification evidence (already done)

- `pytest`: **118 passed** (all previous + yaw wrapper angle rotation + background remap + end-to-end tracking through rotated window + settings JSON round-trip + pipeline: sim source comes up, background calibrates, pointer drives simulated hand, full tap flow with own-cell no-op, board-size reconfigure, yaw reconfigure + background remap).
- Headless UI smoke test (SDL dummy driver): sim source up at 10 Hz / 300 pts, tracking at (1.26, 0.76) conf 1.00, settings screen draws for 2 s then starts on Enter, game phase draws for 2 s then ESC returns to settings, 1 move committed while playing, win overlay renders with team names. ALL SMOKE CHECKS PASSED.
- Real device probe: 12.05 Hz, 332 pts/rev, 0 bad checksums on `/dev/ttyUSB0` at 115200 baud.
- Previous verifications still hold: transforms, raycast, noise, sim API/parity, swap surface, tracking accuracy, wire schema, game rules, e2e WS, record/replay round trip, dwell-clock regression, lidar-pointer-mode contract.

## 9. Gotchas

- **Headless Chrome fan issue**: browser smoke tests leave a headless Chrome whose GPU process pegs ~870% CPU (SwiftShader). Kill the whole tree after use: `pgrep -f "omp.browser.headless" | head -1 | xargs kill`. The `browser` tool's `close` with `kill:true` does NOT reliably kill it.
- **Real lidar**: `--lidar real` lazy-imports `ydlidar` (sim runs without it). Canonical option block includes `LidarPropFixedResolution=True` (without it the real SDK emits variable-size scans). Calibration = 30 empty-board scans, median per quantized angle bin, interpolated (real angles are jittered).
- Scan pacing: sim holds 10 Hz inside `doProcessSimple` (~100 ms blocks); real blocks up to 1 s — fine, dedicated thread. Tracker dt clamped [0.05, 0.5] s absorbs scanFreq differences.
- `web.main` mounts static at `/` — the `/ws` route MUST be registered before the mount (starlette route order); a regression here breaks all WS.
- **Replay at `--replay-speed 0` outruns the browser.** SceneModel is latest-wins, so a free-running replay (31 scans in ~30 ms) lets the WS client sample only a handful of frames. Use `speed 0` for headless/offline analysis; use a paced speed to actually watch it. `web/run.py` prints a note when you pick `0`.
- **Never call the wrapped source's `doProcessSimple` directly while a `RecordingLidarSource` is in play** — `rec.doProcessSimple()` already pulls one scan from the wrapped source and captures it. Calling both doubles the scan rate and records every other scan.
- Sim pacing lives in `SimLidarSource._scan_freq` (10 Hz). Tests that want many scans quickly set it to 500.0 rather than waiting in real time.
- **Mixed pixels split one object into two clusters, and acquisition used to pick the wrong one — FIXED.** This was the sharpest bug in the tracker; the reasoning is kept here because it is the trap to watch for on real data. With the object static at (1.0, 1.0), seed 5, background subtraction yields `n=3, r=2.05 m, pos=(1.42, 1.47)` — the object's *own* mixed-pixel points, pulled halfway toward the board behind it — and `n=3, r=1.34 m, pos=(0.99, 0.90)`, the clean object. **Equal size.** `_acquire` took the largest and broke ties by angle order, which is arbitrary, so it locked onto the fragment 0.70 m *behind* the object. From there it never self-corrected: the real object sits outside the 0.5 m gate and is rejected every scan, while the phantom holds `missed == 0` and `conf == 1.0`. It is position- and seed-dependent — (1.0, 1.0) and (0.333, 1.0) failed at seed 5, nothing failed at seeds 11 or 23, which is exactly why it survived testing. `_acquire` now takes the **nearest** cluster (the rule `_process` already used). Measured over 8 positions × 3 seeds: 3 broken cases fixed, none made worse, worst-case static error 70.4 cm → 1.9 cm. Real hardware will produce far more mixed pixels than the sim's 15%, so plan for this pattern rather than for the sim's numbers. Regression guards: `tests/test_tracking.py`.
- **Single-point clusters are still the tracker's blind spot.** Measured: with the object parked at (0.333, 0.333) and the track still sitting at (1.0, 1.0), the scan contains one 11-point cluster and one 6-point cluster (the real object, r≈0.38 m) plus three *single-point* outliers at r = 1.41–1.56 m — mixed pixels pulled halfway to the board behind the object. The object is 1.03 m from the track, so it is rejected by the 0.5 m gate; the single mixed pixel at (1.13, 1.08) is 0.15 m away and is accepted. `_process` explicitly allows 1-point clusters "within the prediction gate", so the tracker follows the phantom with `missed = 0` and `conf = 1.0` while the real object is right there. It never self-corrects: the phantom keeps it inside the gate. **Consequence in the app:** any jump larger than the gate must be followed by `Tracker.reset()`, or the track latches onto phantoms (the game does this on mouse jumps — `MOUSE_JUMP_RESET` in `game/run.py`; the web does it on click-drag). **With real hardware there is no reset to call**, so fixing the acceptance rule (#1 in §10) matters more than it looks.
- The dwell clock is scan time, not pointer sample time. `TapDetector` measures `t - _entered` against whatever the caller passes; feed it a pointer sample timestamp and a still pointer can never complete a dwell. `game/run.py` passes `time.monotonic()` per scan. Regression guards live in `tests/test_game.py`.
- **Do not debug the tracker with `SimLidarSource._scan_freq` raised to skip wall-clock time.** `Tracker._dt` clamps to [0.05, 0.5] s, so at 500 Hz the alpha-beta filter sees dt = 0.05 while only 2 ms actually elapsed — a 25× velocity exaggeration. It produces a phantom-latch that does **not** reproduce at the real 10 Hz, where the missed counter climbs, the track drops after 10 misses, and `_acquire` correctly picks the real object back up. Reproduce at 10 Hz. (Setting the speedup is still fine for *recording* fixtures, where nothing depends on filter dynamics.)
- Tests use `websockets.sync.client` (comes with uvicorn[standard]); deprecation warnings are harmless.

## 10. Suggested next steps (NOT done, no user request yet)

Ranked by risk retired, not by effort. The tracker is still unverified against
real hardware, so anything that makes real data easier to inspect outranks
anything that makes the demo prettier.

1. **Single-point clusters are accepted as measurements** (`core/tracking.py`, the
   `len(cr) == 1` branch in `_process`). When a jump leaves the track behind, a
   lone mixed-pixel outlier inside the 0.5 m gate keeps the phantom alive with
   `missed == 0` and full confidence — see §9. Fixing this means giving up the
   ability to track a genuinely 1-point object at long range, so it needs a
   deliberate call plus a re-run of the §8 accuracy numbers.
2. **Cluster-width validation** in `core/tracking.py` — the natural companion to
   #1: accept only clusters whose angular width is consistent with a 0.10 m
   object at the measured range (~2.9° at 2 m). Board/wall returns and
   mixed-pixel fragments subtend differently.
3. **Hardware bring-up, driven through recordings** (`--lidar real --record`).
   Capture empty board, slow finger sweep, fast taps and grazing corners, then
   replay until the §8 numbers hold for real data. The acquisition fix in §9 was
   found in the sim; expect real mixed-pixel rates to be higher and to expose
   more of the same class of failure.
4. `TrackResult.conf` is computed and on the wire but read by nobody. The game's
   pointer ring now fades with it; the web viz still ignores it.
5. Keyboard-driven pointer in the game for sim-mode play/testing.
6. Shared state between web viz and game (one sim instance, two consumers).
   Deliberately LAST: two processes with separate sims is a *feature* while the
   tracker is still being shaken out — coupling them means one tracker bug takes
   down both consumers at once.
7. README.md (plan §14 step 12 — only if requested; user has NOT asked).
8. Remove `window.__viz` debug hook if it ever becomes a liability (currently harmless and useful for browser automation).

## 11. Record & replay (.npz)

`core/recording.py` lets you capture raw scans from **any** source and play them
back later with no device attached. Both new classes duck-type the LidarSource
surface, so nothing in `web/` or `game/` knows the difference.

- `RecordingLidarSource` wraps a source and captures every scan
  `doProcessSimple` returns. It delegates everything else via `__getattr__`, so
  sim-only extras (`object_pose`, `set_object_pose`) keep working and the viz
  still shows its true-pose marker while recording. Flushes every 60 scans, on
  `close()`, and at process exit (`atexit`) — ScanLoop calls `close()` in its
  `finally`, alongside `turnOff`/`disconnecting`.
- `ReplayLidarSource` plays a file back, paced by the recorded stamps so
  playback runs at the rate the data was actually captured. `loop`, `speed`
  (`0` = free-run) and `seek()` are supported; the web `reset` control rewinds
  to scan 0 and calls `Tracker.reset()`.
- Non-looping playback sets `exhausted` when it finishes; ScanLoop breaks out
  instead of spinning.

File layout: `ranges (T,N) float32`, `angles (T,N) float32` (collapsed to `(N,)`
when every scan shares a grid), `stamps (T,) int64` ns, optional `bg_angles` /
`bg_ranges` float64, and `meta` (JSON string: kind, note, created, n_scans,
point_count, duration_s). float32 is ~4 orders of magnitude below the noise
floor, so the round trip is lossless in every way that matters.

**The background must be captured before the first recorded scan.** The
recorder stores whatever `calibrate_background()` returned; `record()` and
ScanLoop both call it before the loop starts. Without it, replay falls back to a
per-angle median over the first 30 scans and warns — that is wrong the moment
the object is already in frame at t=0 (measured: median tracking error went from
<5 cm to 53 cm).

## 12. Files NOT to touch casually

- `core/tracking.py` constants (BG_THRESHOLD, GATE, ALPHA/BETA, MISSED_LIMIT) are acceptance-coupled — changing them invalidates the verified accuracy numbers in §8.
- `core/scan.py` grid constants (2π/299) are SDK-coupled.
- `core/real.py` canonical option block is hardware-coupled (verified against tri_test.cpp).