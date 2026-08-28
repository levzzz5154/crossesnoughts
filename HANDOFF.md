# HANDOFF — YDLidar X3 Emulator + 3D Scene + Crosses & Noughts

**For:** next agent picking up this repo.
**Date:** 2026-08-29
**Status:** PLAN.md fully implemented, 96/96 tests passing, features verified live.
**Added this session:** record/replay (`core/recording.py`) — capture raw scans from
sim *or* real hardware to .npz and play them back with no device attached (§11).
**Fixed this session:** (a) the game now plays from the lidar (`--pointer lidar`,
default) instead of the mouse, and the dwell clock runs off scan time so a
held-still pointer registers a tap (§6, §9); (b) `_acquire` picks the nearest
cluster instead of the largest, which was locking onto a mixed-pixel fragment up
to 70 cm behind a static object while reporting full confidence (§9). One sharp
tracker issue remains — single-point cluster acceptance, §10 #1.

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
python -m pytest tests/ -q          # 96 passed
python -m web.run --lidar sim       # 3D viz at http://127.0.0.1:8000
python -m game.run --lidar sim      # 2D pygame window (no web server needed)
python -m game.run --lidar sim --pointer mouse   # tracker out of the loop

# record 20 s of real hardware, then replay it with no device attached
python -m core.recording --lidar real --out session.npz --seconds 20 --note "finger sweep"
python -m web.run --lidar replay --replay-file session.npz
# ...or record whatever the server is currently showing (sim or real)
python -m web.run --lidar real --record session.npz
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
core/game.py          GameState (3×3, overwrite, per-move win, no draw) + TapDetector (0.3 s dwell)
web/frames.py         Frame.to_json, SceneModel (latest-wins + control queue), hello/scene messages
web/main.py           create_app, /ws endpoint, ScanLoop (thread + control handling)
web/run.py            CLI
web/static/           index.html, main.js (three.js scene + click/drag + grid overlay), style.css, vendor/ (three.module.js + three.core.js + OrbitControls.js, r185)
game/state.py         re-exports core/game
game/render.py        pygame renderer (grid, glyphs, hover, status)
game/run.py           CLI, two-thread loop
tests/                test_transforms, test_raycast, test_noise, test_sim_api, test_swap, test_tracking, test_wire, test_game, test_e2e, test_recording
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

- Tap = pointer dwell 0.3 s in a cell (TapDetector, no auto-repeat, re-fire on re-entry). Turn alternates X/O per tap. Overwrite ANY cell allowed. Win checked per move. No draw state. `N` = New Game (applies at next scan boundary via control queue).
- **`--pointer lidar` (default)**: moves are committed from the TRACKED position. In sim mode the mouse acts as the hand — it moves the simulated object, and the tap still comes out of the full raycast → noise → tracking pipeline, so sim exercises the same path a real finger would. With real hardware or a replay the mouse is ignored entirely. The hover highlight and the new amber pointer ring both follow the tracker, lag and all, not the raw mouse.
- **`--pointer mouse`**: legacy. The tap position is the raw mouse position, bypassing the tracker. Useful for testing game rules in isolation.
- **Known limitation**: the web viz and game are SEPARATE processes with separate sim instances — no shared state. Click-dragging in the browser does not drive the game.

## 7. Key decisions & deviations from PLAN.md

1. **Board size capped at 2.0 m** (user override of plan's 3.0). Background threshold 0.10 m = 5σ_max at S=2.0 far corner (σ_r(2.83)=0.020 m). `SceneConfig(board_size=3.0)` raises ValueError.
2. **Nearest-cluster selection** within the gate (plan's r_pred anchor could grab mixed-pixel clusters during fast jumps).
3. **`Tracker.reset()`** added (plan didn't have it) — required for click-drag object placement.
4. **three.core.js vendored** — r185 splits three.module.js; without it the browser 404s.
5. E2E game test uses continuous pointer motion (~0.3 m/s); instant cell jumps exceed the 0.5 m gate by design.
6. Angle grid: N=300, θ_i = −π + i·2π/299 inclusive; increment 2π/299 (SDK FOV/(count−1), CYdLidar.cpp:634-635).

## 8. Verification evidence (already done)

- `pytest`: 96 passed (transforms pins incl. the θ=+π/4 sign-fix test; raycast exactness |err|<1e-9; noise stats; sim API/parity; swap surface vs installed ydlidar; tracking: median <2.5 cm continuous path, far-corner <3 cm, never lost >3 scans, BG FP <1%, mixed-pixel rejection; wire schema; game rules; e2e WS hello/scene/frames/reset/latest-wins + full headless game; **record/replay round trip incl. replay-driven tracking <5 cm**; dwell-clock regression guards; lidar-pointer-mode contract test).
- Real app, driven in-process with a scripted pointer (SDL dummy driver): a single mouse move followed by 3 s of holding **perfectly still** now commits exactly one move in both `--pointer lidar` (at (0.339, 0.333), 0.54 cm off the raw mouse point — i.e. it came through the tracker) and `--pointer mouse` (exactly (0.333, 0.333)). Before the fix, lidar mode committed two phantom moves at the old object position and mouse mode committed none.
- Record/replay live (headless, real server + WS): CLI recorder 31 scans/3.00 s with background; server `--record` stored 35 scans with the note; paced replay streamed monotonically with the tracker held on 22/22 frames; free-run emitted exactly 31 frames for 31 scans then stopped; looping replay wrapped twice in 8 s; `reset` rewound scan 8 → 0.
- Live browser (headless Chrome): connected, ~10.5 Hz frames, click → track re-acquires exactly, drag → true_pose (1.20, 0.26) confirmed server-side, cell highlight follows, no console errors.
- Game headless: full X-win (5 moves) at S=1.0 and S=2.0 through the real two-thread app.

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