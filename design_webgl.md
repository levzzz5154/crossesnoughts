# Design: X3 Lidar Emulator + 3D WebGL Scene + Object Tracker (WebSocket Approach)

**Decision (vs. alternatives):** Browser WebGL (three.js) + Python backend (FastAPI/uvicorn) + JSON frames over WebSocket with latest-wins broadcast. Chosen over pygame-ce / vispy / matplotlib because the context research shows the browser dodges Wayland/X11 GL-context friction on CachyOS, gives the highest-quality 3D visualization with trivial dependency-free serving, and the WS boundary cleanly separates the sim/swap layer from the renderer (context §Viz research findings).

---

## 1. Architecture

### 1.1 Topology

```
┌───────────────────────────── Python process (single) ─────────────────────────────┐
│                                                                                    │
│  ┌──────────────┐  scan  ┌──────────────┐  scan    ┌──────────────┐  frame  ┌─────┐│
│  │ LidarSource  │───────▶│  Raycaster   │─────────▶│   Tracker    │────────▶│  WS ││
│  │ (Sim or Real)│        │ (numpy)      │          │ (numpy)      │         │ Hub ││
│  └──────────────┘        └──────────────┘          └──────────────┘         └──┬──┘│
│        ▲                                                                         │   │
│        │ scan loop thread (10 Hz)                                               │   │
│  ┌─────┴──────┐   latest-wins (drop stale)                                      │   │
│  │  Scene     │◀────────────────────────────────────────────────────────────────┘   │
│  │  Model     │  (shared, lock-protected)                                          │
│  └────────────┘                                                                     │
│        ▲                                                                             │
│  ┌─────┴───────────────────────────────────────────────┐  uvicorn event loop        │
│  │ FastAPI app: static files + /ws WebSocket           │  (async, no blocking)     │
│  └─────────────────────────────────────────────────────┘                            │
└────────────────────────────────────────────────────────────────────────────────────┘
        │  HTTP /static/*        │  WS ws://host:8000/ws  (JSON frames, server→client)
        ▼                        ▼
┌────────────────────────────────────────────────────────────────────────────────────┐
│ Browser: index.html → three.module.js (vendored) → main.js (scene, WS client)     │
└────────────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 Threads and data flow

| # | Thread | Role |
|---|--------|------|
| 1 | **main** | uvicorn runs here (`uvicorn.run(app, ...)` in-process). Serves static files and the `/ws` WebSocket endpoint. Never blocks: all lidar work happens in thread 2. |
| 2 | **scan loop** | `while running: scan = source.doProcessSimple(...)` at 10 Hz; raycast → noise → track → publish. Owns the only copy of `LaserScan`-shaped data. |

- Thread 2 produces one `Frame` per scan and stores it in a shared `SceneModel` under a `threading.Lock` (single slot — **latest-wins**; if the WS layer is slower than 10 Hz, older frames are dropped, never queued).
- The WS endpoint reads the slot: on every client connect it sends a `hello` + `scene` message, then loops `await websocket.receive_text()` with a short timeout, re-sending the current frame only when `frame.seq != last_sent_seq`. This is a **poll-on-demand** pattern: no producer→consumer queue, no stale-frame backlog, no broadcaster bookkeeping. At 10 Hz this is trivially cheap.
- Client→server messages are only `reset` (restart sim with a fresh seed) — see §7. Everything else is server→client.

### 1.3 Why latest-wins + poll-on-demand

The lidar produces 10 scans/s; browsers need ~10–30 fps. A `queue.Queue` would accumulate frames when the client stalls (tab backgrounded, GC pause) and replay stale data on resume. The single-slot design means the client always sees the newest complete frame; the `seq` field lets the client detect drops and interpolate if desired. This matches the context's "latest-wins broadcast (drop stale frames)" requirement exactly.

---

## 2. File / module layout

```
crossesnoughts/
├── server/                        # Python backend package
│   ├── __init__.py
│   ├── main.py                    # FastAPI app, /ws endpoint, static mount, startup/shutdown
│   ├── config.py                  # SceneConfig, LidarConfig, NoiseConfig dataclasses (all constants)
│   ├── scan.py                    # LaserScan / LaserPoint / LaserConfig pure-Python dataclasses
│   ├── lidar_source.py            # LidarSource ABC + SimLidarSource + RealLidarSource + factory
│   ├── sim.py                     # SimLidarSource.impl: scene model, object motion, noise, raycast
│   ├── real.py                    # RealLidarSource.impl: ydlidar.CYdLidar wrapper
│   ├── tracking.py                # Tracker: cluster → centroid → association → board coords
│   ├── frames.py                  # Frame dataclass + to_json (wire schema) + SceneModel
│   ├── transforms.py              # lidar-frame ↔ board-frame math (pure functions)
│   └── run.py                     # entry point: python -m server.run  (uvicorn.run in-process)
├── static/
│   ├── index.html                 # page, canvas, HUD overlay
│   ├── three.module.js            # vendored three.js r185 (single file, no build step)
│   ├── main.js                    # scene graph build, WS client, animation loop
│   └── style.css
├── tests/
│   ├── test_raycast.py            # raycast math vs. analytic ground truth
│   ├── test_noise.py              # noise model statistical properties
│   ├── test_tracking.py           # cluster/centroid/association behavior
│   ├── test_transforms.py         # lidar→board round trips
│   ├── test_wire.py               # Frame.to_json schema compliance
│   └── test_swap.py               # SimLidarSource API parity with RealLidarSource
├── requirements.txt               # fastapi, uvicorn[standard], numpy
└── README.md                      # run instructions (uvicorn + open browser)
```

**Rationale for one Python package `server/`:** the backend is one process with one concern; flat modules avoid a pointless `app/`/`core/` split for ~1200 lines.

---

## 3. Coordinate frames and transforms

### 3.1 Conventions (fixed, cited)

- **CW-positive radians**: angle θ ∈ [−π, π], θ = 0 is the lidar's forward (+X) axis, and θ increases **clockwise** when viewed from the top. Direction of a ray at θ is `d = (cos θ, sin θ)` in the lidar's 2D plane. **Source:** `YDLidar-SDK/core/common/ydlidar_def.h` lines 130–133 ("0 is forward and angles are measured clockwise when viewing YDLIDAR from the top") and the SDK's own polar→Cartesian conversion `Yd2DPoint(y*cos(x), y*sin(x))` in `examples/gs_test2.cpp:58`. The context states the same convention (§X3 hardware facts).
- **World (three.js) frame**: right-handed, X east, Y up, Z south (three.js default: camera looks down −Z). `x_world = x_board`, `z_world = y_board`, `y_world = height`. This makes the board plane `y = 0` and keeps the scene readable from the default camera.
- **Board frame**: 2D, origin at the board's **back-left corner** (as seen from the lidar at front-right), +X along the board's long edge pointing from back-left **toward the front edge** (toward the lidar), +Y along the board's short edge pointing **left → right** (toward the lidar's side). In world: `x_board = x_world`, `y_board = z_world`.
- **Lidar frame**: 2D, origin at the lidar, +X = forward = along the board's long edge pointing from front edge toward back edge (opposite of board +X), +Y = 90° CW from +X, i.e. pointing along the board's short edge toward the back-left corner.

### 3.2 Geometry (board = W × D, lidar at front-right corner)

```
        board (W = depth, D = width) — top view, lidar at FRONT-RIGHT corner
        y_board (world +Z) ▲
                           │
        back-left ◄────────┼──────────► back-right
        (0,0)              │              (0, W)
                           │   ┌──────────────┐
                           │   │              │
                           │   │    board     │   lidar ⊙ at (D, W)
                           │   │              │   (front-right corner)
                           │   └──────────────┘
        front-left ◄───────┼──────────────────► front-right
        (D, 0)             │                    (D, W)
        ───────────────────┴───────────────────► x_board (world +X)
```

- Board corners in board coords: back-left (0, 0), back-right (0, W), front-left (D, 0), front-right (D, W) — where D = depth along X, W = width along Y (both positive).
- Lidar position (board coords): `p_L = (D, W)` — mounted at floor level (scan plane = board surface plane, `y = 0`), at the front-right corner, per the scene spec.
- Lidar axes in board coords: `+X_L = (−1, 0)` (forward = toward back edge), `+Y_L = (0, −1)` (90° CW from +X_L = toward back-left corner). I.e. the lidar is rotated by π relative to the board frame.

### 3.3 Transform formulas

**Lidar → board** (point `(x_L, y_L)` in lidar frame → `(x_B, y_B)` in board frame):

```
x_B = D − x_L
y_B = W − y_L
```

**Lidar polar → board** (scan point at CW angle θ, range r):

```
x_B = D − r·cos θ
y_B = W − r·sin θ
```

**Board → lidar** (inverse, used by the sim to raycast from the lidar):

```
x_L = D − x_B
y_L = W − y_B
θ   = atan2(y_L, x_L)     (numpy: np.arctan2; result in [−π, π], CW-positive by convention)
r   = hypot(x_L, y_L)
```

**Board → world (3D)**: `(x, y, z) = (x_B, 0, y_B)` — scan points live on the board plane, `y = 0`. All objects (board, object, lidar marker) sit at `y = 0` or above; the box spans `y ∈ [0, H]`.

**World → screen**: handled by three.js `PerspectiveCamera` + `OrbitControls`; no manual projection.

All transforms are in `server/transforms.py` as pure functions (numpy, no state):

```python
def polar_to_board(theta_rad: np.ndarray, r_m: np.ndarray, cfg: SceneConfig) -> np.ndarray:
    """(N,) angles (CW-positive rad), (N,) ranges (m) -> (N,2) board coords."""
    return np.column_stack((cfg.board_depth - r_m * np.cos(theta_rad),
                            cfg.board_width  - r_m * np.sin(theta_rad)))

def board_to_lidar(xy: np.ndarray, cfg: SceneConfig) -> np.ndarray:
    """(N,2) board coords -> (N,2) lidar coords."""
    return np.column_stack((cfg.board_depth - xy[:, 0], cfg.board_width - xy[:, 1]))

def board_to_lidar_polar(xy: np.ndarray, cfg: SceneConfig) -> tuple[np.ndarray, np.ndarray]:
    """(N,2) board coords -> (theta_rad[N], r_m[N]); theta CW-positive in [-pi, pi]."""
    xy_l = board_to_lidar(xy, cfg)
    return np.arctan2(xy_l[:, 1], xy_l[:, 0]), np.hypot(xy_l[:, 0], xy_l[:, 1])
```

### 3.4 Scene config (defaults, `server/config.py`)

```python
@dataclass(frozen=True)
class SceneConfig:
    board_depth: float = 2.0   # D, meters, along board X (lidar forward axis is -X_B)
    board_width: float = 2.0   # W, meters, along board Y
    box_w: float = 4.0         # room interior, X
    box_d: float = 4.0         # room interior, Z
    box_h: float = 2.5         # room interior, Y (walls)
    wall_t: float = 0.1        # wall/floor thickness (visual only)
    floor_y: float = 0.0       # board plane height in world
```

Board is centered in the box: box spans `x ∈ [1, 3]`, `z ∈ [1, 3]` in world (board `x,z ∈ [0,2]` offset by +1; equivalently box corners at world (1,0,1) and (3,0,3)). Lidar at board corner (D=2, W=2) → world (3, 0, 3): the **front-right corner is the box corner nearest the camera's default position** (camera at world (4, 3, 4) looking at (2, 0.6, 2)), so the lidar and the object are both in view.

---

## 4. Lidar emulation (`server/sim.py`)

### 4.1 API-level emulation (decision)

**Chosen: (a) API-level pure-Python class mirroring the SWIG module.** The context's research (§Emulation research findings) recommends this for drop-in swap, and it is the only level that can run without a PTY and without reimplementing the binary protocol. Serial-level (pty) fidelity is explicitly out of scope; the noise model (§5) recovers most of its realism. The class is a **duck-typed twin** of `ydlidar.CYdLidar`/`LaserScan`/`LaserPoint`/`LaserConfig` (verified against `YDLidar-SDK/build/python/ydlidar.py`): same method names, same option-setting pattern, same field names.

### 4.2 Dataclasses (`server/scan.py`) — mirror SWIG shapes exactly

```python
@dataclass
class LaserPoint:
    angle: float      # radians, CW-positive, normalized to [-pi, pi]
    range: float      # meters; 0.0 = invalid/no return
    intensity: float  # always 0.0 (X3 has no intensity)

@dataclass
class LaserConfig:
    min_angle: float      # rad, = -pi
    max_angle: float      # rad, = +pi
    angle_increment: float  # rad = 2*pi/(N-1)
    time_increment: float   # s, = scan_time / N
    scan_time: float        # s, = 1/scan_freq
    min_range: float        # m = 0.10 (X3 hardware min)
    max_range: float        # m = 8.0  (X3 hardware max)

@dataclass
class LaserScan:
    stamp: int            # uint64 ns, time.time_ns() at scan start
    scanFreq: float       # Hz
    sampleRate: float     # kHz (3.0)
    points: list[LaserPoint]
    size: int             # len(points)
    config: LaserConfig
    moduleNum: int        # 0
    envFlag: int          # 0
```

Field names verified from `build/python/ydlidar.py` (LaserScan: `stamp`, `scanFreq`, `sampleRate`, `points`, `size`, `config`, `moduleNum`, `envFlag`; LaserPoint: `angle`, `range`, `intensity`; LaserConfig: `min_angle`, `max_angle`, `angle_increment`, `time_increment`, `scan_time`, `min_range`, `max_range`).

### 4.3 Option constants (`server/scan.py` or `server/lidar_source.py`)

Verified against `YDLidar-SDK/core/common/ydlidar_def.h` (lines 61–86). **Note: the context's guessed values were wrong; the header is authoritative:**

```python
LidarPropSerialPort       = 0
LidarPropIgnoreArray      = 1
LidarPropSerialBaudrate   = 10
LidarPropLidarType        = 11
LidarPropDeviceType       = 12
LidarPropSampleRate       = 13
LidarPropAbnormalCheckCount = 14
LidarPropIntenstiyBit     = 15
LidarPropMaxRange         = 20
LidarPropMinRange         = 21
LidarPropMaxAngle         = 22
LidarPropMinAngle         = 23
LidarPropScanFrequency    = 24
LidarPropFixedResolution  = 30
LidarPropReversion        = 31
LidarPropInverted         = 32
LidarPropAutoReconnect    = 33
LidarPropSingleChannel    = 34
LidarPropIntenstiy        = 35
LidarPropSupportMotorDtrCtrl = 36
LidarPropSupportHeartBeat = 37
```

### 4.4 Point count (verified formula)

The SDK sizes the scan from the angular field of view and the angle increment: `size = (max_angle − min_angle)/angle_increment + 1` (`core/common/ydlidar_datatype.h:94`, `ydlidar_def.h:181`), with `angle_increment = from_degrees(m_field_of_view)/(all_node_count − 1)` (`src/CYdLidar.cpp:634-635`) — i.e. for a full 360° scan, **N = 300 points at 3 kHz / 10 Hz** (context §X3 facts: `round((sample_rate*1000)/(scan_frequency−0.1))` → 300). The emulator uses exactly N = 300 with `angle_increment = 2π/300` and angles `θ_i = −π + i·Δθ` for `i ∈ [0, 300)`. (The real device's −0.1 Hz fudge is a hardware quirk; the emulator keeps the clean 300 to match the documented formula.)

### 4.5 SimLidarSource (public API — the swap surface)

```python
class SimLidarSource(LidarSource):
    """Duck-typed twin of ydlidar.CYdLidar for the API level."""

    def __init__(self, cfg: SimConfig, seed: int | None = None) -> None
    # -- ydlidar.CYdLidar mirror --
    def setlidaropt(self, prop: int, value) -> bool          # stores; returns True
    def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]
    def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]
    def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]
    def getlidaropt_toString(self, prop: int) -> tuple[bool, str]
    def initialize(self) -> bool      # validates opts, builds rng; True
    def turnOn(self) -> bool          # starts internal clock; True
    def doProcessSimple(self, scan: LaserScan) -> bool  # fills scan; True
    def turnOff(self) -> None
    def disconnecting(self) -> None
    # -- sim extras (not on CYdLidar; used by the server only) --
    def set_object_pose(self, xy_board: tuple[float, float]) -> None  # manual override
    def reset(self, seed: int | None = None) -> None      # new trajectory + rng
    @property
    def object_pose(self) -> tuple[float, float]          # current true board pose (x_B, y_B)
    @property
    def scan_freq(self) -> float
```

`doProcessSimple` semantics mirror the real call: it is **blocking up to `1000/scan_freq` ms** (sleeps to hold 10 Hz cadence), fills the passed `LaserScan` in place, returns `True`. `initialize()` returns `False` on invalid options (e.g. scan frequency outside 4–8 Hz for an X3 — the hardware range; the context's example config uses 10.0 anyway, so the emulator **accepts 4–10 Hz** and warns outside 4–8).

### 4.6 Simulated scene + object motion

The sim owns a **true scene** (not the noisy observed one):

- Board: rectangle `[0, D] × [0, W]` in board coords (a ray hitting the board edge beyond the object is a valid return).
- Box walls: four segments at the box perimeter, world `x ∈ [1,3]`, `z ∈ [1,3]`, i.e. board coords `x_B ∈ [1,3]`, `y_B ∈ [1,3]`; wall segments at `x_B = 1`, `x_B = 3`, `y_B = 1`, `y_B = 3` (board is inset 1 m from each wall). Rays that miss the board still hit a wall (max range 8 m > box diagonal ≈ 2.83 m), so the scan always has returns — realistic for an enclosed room.
- Object: a **circle of radius `r_obj = 0.05 m`** (5 cm — a small can/cylinder; the context says "ONE object on a square board", concurrency of one). It intersects the scan plane (sits on the board), so the lidar sees a ~10 cm arc of points.
- Object motion: **deterministic Lissajous-ish path** within the board, seeded by `seed` so tests are reproducible:

```python
t = self._t  # seconds since turnOn
x_B = 0.5 + 0.4 * math.sin(2π * 0.10 * t + φ1)   # slow drift
y_B = 0.5 + 0.4 * math.cos(2π * 0.07 * t + φ2)   # different frequency
```

with `φ1, φ2` drawn from the seed. Speed ≈ 0.25 m/s max — comfortably below the 10 Hz scan rate, so the tracker sees smooth motion. `set_object_pose` overrides the path (used by tests and by a future "drag the object" UI).

The raycast uses the **true** object pose; the tracker sees only the noisy scan points (§5–6).

### 4.7 Raycasting (`server/sim.py` — pure numpy, vectorized)

For each of the N=300 angles θ_i, cast a ray from the lidar origin in direction `d_i = (cos θ_i, sin θ_i)` (CW-positive convention). Find the nearest intersection with any primitive in `[0.10, 8.0]` m; report that range. Primitives:

1. **Circle** (object), center `c`, radius `r`:
   Quadratic `|c + t·d|² = r²` → `t·d·d·t + 2(c·d)t + (|c|² − r²) = 0` with `d·d = 1`:
   ```
   a = 1,  b = c·d,  cq = |c|² − r²
   disc = b² − cq
   if disc < 0: no hit
   t = −b − sqrt(disc)          # nearer root
   hit if t > 0
   ```
2. **Segments** (board edges, walls): segment `p0 → p1`; solve `p0 + s·(p1−p0) = t·d`, `s ∈ [0,1]`, `t > 0`:
   ```
   denom = d × (p1−p0)          # 2D cross
   if |denom| < 1e-12: parallel, skip
   s = ((p0) × d) / denom       # using cross((p0 − origin), d) with origin = (0,0)
   t = (p0 × (p1−p0)) / denom
   hit if s ∈ [0,1] and t > 0
   ```
   (all in lidar frame; origin is the lidar at (0,0)).

Vectorized: build arrays of all primitives' parameters once per scan (`seg_p0`, `seg_p1`, circle `c`, `r`), then for all 300 rays at once with broadcasting — `rays (300,2)`, segments `(4+4, 2)` → `(300, 8)` t-values, circle → `(300,)`. Take `t_min = min over primitives`; `r = t_min` if `t_min ≤ 8.0` else `8.0` (max-range clamp; walls guarantee a hit anyway). Points with `t_min < 0.10` are clamped to `0.0` (invalid) — the X3 min-range blind spot.

Complexity: O(N × P) with P = 9 primitives → 2700 ray-primitive tests per scan, microseconds in numpy. No acceleration structure needed.

### 4.8 Noise model (`server/sim.py`, `NoiseConfig`)

Heteroscedastic triangulation-lidar noise (context §Emulation research findings, Alhashimi ICINCO 2016: σ ∝ d²):

```python
@dataclass(frozen=True)
class NoiseConfig:
    range_std_a: float = 0.004    # m  — constant term (σ at d→0)
    range_std_b: float = 0.002    # m/m² — quadratic term (σ ∝ d²)
    angular_jitter_std: float = 0.0017  # rad ≈ 0.1°
    dropout_rate: float = 0.02    # 2% of points dropped per scan
    ghost_rate: float = 0.001     # 0.1% stray points
    min_range: float = 0.10       # X3 hardware blind spot
    max_range: float = 8.0
```

Per scan, per point:

```
σ_r(d) = range_std_a + range_std_b · d²          # e.g. 0.004 + 0.002·4 = 0.012 m at 2 m, 0.02 m at 2.83 m
θ' = θ + N(0, angular_jitter_std)
r' = max(0, r + N(0, σ_r(d)))
```

- **Dropouts**: Bernoulli(0.02) → point becomes invalid (`range = 0.0`).
- **Ghosts**: Bernoulli(0.001) → one extra point at a random angle with range `U(0.5, 7.0)` m (rare stray; the tracker must tolerate them — §6).
- **Blind spot**: any true hit `d < 0.10` → `range = 0.0` (invalid). Object radius 5 cm at ≥ 0.5 m from the corner is never in the blind spot in the default trajectory, but the guard is there for manual poses.
- **Edge behavior (mixed pixels)**: for the two rays whose circle hit is within 1° of the object silhouette tangent, the range is blended toward the background: `r = r_circle + U(0, 0.3)·(r_bg − r_circle)` — simulates the mixed-pixel effect at object silhouettes (context §Emulation research findings). Implemented as a deterministic post-pass on the two indices nearest the tangent angles.
- **Angular jitter** is applied to θ before the raycast (the ray is cast at the jittered angle) — this is the physically correct order: the motor/encoder error shifts the measured direction, and the range is whatever is along that direction. Range noise is applied to the returned distance after the hit test.

All noise uses `rng = np.random.default_rng(seed)`; `reset(seed)` re-seeds so tests are deterministic.

### 4.9 RealLidarSource (`server/real.py`)

```python
class RealLidarSource(LidarSource):
    """Wraps ydlidar.CYdLidar (SWIG, SDK v1.2.20). Same surface as SimLidarSource."""

    def __init__(self, port: str = "/dev/ttyUSB0") -> None
    def setlidaropt(self, prop: int, value) -> bool   # delegates to _dev.setlidaropt
    def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]
    def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]
    def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]
    def getlidaropt_toString(self, prop: int) -> tuple[bool, str]
    def initialize(self) -> bool                      # ydlidar.os_init() + _dev.initialize()
    def turnOn(self) -> bool                          # _dev.turnOn()
    def doProcessSimple(self, scan: LaserScan) -> bool
        # _dev.doProcessSimple(scan); returns its bool. scan is the SWIG LaserScan
        # object — same field names as our dataclass, so downstream code is agnostic.
    def turnOff(self) -> None
    def disconnecting(self) -> None
```

`LaserScan` here is the **SWIG object** (not the dataclass) — the consumer (raycast/tracker in `server/`) only reads `scan.points[i].angle/.range` and `scan.config.*`, which exist identically on both. `ydlidar.os_shutdown()` is called in `disconnecting()`.

### 4.10 Factory (`server/lidar_source.py`)

```python
def make_lidar_source(kind: str, port: str | None = None, seed: int | None = None) -> LidarSource:
    """kind: 'sim' (default) | 'real'. Raises ValueError on unknown kind."""
```

Selected via `--lidar sim|real` on the CLI (`server/run.py`). **No consumer code changes when swapping**: the scan loop, raycast, tracker, and WS layer only ever see the `LidarSource` ABC.

```python
class LidarSource(ABC):
    @abstractmethod def setlidaropt(self, prop: int, value) -> bool: ...
    @abstractmethod def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]: ...
    @abstractmethod def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]: ...
    @abstractmethod def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]: ...
    @abstractmethod def getlidaropt_toString(self, prop: int) -> tuple[bool, str]: ...
    @abstractmethod def initialize(self) -> bool: ...
    @abstractmethod def turnOn(self) -> bool: ...
    @abstractmethod def doProcessSimple(self, scan) -> bool: ...
    @abstractmethod def turnOff(self) -> None: ...
    @abstractmethod def disconnecting(self) -> None: ...
```

---

## 5. Tracking (`server/tracking.py`)

### 5.1 Pipeline (per scan, all numpy)

```
scan points (polar) ──► filter valid (r > 0.10, r < 8.0) ──► to board coords
      ──► subtract static background (board/wall points) ──► cluster
      ──► pick object cluster (largest) ──► weighted centroid (board coords)
      ──► association (one-object Kalman-ish filter) ──► publish (x_B, y_B, conf)
```

### 5.2 Static background subtraction

The board edges and walls are **static geometry known to the sim** — but the tracker must not cheat. Instead it learns the background once: on the first scan after `turnOn` (or after a `reset`), it records the expected range per angle bin by raycasting the static scene (board + walls, no object). A point is "object" if `|r_measured − r_static(θ)| > 0.03 m` (3 cm ≈ 2.5× the max noise σ at 2.8 m). This is exactly what a real deployment would do with an empty-room calibration scan, so the swap to `RealLidarSource` needs no tracker change (the real device would get a calibration scan at startup).

Threshold rationale: noise σ ≤ 0.02 m at max range (NoiseConfig), so 0.03 m separates object hits from background with margin; the object (10 cm chord) is always ≥ 3 cm from the board edge in the default trajectory.

### 5.3 Clustering

After background subtraction, cluster remaining points with **connected-components on angle adjacency**: two points `i, j` (sorted by θ) are in the same cluster if `Δθ < 3·Δθ_scan` (3 bins ≈ 3.6°) **and** `|r_i − r_j| < 0.25 m`. The object's silhouette spans ~10 cm over ~4–5 bins (object radius 5 cm at ~1 m: chord angle ≈ 2·atan(0.05/1) ≈ 5.7° ≈ 5 bins), so a 3-bin adjacency with a 0.25 m range gate cleanly separates it from ghosts and wall fragments. Implementation: single pass over the sorted-by-θ points, union-find or greedy label propagation; O(N).

### 5.4 Object selection + centroid

- Drop clusters with < 2 points (noise floor; a single ghost point is not an object).
- Pick the **largest cluster** (by point count). If none: `tracked = None` (object lost this scan).
- Weighted centroid in board coords: `w_i = 1/(σ_r(d_i)² + 1e-6)` (inverse-variance weights — closer points are more accurate), then

```
x̂_B = Σ w_i x_i / Σ w_i
ŷ_B = Σ w_i y_i / Σ w_i
```

Because the object is a circle, the centroid of the visible arc is a biased estimate of the circle center (the arc's chord centroid sits slightly toward the lidar). **Bias correction**: for a circle of radius r at distance d, the visible half-angle is `α = asin(r/d)`; the measured arc centroid is offset from the true center along the ray by `δ ≈ r²/(2d)` (small-angle expansion). We correct: `c_true ≈ c_meas + (r_obj²/(2·d̂)) · (unit vector from lidar to c_meas)`. With r = 0.05 m, d = 1 m: δ = 1.25 mm — **below the noise floor**, so the design includes the correction as a documented constant `OBJECT_RADIUS_BIAS = r_obj²/(2·d̂)` applied only when `d̂ > 0.3 m`, but flags it as negligible (kept for correctness when the object is close to the lidar, where δ grows: at d = 0.3 m, δ ≈ 4 mm).

### 5.5 Association (one object → one track)

```python
@dataclass
class TrackState:
    pos: np.ndarray      # (2,) board coords (x_B, y_B)
    vel: np.ndarray      # (2,) m/s
    last_seen: float     # monotonic seconds
    conf: float          # 0..1

class Tracker:
    def __init__(self, dt: float = 0.1, process_noise: float = 0.05, meas_noise: float = 0.01) -> None
    def update(self, z: np.ndarray | None, t: float) -> TrackState | None
```

- **Prediction**: `pos += vel·Δt` (constant-velocity model).
- **Association**: since there is exactly one object, the measurement is associated if `|z − pos_pred| < 0.5 m` (generous gate; the object moves ≤ 2.5 cm between scans). If the gate fails or no cluster exists, the measurement is treated as missing: `vel` decays by 0.5 per scan, `conf` decays; after 20 consecutive misses the track is dropped (`update` returns `None`).
- **Update** (scalar-gain α-β filter, deliberately simpler than a full Kalman — one object, near-constant velocity, and the Kalman's covariance bookkeeping buys nothing here):

```
α = 0.6, β = 0.3
pos += α · (z − pos)
vel += (β/Δt) · (z − pos)
conf = min(1.0, conf + 0.15)        # on successful association
conf = max(0.0, conf − 0.05)        # on missing measurement
```

- Output: `TrackState` with `pos` in board coords, `conf`. The server publishes `tracked: null` while `conf < 0.3` (not yet confident / lost).

### 5.6 Tracker output (board-space coordinates)

The **deliverable** of the whole system: `(x_B, y_B)` — object position in board coords (origin at back-left corner, +X toward the front edge, +Y toward the lidar side), meters, plus `conf ∈ [0,1]`. This is what the frontend displays as the coordinate readout and marker.

---

## 6. Wire schema (WebSocket JSON frames, server → client)

All frames are JSON objects with a `type` discriminator. Numbers are float64 JSON; arrays are plain JSON arrays (no typed arrays in the wire format — the payload is ~300 points ≈ 5 KB, trivially small; typed-array binary framing is an optimization we do not need at 10 Hz).

### 6.1 `hello` (sent once, immediately after connect)

```json
{
  "type": "hello",
  "version": 1,
  "config": {
    "board_depth": 2.0,
    "board_width": 2.0,
    "box_w": 4.0, "box_d": 4.0, "box_h": 2.5,
    "lidar_pos_board": [2.0, 2.0],
    "scan_freq": 10.0,
    "sample_rate": 3.0,
    "point_count": 300,
    "min_range": 0.1, "max_range": 8.0,
    "object_radius": 0.05
  }
}
```

Purpose: lets the client build the scene (box/board/lidar marker) from server truth instead of duplicating constants; `point_count` sizes the preallocated buffer geometry.

### 6.2 `scene` (sent once per connect, before the first `frame`)

```json
{
  "type": "scene",
  "segments": [
    {"x1": 1.0, "y1": 1.0, "x2": 3.0, "y2": 1.0},
    {"x1": 3.0, "y1": 1.0, "x2": 3.0, "y2": 3.0},
    {"x1": 3.0, "y1": 3.0, "x2": 1.0, "y2": 3.0},
    {"x1": 1.0, "y1": 3.0, "x2": 1.0, "y2": 1.0}
  ],
  "board": {"x": 0.0, "y": 0.0, "depth": 2.0, "width": 2.0}
}
```

`segments` are the box walls in board coords (world: `x = x`, `z = y`); `board` is the board rect in board coords. The client renders these as the room/board. (The frontend could hardcode this, but sending it keeps the client a pure renderer of server state — one source of truth.)

### 6.3 `frame` (10 Hz, latest-wins)

```json
{
  "type": "frame",
  "seq": 1234,
  "stamp_ns": 1724800000000000000,
  "scan_freq": 10.0,
  "points": [
    {"x": 1.994, "y": 1.983},
    {"x": 1.981, "y": 1.954},
    ...
  ],
  "points_invalid": [17, 42, 103],
  "tracked": {"x": 0.612, "y": 0.733, "conf": 0.87},
  "true_pose": {"x": 0.610, "y": 0.735}
}
```

| Field | Type | Meaning |
|---|---|---|
| `type` | string | `"frame"` |
| `seq` | int | monotonically increasing frame counter; the client uses it to detect drops |
| `stamp_ns` | int | `time.time_ns()` at scan start (uint64) |
| `scan_freq` | float | actual scan frequency of this frame |
| `points` | array of `{x, y}` | **valid** scan points, **already in board coords** (browser does no trig — context §Viz research findings); each `{x, y}` is `(x_B, y_B)` in meters |
| `points_invalid` | array of int | indices of dropped/invalid points (as indices into the 300-angle grid, in scan order); the client can render them dimmed or skip them |
| `tracked` | object \| null | tracker output: `{x, y, conf}` in board coords; `null` when the track is lost/not confident |
| `true_pose` | object \| null | **sim-only**: true object pose `{x, y}` (board coords). `null` when running against `RealLidarSource`. Lets the user visually verify tracker accuracy in sim mode; the tracker never sees it. |

`points` are precomputed server-side (`polar_to_board`), so the browser only does `new THREE.Vector3(p.x, 0, p.y)` — no trig, no coordinate math in JS. At 300 points × 10 Hz this is ~300 KB/s of JSON — fine for localhost.

### 6.4 Client → server

```json
{"type": "reset", "seed": 42}
```

Resets the sim: new object trajectory, re-seeded noise, fresh background calibration. `seed` optional (server picks random). Server replies with a fresh `hello` + `scene` + frames. Unknown message types are ignored (forward compat).

### 6.5 Frame dataclass + serializer (`server/frames.py`)

```python
@dataclass
class Frame:
    seq: int
    stamp_ns: int
    scan_freq: float
    points: np.ndarray          # (M,2) board coords, valid points only
    points_invalid: list[int]   # grid indices of invalid points
    tracked: TrackState | None
    true_pose: tuple[float, float] | None

    def to_json(self) -> str: ...   # json.dumps with exact key order above

class SceneModel:
    """Single-slot latest-wins store, safe for cross-thread use."""
    def __init__(self) -> None: self._lock = threading.Lock(); self._frame: Frame | None = None
    def publish(self, frame: Frame) -> None: ...   # lock, replace
    def current(self) -> Frame | None: ...         # lock, return
    @property
    def seq(self) -> int: ...                      # last published seq (lock)
```

---

## 7. Backend server (`server/main.py`, `server/run.py`)

### 7.1 Decision: FastAPI + uvicorn[standard] vs plain `websockets`

**Chosen: FastAPI + uvicorn[standard].** One dependency set serves both static files and `/ws` (no second static-file server, no CORS config since same origin), and the async endpoint code is identical to plain `websockets` — the extra dependency is small and already the context's recommendation (§Viz research findings: "FastAPI + uvicorn[standard] (or minimal websockets lib), one process: static files + /ws endpoint").

### 7.2 `server/main.py`

```python
app = FastAPI()
app.mount("/static", StaticFiles(directory="static"), name="static")

scene_model = SceneModel()          # module-level singletons (one process)
lidar: LidarSource | None = None
scan_thread: threading.Thread | None = None
state_lock = threading.Lock()       # guards lidar/thread lifecycle

@app.on_event("startup")  # (or lifespan context)
def _startup() -> None:
    global lidar, scan_thread
    lidar = make_lidar_source(settings.lidar_kind, settings.port, settings.seed)
    # mirror of tri_test.py option pattern (context §X3 facts):
    lidar.setlidaropt(LidarPropSerialBaudrate, 115200)
    lidar.setlidaropt(LidarPropLidarType, TYPE_TRIANGLE)          # 0
    lidar.setlidaropt(LidarPropDeviceType, YDLIDAR_TYPE_SERIAL)   # 1
    lidar.setlidaropt(LidarPropScanFrequency, 10.0)
    lidar.setlidaropt(LidarPropSampleRate, 3)
    lidar.setlidaropt(LidarPropSingleChannel, True)
    lidar.setlidaropt(LidarPropMaxAngle, 180.0)
    lidar.setlidaropt(LidarPropMinAngle, -180.0)
    lidar.setlidaropt(LidarPropMaxRange, 16.0)
    lidar.setlidaropt(LidarPropMinRange, 0.08)
    lidar.setlidaropt(LidarPropIntenstiy, False)
    assert lidar.initialize() and lidar.turnOn()
    scan_thread = threading.Thread(target=scan_loop, daemon=True)
    scan_thread.start()

@app.on_event("shutdown")
def _shutdown() -> None:
    lidar.turnOff(); lidar.disconnecting()

def scan_loop() -> None:
    """10 Hz: raycast+noise (inside SimLidarSource.doProcessSimple) → track → publish."""
    scan = LaserScan(...)          # reused buffer (dataclass or SWIG object)
    tracker = Tracker()
    while True:
        ok = lidar.doProcessSimple(scan)   # blocks ~100 ms; True
        if not ok: continue
        pts = np.array([(p.angle, p.range) for p in scan.points])  # (300,2)
        xy = polar_to_board(pts[:, 0], pts[:, 1], scene_cfg)
        valid = (pts[:, 1] > 0.10) & (pts[:, 1] < 8.0)
        tracked = tracker.update(centroid if any else None, time.monotonic())
        frame = Frame(seq=next_seq(), stamp_ns=scan.stamp, scan_freq=scan.scanFreq,
                      points=xy[valid], points_invalid=where(~valid),
                      tracked=tracked, true_pose=lidar.object_pose if sim else None)
        scene_model.publish(frame)

@app.websocket("/ws")
async def ws_endpoint(ws: WebSocket) -> None:
    await ws.accept()
    await ws.send_text(hello_json())
    await ws.send_text(scene_json())
    last_seq = -1
    while True:
        try:
            msg = await asyncio.wait_for(ws.receive_text(), timeout=0.05)
            handle_client_msg(msg)          # "reset" → re-init sim, resend hello/scene
        except asyncio.TimeoutError:
            pass
        except WebSocketDisconnect:
            break
        frame = scene_model.current()
        if frame is not None and frame.seq != last_seq:
            await ws.send_text(frame.to_json())
            last_seq = frame.seq
```

Notes:
- The 50 ms receive timeout is the poll interval — frames are pushed at up to 20 Hz, faster than the 10 Hz production rate, so the client never waits more than 50 ms for a frame.
- `asyncio.wait_for` + `receive_text` is the standard uvicorn pattern for a send-priority loop; a pure `while True: send; receive` would stall sends on a silent client.
- The scan loop is the **only** thread touching `lidar` and `tracker`; the WS handler only reads `SceneModel` (lock-protected) — no shared mutable state beyond that.
- `TYPE_TRIANGLE`/`YDLIDAR_TYPE_SERIAL` constants exist in the real `ydlidar` module; the sim module defines the same names (values 0 and 1) for parity.

### 7.3 `server/run.py`

```python
python -m server.run [--lidar sim|real] [--port /dev/ttyUSB0] [--seed N] [--host 127.0.0.1] [--http 8000]
```

Sets `settings`, then `uvicorn.run(app, host, port)` — one process, one command. `--lidar real` is the swap: same app, same scan loop, same tracker, same WS schema.

---

## 8. Frontend (three.js)

### 8.1 Decision: vendored `three.module.js` vs CDN

**Chosen: vendored single-file `three.module.js` (r185) in `static/`.** The context recommends it (§Viz research findings: "r185 vendored single three.module.js (no npm/build)"); it makes the app work offline and pin the exact version, at the cost of a ~1.2 MB static file served by the same uvicorn process — no build step, no network dependency, no npm.

### 8.2 `static/index.html`

- `<canvas id="scene">` full-window; `<div id="hud">` overlay with the coordinate readout (`X: 0.612 m  Y: 0.733 m  conf: 87%  seq: 1234  source: sim`).
- `<script type="module">` importing `./three.module.js` and `./main.js`.
- A "Reset" button (sends `{"type":"reset"}`).

### 8.3 Scene graph (`main.js`)

```
scene (THREE.Scene, background #101418)
├── box group
│   ├── 4 walls: BoxGeometry(wall_t, box_h, box_d) etc. — MeshStandardMaterial,
│   │   color 0x8899aa, transparent, opacity 0.25, side: DoubleSide
│   └── floor: PlaneGeometry(box_w, box_d), same material, rotated -π/2 about X
├── board: BoxGeometry(board_depth, 0.02, board_width) at y=0.01 — color 0x2a3a4a,
│   opacity 0.9; a thin green edge line (LineSegments + EdgesGeometry) marks the border
├── lidar marker: cylinder (r=0.03, h=0.06) at world (3, 0.03, 3), color 0xff8800,
│   plus a small "+X" arrow (ArrowHelper) pointing forward (−X_B)
├── points: THREE.Points, BufferGeometry with preallocated Float32Array
│   (MAX_POINTS = 512), PointsMaterial(size=0.015, color 0x44ddff, transparent,
│   opacity 0.9); updated per frame from `points` (world: (x, 0, y))
├── track marker: sphere r=0.04 at tracked pos (world (x, 0.04, y)), color 0xff4444;
│   hidden when `tracked` is null
├── true pose marker (sim only): small green sphere, color 0x44ff66
├── ground grid: GridHelper(10, 20) at y=0 for spatial reference
└── lights: HemisphereLight(0xffffff, 0x334455, 1.0) + DirectionalLight(0xffffff, 0.8)
camera: PerspectiveCamera(60, aspect, 0.01, 100) at (4, 3.2, 4) lookAt (2, 0.4, 2)
controls: OrbitControls(camera, canvas) — target (2, 0.4, 2), damping 0.08
renderer: WebGLRenderer({antialias: true}), setPixelRatio(devicePixelRatio), setSize
```

### 8.4 WS client + animation loop

```js
const ws = new WebSocket(`ws://${location.host}/ws`);
let lastSeq = -1, pending = null;
ws.onmessage = (ev) => { pending = JSON.parse(ev.data); };   // latest-wins in JS too
function animate() {
  requestAnimationFrame(animate);
  if (pending) {
    const m = pending; pending = null;
    if (m.type === "hello")      rebuildScene(m.config);
    else if (m.type === "scene") buildRoom(m.segments, m.board);
    else if (m.type === "frame") {
      updatePoints(m.points, m.points_invalid);
      updateTrack(m.tracked);
      updateTruePose(m.true_pose);
      updateHud(m); lastSeq = m.seq;
    }
  }
  controls.update(); renderer.render(scene, camera);
}
```

- `updatePoints`: write `x, 0, y` into the preallocated buffer, set `geometry.setDrawRange(0, m.points.length)`, `attributes.position.needsUpdate = true`. Zero allocation per frame.
- `updateTrack`: `marker.position.set(x, 0.04, y)`, `marker.visible = !!tracked`.
- **Reconnect**: `ws.onclose` → retry with `setTimeout(1000)`; the server resends `hello`/`scene` on each connect, so the client is stateless across reconnects.
- Drops: if `m.seq !== lastSeq + 1`, optionally interpolate the track marker between the last two poses (simple lerp in `animate`); not required for correctness.

### 8.5 Coordinate conventions in the client

- Wire `{x, y}` is board coords → world: `new THREE.Vector3(x, 0, y)` (board X = world X, board Y = world Z).
- The HUD shows board coords verbatim; a small caption on the HUD defines the frame: "board coords: origin back-left corner of board (as seen from lidar), +X toward front edge, +Y toward lidar side".

---

## 9. Dependencies

| Package | Version | Purpose |
|---|---|---|
| `fastapi` | ≥ 0.110 | app + `/ws` endpoint + static mount |
| `uvicorn[standard]` | ≥ 0.29 | ASGI server (in-process) |
| `numpy` | 2.3.4 (present) | raycast, noise, tracking, transforms |
| `ydlidar` | 1.2.20 (installed, SWIG) | `RealLidarSource` only; not imported by the sim path |
| three.js | r185 (vendored `three.module.js`) | browser rendering, no npm |
| browser | any modern (Chrome/Firefox) | WebGL + ES modules |

`requirements.txt`: `fastapi`, `uvicorn[standard]`, `numpy`. No other Python deps. Python 3.12 (conda, present). The vendored three.js file is downloaded once (`curl -O https://unpkg.com/three@0.185.0/build/three.module.js`) into `static/` — pinned, offline-capable.

---

## 10. Risks and mitigations

| Risk | Impact | Mitigation |
|---|---|---|
| **three.js vendoring size/version drift** | 1.2 MB static file; API changes across versions | Pin r185 exactly; only `Scene/WebGLRenderer/PerspectiveCamera/OrbitControls/Points/BufferGeometry` used — stable API surface |
| **WebSocket frame rate vs 10 Hz production** | client sees stale frames after stall | Latest-wins slot + `seq` + 50 ms poll; client drops stale `pending` frames |
| **Tracker errors near board edges** | object arc merges with board-edge background points | 0.03 m background threshold (2.5× noise σ); 0.25 m range gate in clustering; bias correction for near-lidar poses |
| **Real lidar swap breaks consumer** | API mismatch between sim and SWIG module | `LidarSource` ABC + `test_swap.py` asserting identical method/field names; sim mirrors `build/python/ydlidar.py` surface exactly (verified) |
| **Noise model unrealism** | tracker tuned to wrong statistics | Parameters sourced from context research (Alhashimi ICINCO 2016, σ ∝ d²); all in `NoiseConfig` — single place to retune; tests assert statistical bounds only |
| **numpy 2.x API changes** | `np.arctan2`, `np.hypot` fine; `np.random.default_rng` stable | Pin numpy 2.3.4 in requirements |
| **Browser WebGL unavailable** (rare) | blank canvas | `renderer = new WebGLRenderer(...)` wrapped in try/catch → HUD shows clear "WebGL unavailable" error instead of silent failure |
| **uvicorn event-loop blocking** | WS lag if scan loop ran in the loop | Scan loop is a dedicated thread; the loop only does `receive_text` + `send_text` (non-blocking) |
| **Port conflicts / stale processes** | bind error on restart | run.py logs a clear message on `OSError: [Errno 98]`; README documents `pkill -f "server.run"` cleanup |

---

## 11. Acceptance criteria

1. **`python -m server.run`** starts one process; `http://127.0.0.1:8000/` serves the page; opening it shows the box (semi-transparent walls + floor), the board, the orange lidar marker at the front-right board corner, a live cyan point cloud, a red tracked-object marker, and a HUD coordinate readout updating ~10×/s.
2. **Point cloud correctness**: with noise disabled (`NoiseConfig` zeroed via a debug flag), every point lies exactly on the board edge, the object circle, or a wall segment — verified by `test_raycast.py` against analytic ground truth (max error < 1e-9 m).
3. **Board-coordinate output**: the HUD readout and the red marker match the sim's `true_pose` within the noise budget: median |error| < 1.5 cm, p95 < 3 cm over a 60 s run (tracker + bias correction). Verified by `test_tracking.py` on a recorded deterministic trajectory.
4. **Latest-wins**: when the browser tab is backgrounded for 5 s and restored, the first frame shown has `seq` equal to the current server seq (no stale replay) — checked by the client log line on reconnect/resume.
5. **Swap-in**: `--lidar real` runs the identical server code path against `RealLidarSource`; `test_swap.py` asserts `SimLidarSource` and `RealLidarSource` expose the same `LidarSource` methods and that a `LaserScan` produced by either satisfies the same consumer contract (field names/types). (Real-device end-to-end requires hardware; the test covers the interface.)
6. **Reset**: clicking Reset re-seeds the sim (new trajectory), the tracker re-calibrates background, and the cloud/marker continue without client restart.
7. **Dropout tolerance**: with `dropout_rate = 0.02` and `ghost_rate = 0.001`, the tracker never loses the object for more than 3 consecutive scans over a 60 s run (verified by `test_tracking.py`).
8. **Schema compliance**: every WS frame parses against the §6 shapes; `test_wire.py` asserts exact keys and types, and that `points` are all within `[0, board_depth] × [0, board_width]` (board coords) — the browser does no trig.

---

## 12. Build order

1. `server/config.py`, `server/scan.py` (dataclasses + verified LidarProp constants) — no deps.
2. `server/transforms.py` + `tests/test_transforms.py` (round-trip lidar↔board; CW convention).
3. `server/sim.py` raycast core (static scene only) + `tests/test_raycast.py` (analytic ground truth).
4. Noise model + object motion in `sim.py`; `tests/test_noise.py` (σ bounds, dropout rate, determinism under seed).
5. `server/lidar_source.py` (ABC + factory), `SimLidarSource` full API, `server/real.py`; `tests/test_swap.py`.
6. `server/tracking.py` + `tests/test_tracking.py` (cluster/centroid/association; accuracy vs. true pose).
7. `server/frames.py` (Frame, SceneModel, `to_json`) + `tests/test_wire.py`.
8. `server/main.py` + `server/run.py` (scan thread, WS endpoint, static mount).
9. `static/` — vendor three.module.js; `index.html` + `main.js` (scene graph, WS client, HUD); `style.css`.
10. End-to-end smoke: run server, open browser, observe cloud/marker/readout; background-tab latest-wins check; Reset button.
11. README (run instructions, `--lidar real` swap, cleanup note).