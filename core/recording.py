"""Record raw lidar scans to disk and replay them later.

Two pieces, both duck-typed against the LidarSource surface so nothing in
web/ or game/ needs to know whether it is talking to hardware or a file:

* ``RecordingLidarSource`` wraps ANY source (sim or real). Every scan that
  ``doProcessSimple`` returns goes into a buffer that is flushed to a
  compressed .npz periodically and on close. It also captures the background
  table the wrapped source produced, so a replay reproduces the same
  background subtraction instead of having to re-derive it.
* ``ReplayLidarSource`` is a real LidarSource that plays a .npz back, paced
  by the recorded stamps. Loop, speed and seek are supported, which makes it
  usable both as a visualisation feed and as an offline test fixture.

Typical use:

    python -m web.run --lidar real --record board_empty_then_finger.npz
    python -m web.run --lidar replay --replay-file board_empty_then_finger.npz

File format (.npz, compressed):

    ranges     (T, N) float32   metres, 0.0 = invalid, padded for variable N
    angles     (T, N) float32   rad; stored (N,) when every scan shares a grid
    sizes      (T,) int32       valid point count for each scan
    stamps     (T,)   int64     ns, LaserScan.stamp
    bg_angles  (M,)   float64   background table captured at record time
    bg_ranges  (M,)   float64
    meta       0-d unicode      JSON: source kind, board size, created, note

float32 is 4 orders of magnitude below the noise floor (sigma_r >= 4 mm,
angular jitter 1.7 mrad), so the round trip is lossless in every way that
matters; at 10 Hz it costs ~2.4 KB/scan uncompressed.
"""
from __future__ import annotations

import atexit
import json
import threading
import time
import warnings
from pathlib import Path

import numpy as np

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable, LidarSource
from core.scan import (
    LaserPoint,
    LaserScan,
    SCAN_FREQ,
    SCAN_MAX_ANGLE,
    SCAN_MAX_RANGE,
    SCAN_MIN_ANGLE,
    SCAN_MIN_RANGE,
    SCAN_POINT_COUNT,
)
from core import scan as scan_mod

FORMAT_VERSION = 2
MAX_SLEEP = 1.0  # s; cap per-scan pacing so a long gap cannot stall shutdown


# --------------------------------------------------------------------- record
class RecordingLidarSource:
    """Wraps a LidarSource and captures every successful scan to .npz.

    Everything except ``doProcessSimple``, ``calibrate_background`` and
    ``close`` is delegated to the wrapped source, so this is transparent to
    consumers (including sim-only extras like ``object_pose``).
    """

    def __init__(
        self,
        source,
        path,
        *,
        note: str = "",
        kind: str = "unknown",
        flush_every: int = 60,
    ):
        self._source = source
        self.path = Path(path)
        self.note = note
        self.kind = kind
        self.flush_every = flush_every
        self._angles: list[np.ndarray] = []
        self._ranges: list[np.ndarray] = []
        self._stamps: list[int] = []
        self._bg: BackgroundTable | None = None
        self._closed = False
        self._lock = threading.Lock()
        atexit.register(self.close)

    # -- delegation ------------------------------------------------------
    def __getattr__(self, name: str):
        if name.startswith("__") and name.endswith("__"):
            raise AttributeError(name)
        return getattr(self._source, name)

    # -- capture ---------------------------------------------------------
    def doProcessSimple(self, scan: LaserScan) -> bool:
        ok = self._source.doProcessSimple(scan)
        if not ok:
            return False
        with self._lock:
            if self._closed:
                return True
            pts = scan.points[: scan.size]
            self._angles.append(
                np.fromiter((p.angle for p in pts), dtype=np.float32, count=len(pts))
            )
            self._ranges.append(
                np.fromiter((p.range for p in pts), dtype=np.float32, count=len(pts))
            )
            self._stamps.append(int(scan.stamp))
            if self.flush_every and len(self._stamps) % self.flush_every == 0:
                self._write()
        return True

    def calibrate_background(self) -> BackgroundTable:
        """Capture the wrapped source's background so replay can reuse it."""
        bg = self._source.calibrate_background()
        with self._lock:
            self._bg = bg
            self._write()
        return bg

    # -- bookkeeping -----------------------------------------------------
    @property
    def n_scans(self) -> int:
        with self._lock:
            return len(self._stamps)

    @property
    def wrapped(self):
        return self._source

    def close(self) -> None:
        """Flush and finalise the file. Idempotent; safe from atexit."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
            self._write()
        try:
            atexit.unregister(self.close)
        except Exception:
            pass

    # -- internals -------------------------------------------------------
    def _write(self) -> None:
        if not self._stamps or self.path is None:
            return
        sizes = np.asarray([a.size for a in self._angles], dtype=np.int32)
        max_points = int(sizes.max(initial=0))
        # Real X3 revolutions can have different sample counts.  Store a
        # rectangular, zero-padded matrix plus the per-scan sizes so replay
        # can emit every sample without asking NumPy to build a ragged array.
        angles = np.zeros((sizes.size, max_points), dtype=np.float32)
        ranges = np.zeros((sizes.size, max_points), dtype=np.float32)
        for i, (a, r) in enumerate(zip(self._angles, self._ranges)):
            n = int(a.size)
            angles[i, :n] = a
            ranges[i, :n] = r
        stamps = np.asarray(self._stamps, dtype=np.int64)
        # Collapse to a 1-D grid when every scan used the same angles
        # (the sim does; variable-resolution real scans stay 2-D).
        uniform = (
            sizes.size > 0
            and np.all(sizes == sizes[0])
            and np.all(angles == angles[0])
        )
        angles_out = angles[0] if uniform else angles
        meta = {
            "format": FORMAT_VERSION,
            "source_kind": self.kind,
            "note": self.note,
            "created": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "n_scans": int(stamps.size),
            "point_count": int(ranges.shape[1]),
            "variable_point_count": bool(not np.all(sizes == sizes[0])),
            "duration_s": float((stamps[-1] - stamps[0]) / 1e9) if stamps.size > 1 else 0.0,
        }
        arrays = {
            "ranges": ranges,
            "angles": angles_out,
            "sizes": sizes,
            "stamps": stamps,
            "meta": np.array(json.dumps(meta)),
        }
        if self._bg is not None:
            arrays["bg_angles"] = np.asarray(self._bg.angles, dtype=np.float64)
            arrays["bg_ranges"] = np.asarray(self._bg.ranges, dtype=np.float64)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.path.with_name(self.path.name + ".tmp")
        with open(tmp, "wb") as f:
            np.savez_compressed(f, **arrays)
        tmp.replace(self.path)


# --------------------------------------------------------------------- replay
class ReplayLidarSource(LidarSource):
    """Plays a recording back through the standard LidarSource surface.

    Paced by the recorded stamps so playback runs at the rate the data was
    actually captured. ``speed=0`` free-runs (no sleeps), which is what you
    want for an offline batch run. ``loop=False`` plays once and then reports
    ``exhausted`` so the scan loop can stop instead of spinning.
    """

    def __init__(
        self,
        scene: SceneConfig,
        noise: NoiseConfig,
        path,
        *,
        loop: bool = True,
        speed: float = 1.0,
    ):
        self.scene = scene
        self.noise = noise
        self.path = Path(path)
        self.loop = loop
        self.speed = float(speed)
        self._opts: dict[int, object] = {}
        self._on = False
        self._i = 0
        self._t0: float | None = None
        self._s0: int = 0
        self._bg: BackgroundTable | None = None
        self._load()

    # -- loading ---------------------------------------------------------
    def _load(self) -> None:
        if not self.path.exists():
            raise FileNotFoundError(f"replay file not found: {self.path}")
        with np.load(self.path, allow_pickle=False) as z:
            self._ranges = np.asarray(z["ranges"], dtype=np.float32)
            self._stamps = np.asarray(z["stamps"], dtype=np.int64)
            angles = np.asarray(z["angles"], dtype=np.float32)
            sizes = np.asarray(z["sizes"], dtype=np.int32) if "sizes" in z else None
            self.meta = json.loads(str(z["meta"].item())) if "meta" in z else {}
            if "bg_angles" in z and "bg_ranges" in z:
                self._bg = BackgroundTable(
                    np.asarray(z["bg_angles"], dtype=np.float64),
                    np.asarray(z["bg_ranges"], dtype=np.float64),
                )
        if self._ranges.ndim != 2 or self._stamps.ndim != 1:
            raise ValueError(f"malformed replay file: {self.path}")
        if self._ranges.shape[0] != self._stamps.shape[0]:
            raise ValueError(
                f"replay file scan/stamp mismatch: {self._ranges.shape[0]} vs {self._stamps.shape[0]}"
            )
        self._n = int(self._ranges.shape[0])
        self._pts = int(self._ranges.shape[1])
        if sizes is None:
            # Format 1 recordings were fixed-resolution and have no sizes
            # array; retain their original behavior.
            self._sizes = np.full(self._n, self._pts, dtype=np.int32)
        else:
            if sizes.ndim != 1 or sizes.shape[0] != self._n:
                raise ValueError(
                    f"replay file scan/size mismatch: {self._n} vs {sizes.shape[0] if sizes.ndim == 1 else sizes.shape}"
                )
            if np.any(sizes < 0) or np.any(sizes > self._pts):
                raise ValueError("replay file contains invalid scan sizes")
            self._sizes = sizes
        if angles.ndim == 1:
            self._angles_2d = None
            self._angles_1d = angles
        else:
            self._angles_2d = angles
            self._angles_1d = angles[0] if angles.shape[0] else angles
        if self._n == 0:
            raise ValueError(f"replay file has no scans: {self.path}")

    def _angle_row(self, i: int) -> np.ndarray:
        return self._angles_1d if self._angles_2d is None else self._angles_2d[i]

    # -- SWIG-mirroring surface -----------------------------------------
    def setlidaropt(self, prop: int, value) -> bool:
        self._opts[prop] = value
        return True

    def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]:
        v = self._opts.get(prop)
        if v is None:
            return False, 0
        try:
            return True, int(v)
        except (TypeError, ValueError):
            return False, 0

    def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]:
        v = self._opts.get(prop)
        if v is None:
            return False, False
        try:
            return True, bool(v)
        except (TypeError, ValueError):
            return False, False

    def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]:
        v = self._opts.get(prop)
        if v is None:
            return False, 0.0
        try:
            return True, float(v)
        except (TypeError, ValueError):
            return False, 0.0

    def getlidaropt_toString(self, prop: int) -> tuple[bool, str]:
        v = self._opts.get(prop)
        if v is None:
            return False, ""
        return True, str(v)

    def initialize(self) -> bool:
        return True

    def turnOn(self) -> bool:
        self._on = True
        self.seek(0)
        return True

    def turnOff(self) -> bool:
        self._on = False
        return True

    def disconnecting(self) -> None:
        self._on = False

    def doProcessSimple(self, scan: LaserScan) -> bool:
        """Emit the next recorded scan. False once exhausted (no loop)."""
        if not self._on:
            return False
        if self._i >= self._n:
            if not self.loop:
                return False
            self.seek(0)
        self._pace()
        self._fill(self._i, scan)
        self._i += 1
        return True

    # -- replay controls -------------------------------------------------
    @property
    def n_scans(self) -> int:
        return self._n

    @property
    def index(self) -> int:
        return self._i

    @property
    def exhausted(self) -> bool:
        """True when a non-looping replay has run to the end."""
        return (not self.loop) and self._i >= self._n

    @property
    def duration(self) -> float:
        if self._n < 2:
            return 0.0
        return float((self._stamps[-1] - self._stamps[0]) / 1e9)

    @property
    def scan_freq(self) -> float:
        """Median scan rate of the recording (Hz)."""
        d = np.diff(self._stamps)
        d = d[d > 0]
        if d.size == 0:
            return float(SCAN_FREQ)
        return float(1e9 / np.median(d))

    def seek(self, i: int = 0) -> None:
        """Jump to scan i and re-anchor the pacing clock."""
        self._i = max(0, min(int(i), self._n))
        self._t0 = None

    def reset(self, seed: int = 0) -> None:
        """'reset' control restarts playback from the first scan."""
        self.seek(0)

    # -- calibration -----------------------------------------------------
    def calibrate_background(self) -> BackgroundTable:
        """Prefer the background captured at record time.

        Falls back to a per-angle median over the first 30 scans, which is
        only correct if the recording starts with an empty board.
        """
        if self._bg is not None:
            return self._bg
        warnings.warn(
            f"{self.path.name} has no captured background; deriving one from the "
            "first 30 scans (only valid if the recording starts empty)",
            RuntimeWarning,
            stacklevel=2,
        )
        return self._fallback_background()

    def _fallback_background(self, k: int = 30) -> BackgroundTable:
        # Quantize variable-resolution samples to the canonical grid just as
        # real-device calibration does.  This keeps the fallback valid when
        # a recording has no captured background table.
        from core.scan import SCAN_ANGLE_INCREMENT, SCAN_MIN_ANGLE, SCAN_POINT_COUNT, scan_angles

        angles = np.concatenate([
            self._angle_row(i)[: int(self._sizes[i])] for i in range(min(k, self._n))
        ])
        ranges = np.concatenate([
            self._ranges[i, : int(self._sizes[i])] for i in range(min(k, self._n))
        ])
        valid = ranges > 0.0
        if not np.any(valid):
            return BackgroundTable(scan_angles(), np.zeros(SCAN_POINT_COUNT, dtype=float))
        bins = np.arange(SCAN_POINT_COUNT + 1) - 0.5
        edges = SCAN_MIN_ANGLE + bins * SCAN_ANGLE_INCREMENT
        idx = np.clip(np.digitize(angles[valid], edges) - 1, 0, SCAN_POINT_COUNT - 1)
        med = np.full(SCAN_POINT_COUNT, np.nan)
        for i in range(SCAN_POINT_COUNT):
            selected = ranges[valid][idx == i]
            if selected.size:
                med[i] = np.median(selected)
        grid = scan_angles()
        good = ~np.isnan(med)
        if good.sum() < 2:
            return BackgroundTable(grid, np.zeros(SCAN_POINT_COUNT, dtype=float))
        return BackgroundTable(grid, np.interp(grid, grid[good], med[good]))

    # -- internals -------------------------------------------------------
    def _pace(self) -> None:
        if self.speed <= 0.0:
            return
        s = int(self._stamps[self._i])
        if self._t0 is None:
            self._t0 = time.monotonic()
            self._s0 = s
            return
        target = self._t0 + max(0.0, (s - self._s0) / 1e9) / self.speed
        dt = target - time.monotonic()
        if dt > 0:
            time.sleep(min(dt, MAX_SLEEP))

    def _fill(self, i: int, scan: LaserScan) -> None:
        n = int(self._sizes[i])
        r = self._ranges[i, :n]
        a = self._angle_row(i)[:n]
        pts = scan.points
        if len(pts) != n:
            scan.points = [LaserPoint() for _ in range(n)]
            pts = scan.points
        for j in range(n):
            p = pts[j]
            p.angle = float(a[j])
            p.range = float(r[j])
            p.intensity = 0.0
        scan.size = n
        scan.stamp = int(self._stamps[i])
        scan.scanFreq = self.scan_freq
        scan.sampleRate = scan_mod.SCAN_SAMPLE_RATE
        scan.config.min_angle = SCAN_MIN_ANGLE
        scan.config.max_angle = SCAN_MAX_ANGLE
        scan.config.angle_increment = (SCAN_MAX_ANGLE - SCAN_MIN_ANGLE) / max(n - 1, 1)
        scan.config.min_range = SCAN_MIN_RANGE
        scan.config.max_range = SCAN_MAX_RANGE
        scan.moduleNum = scan_mod.MODULE_NUM_X3
        scan.envFlag = 0


# ------------------------------------------------------------------ standalone
def record(
    source,
    path,
    *,
    seconds: float | None = None,
    scans: int | None = None,
    note: str = "",
    kind: str = "unknown",
) -> Path:
    """Drive a source directly and write a recording. Used by the CLI and by
    tests to build fixtures without spinning up a server."""
    rec = RecordingLidarSource(source, path, note=note, kind=kind)
    if not rec.initialize():
        raise RuntimeError("source.initialize() failed")
    if not rec.turnOn():
        raise RuntimeError("source.turnOn() failed")
    # Capture the background the same way the server does, BEFORE the first
    # recorded scan. Without it a replay has to derive one from the recording
    # itself, which is wrong the moment the object is in frame at t=0.
    rec.calibrate_background()
    scan = LaserScan.blank()
    deadline = None if seconds is None else time.monotonic() + seconds
    n = 0
    try:
        while True:
            if scans is not None and n >= scans:
                break
            if deadline is not None and time.monotonic() >= deadline:
                break
            # rec.doProcessSimple pulls one scan from the wrapped source and
            # captures it in the same call - never call the source directly too.
            if rec.doProcessSimple(scan):
                n += 1
            elif scans is not None:
                time.sleep(0.01)
    finally:
        try:
            source.turnOff()
        except Exception:
            pass
        try:
            source.disconnecting()
        except Exception:
            pass
        rec.close()
    return Path(path)


def _main() -> None:
    import argparse

    from core.lidar_source import make_lidar_source

    p = argparse.ArgumentParser(description="Record lidar scans to a .npz file")
    p.add_argument("--lidar", choices=["sim", "real"], default="sim")
    p.add_argument("--out", required=True, help="output .npz path")
    p.add_argument("--board-size", type=float, default=2.0)
    p.add_argument("--seconds", type=float, default=10.0)
    p.add_argument("--scans", type=int, default=None)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--port-dev", default=None)
    p.add_argument("--note", default="")
    args = p.parse_args()

    scene = SceneConfig(board_size=args.board_size)
    noise = NoiseConfig()
    source = make_lidar_source(
        args.lidar, scene=scene, noise=noise, seed=args.seed, port=args.port_dev
    )
    out = record(
        source, args.out, seconds=args.seconds, scans=args.scans,
        note=args.note, kind=args.lidar,
    )
    with np.load(out, allow_pickle=False) as z:
        meta = json.loads(str(z["meta"].item()))
    print(
        f"wrote {out} — {meta['n_scans']} scans, "
        f"{meta['point_count']} pts, {meta['duration_s']:.2f} s"
    )


if __name__ == "__main__":
    _main()
