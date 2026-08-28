"""SimLidarSource: SWIG-mirroring CYdLidar twin with object motion and
raycast+noise inside doProcessSimple at a 10 Hz cadence.
"""
from __future__ import annotations

import time

import numpy as np

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable, LidarSource
from core.noise import apply_noise
from core.raycast import raycast_ranges
from core.scan import LaserScan, scan_angles
from core import scan as scan_mod


class SimLidarSource(LidarSource):
    """Duck-typed twin of ydlidar.CYdLidar (SWIG surface verified against the
    installed module). doProcessSimple blocks ~100 ms holding the 10 Hz
    cadence and internally does raycast + noise.
    """

    def __init__(
        self,
        scene: SceneConfig,
        noise: NoiseConfig,
        seed: int | None = None,
    ):
        self.scene = scene
        self.noise = noise
        self.rng = np.random.default_rng(seed)
        self._seed = seed
        self._object_pose = np.array([scene.board_size / 2.0, scene.board_size / 2.0])
        self._opts: dict[int, object] = {}
        self._scan_freq = scan_mod.SCAN_FREQ
        self._t_next = 0.0
        self._on = False

    # --- SWIG-mirroring surface -------------------------------------------
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
        self._t_next = 0.0
        return True

    def turnOff(self) -> bool:
        self._on = False
        return True

    def disconnecting(self) -> None:
        self._on = False

    def doProcessSimple(self, scan: LaserScan) -> bool:
        """Fill scan in place at ~10 Hz cadence; False on timeout (never)."""
        if not self._on:
            return False
        now = time.monotonic()
        if self._t_next == 0.0:
            self._t_next = now
        wait = self._t_next - now
        if wait > 0:
            time.sleep(wait)
        self._t_next = self._t_next + 1.0 / self._scan_freq
        self._fill(scan)
        return True

    # --- Sim extras --------------------------------------------------------
    def set_object_pose(self, xy_board: tuple[float, float]) -> None:
        self._object_pose = np.asarray(xy_board, dtype=float)

    @property
    def object_pose(self) -> tuple[float, float]:
        return float(self._object_pose[0]), float(self._object_pose[1])

    @property
    def scan_freq(self) -> float:
        return self._scan_freq

    def reset(self, seed: int) -> None:
        self._seed = seed
        self.rng = np.random.default_rng(seed)

    def calibrate_background(self) -> BackgroundTable:
        """Noise-off raycast of the empty scene (board + walls, no object)."""
        angles = scan_angles()
        ranges = self._raycast_empty(angles)
        return BackgroundTable(angles, ranges)

    # -- internals ----------------------------------------------------------
    def _raycast_empty(self, angles: np.ndarray) -> np.ndarray:
        """Raycast without the object circle (background)."""
        return raycast_ranges(angles, self.scene, object_center=None)

    def _fill(self, scan: LaserScan) -> None:
        angles = scan_angles()
        # (1) Angular jitter BEFORE raycast.
        jittered = angles + self.rng.normal(0.0, self.noise.angular_jitter_std, angles.shape)
        # (2) Raycast at jittered angles (object + empty scene).
        ranges = raycast_ranges(jittered, self.scene, object_center=tuple(self._object_pose))
        empty = self._raycast_empty(jittered)
        # second_closest for mixed pixels: use empty-scene range where the
        # object hit is closer, else no pull.
        second = np.where(empty > ranges, empty, ranges)
        r, _ = apply_noise(jittered, ranges, self.noise, self.rng, second_closest=second)

        stamp_ns = int(time.time_ns())
        pts = scan.points
        for i in range(scan_mod.SCAN_POINT_COUNT):
            p = pts[i]
            p.angle = float(jittered[i])
            p.range = float(r[i])
            p.intensity = 0.0
        scan.size = scan_mod.SCAN_POINT_COUNT
        scan.stamp = stamp_ns
        scan.scanFreq = self._scan_freq
        scan.sampleRate = scan_mod.SCAN_SAMPLE_RATE
        scan.config.min_angle = scan_mod.SCAN_MIN_ANGLE
        scan.config.max_angle = scan_mod.SCAN_MAX_ANGLE
        scan.config.angle_increment = scan_mod.SCAN_ANGLE_INCREMENT
        scan.config.min_range = scan_mod.SCAN_MIN_RANGE
        scan.config.max_range = scan_mod.SCAN_MAX_RANGE
        scan.moduleNum = scan_mod.MODULE_NUM_X3
        scan.envFlag = 0