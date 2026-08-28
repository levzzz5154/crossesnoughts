"""LidarSource ABC + BackgroundTable + make_lidar_source factory.

The ABC mirrors the ydlidar.CYdLidar SWIG surface exactly (setlidaropt,
getlidaropt_to*, initialize, turnOn, doProcessSimple, turnOff, disconnecting)
plus the calibration abstraction: calibrate_background() is the ONLY way the
Tracker reads background, so real mode is defined (no raycast dependency).
"""
from __future__ import annotations

from abc import ABC, abstractmethod

import numpy as np

from core.config import NoiseConfig, SceneConfig
from core.scan import LaserScan


class LidarSource(ABC):
    """Duck-typed twin of ydlidar.CYdLidar's SWIG surface."""

    @abstractmethod
    def setlidaropt(self, prop: int, value) -> bool: ...

    @abstractmethod
    def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]: ...

    @abstractmethod
    def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]: ...

    @abstractmethod
    def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]: ...

    @abstractmethod
    def getlidaropt_toString(self, prop: int) -> tuple[bool, str]: ...

    @abstractmethod
    def initialize(self) -> bool: ...

    @abstractmethod
    def turnOn(self) -> bool: ...

    @abstractmethod
    def doProcessSimple(self, scan: LaserScan) -> bool:
        """Fill scan in place; True on success, False on timeout."""

    @abstractmethod
    def turnOff(self) -> bool: ...

    @abstractmethod
    def disconnecting(self) -> None: ...

    @abstractmethod
    def calibrate_background(self) -> "BackgroundTable":
        """Noise-off background ranges over the angle grid."""

    def set_object_pose(self, xy_board: tuple[float, float]) -> None:
        """Sim-only; real source no-ops."""


class BackgroundTable:
    """Background ranges indexed by angle, with linear interpolation.

    The ONLY way the Tracker reads background. Real scans have jittered,
    non-uniform angles, so interpolation is mandatory.
    """

    def __init__(self, angles: np.ndarray, ranges: np.ndarray):
        if angles.ndim != 1 or ranges.ndim != 1 or angles.size != ranges.size:
            raise ValueError("angles and ranges must be 1-D and same length")
        order = np.argsort(angles)
        self.angles = np.asarray(angles, dtype=float)[order]
        self.ranges = np.asarray(ranges, dtype=float)[order]

    def __call__(self, theta: np.ndarray) -> np.ndarray:
        theta = np.asarray(theta, dtype=float)
        return np.interp(theta, self.angles, self.ranges)


def make_lidar_source(
    kind: str,
    *,
    scene: SceneConfig,
    noise: NoiseConfig,
    seed: int | None = None,
    port: str | None = None,
    replay_path: str | None = None,
    replay_loop: bool = True,
    replay_speed: float = 1.0,
    record_path: str | None = None,
    record_note: str = "",
) -> LidarSource:
    """Factory: 'sim' -> SimLidarSource, 'real' -> RealLidarSource,
    'replay' -> ReplayLidarSource reading replay_path.

    record_path, when given, wraps the chosen source in a
    RecordingLidarSource that writes every scan to that .npz.
    """
    if kind == "sim":
        from core.sim import SimLidarSource

        source: LidarSource = SimLidarSource(scene, noise, seed=seed)
    elif kind == "real":
        from core.real import RealLidarSource

        source = RealLidarSource(scene, noise, port=port)
    elif kind == "replay":
        from core.recording import ReplayLidarSource

        if replay_path is None:
            raise ValueError("--replay-file is required when --lidar replay")
        return ReplayLidarSource(
            scene, noise, replay_path, loop=replay_loop, speed=replay_speed
        )
    else:
        raise ValueError(f"unknown lidar kind {kind!r} (expected 'sim', 'real' or 'replay')")

    if record_path is not None:
        from core.recording import RecordingLidarSource

        return RecordingLidarSource(source, record_path, note=record_note, kind=kind)
    return source