"""RotatedLidarSource: applies a runtime-adjustable yaw rotation to every
scan, so the 90-degree board window can sit anywhere in the lidar's
360-degree view (the X3 sees everything; the game only uses one quadrant).

Semantics: `yaw` (radians) is the lidar-frame angle of the board frame's
zero direction. A raw point at angle th appears downstream at
wrap(th - yaw); the background table is rotated to match
(bg'(th') = bg(wrap(th' + yaw))). yaw = 0 reproduces the unrotated
pipeline bit-for-bit.
"""
from __future__ import annotations

import numpy as np

from core.lidar_source import BackgroundTable, LidarSource
from core.scan import LaserScan, scan_angles


def wrap_angle(x):
    """Wrap radians to [-pi, pi)."""
    return (np.asarray(x, dtype=float) + np.pi) % (2.0 * np.pi) - np.pi


class RotatedLidarSource(LidarSource):
    """Wraps any LidarSource and rotates its scans by -yaw.

    The raw (unrotated) background table from the first calibrate_background()
    is kept, so a yaw change only needs an instant numpy remap — no device
    re-calibration (the 30-scan capture stays valid, the window just turns).
    Everything the ABC does not declare (set_object_pose, object_pose, seek,
    exhausted, scan_freq, close, ...) delegates to the wrapped source via
    __getattr__, so capability detection keeps working.
    """

    def __init__(self, inner: LidarSource):
        self.inner = inner
        self.yaw = 0.0  # radians; plain float, set between scans
        self._raw_bg: BackgroundTable | None = None

    # -- rotation -----------------------------------------------------------
    def rotated_background(self) -> BackgroundTable:
        """Background table for the CURRENT yaw, remapped from the raw
        capture (instant, no device access)."""
        if self._raw_bg is None:
            raise RuntimeError("calibrate_background() has not been called")
        grid = scan_angles()
        # bg'(th') = bg(wrap(th' + yaw)), sampled on the canonical grid.
        return BackgroundTable(grid, self._raw_bg(wrap_angle(grid + self.yaw)))

    # -- LidarSource surface (delegated, with rotation where it matters) ----
    def setlidaropt(self, prop: int, value) -> bool:
        return self.inner.setlidaropt(prop, value)

    def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]:
        return self.inner.getlidaropt_toInt(prop)

    def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]:
        return self.inner.getlidaropt_toBool(prop)

    def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]:
        return self.inner.getlidaropt_toFloat(prop)

    def getlidaropt_toString(self, prop: int) -> tuple[bool, str]:
        return self.inner.getlidaropt_toString(prop)

    def initialize(self) -> bool:
        return self.inner.initialize()

    def turnOn(self) -> bool:
        return self.inner.turnOn()

    def doProcessSimple(self, scan: LaserScan) -> bool:
        ok = self.inner.doProcessSimple(scan)
        if ok:
            yaw = self.yaw
            if yaw != 0.0:
                for p in scan.points[: scan.size]:
                    p.angle = float(wrap_angle(p.angle - yaw))
        return ok

    def turnOff(self) -> bool:
        return self.inner.turnOff()

    def disconnecting(self) -> None:
        self.inner.disconnecting()

    def calibrate_background(self) -> BackgroundTable:
        self._raw_bg = self.inner.calibrate_background()
        return self.rotated_background()

    # explicit delegation for methods the ABC already defines as no-ops
    # (class-level lookup wins over __getattr__, so they must be spelled out)
    def set_object_pose(self, xy_board: tuple[float, float]) -> None:
        self.inner.set_object_pose(xy_board)

    # -- delegation of sim/replay/recording extras --------------------------
    def __getattr__(self, name):
        # only called for attributes NOT found on self
        return getattr(self.__dict__["inner"], name)
