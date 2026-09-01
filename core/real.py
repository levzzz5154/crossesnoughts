"""RealLidarSource: wraps ydlidar.CYdLidar (lazy import) behind the
LidarSource ABC. Calibrates background from N averaged empty-board scans.

The canonical option block (tri_test.py values, with variable resolution) is
applied in initialize() — without it the SDK initializes at its default
230400 baud and never sees this 115200 device.
"""
from __future__ import annotations

import time

import numpy as np

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable, LidarSource
from core import scan as scan_mod
from core.scan import (
    LaserPoint,
    LaserScan,
    LidarPropAbnormalCheckCount,
    LidarPropAutoReconnect,
    LidarPropDeviceType,
    LidarPropFixedResolution,
    LidarPropIgnoreArray,
    LidarPropIntenstiy,
    LidarPropIntenstiyBit,
    LidarPropInverted,
    LidarPropLidarType,
    LidarPropMaxAngle,
    LidarPropMaxRange,
    LidarPropMinAngle,
    LidarPropMinRange,
    LidarPropReversion,
    LidarPropSampleRate,
    LidarPropScanFrequency,
    LidarPropSerialBaudrate,
    LidarPropSerialPort,
    LidarPropSingleChannel,
    LidarPropSupportHeartBeat,
    LidarPropSupportMotorDtrCtrl,
    TYPE_TRIANGLE,
    YDLIDAR_TYPE_SERIAL,
)
from core.scan import scan_angles

_N_AVG = 30  # calibration scans

# Baudrate of the X3 (Dataset.md row "X3/X3 Pro" — NOT the SDK default 230400).
X3_BAUDRATE = 115200


def _iter_points(swig_scan):
    """Yield (angle, range, intensity) for every point the SDK actually holds.

    The X3's motor speed varies slightly, so a revolution can contain 333,
    334, or 335 samples even when the requested scan frequency is 10 Hz.
    The real source therefore runs the SDK in variable-resolution mode.  The
    buffer length remains the final guard against an SDK size/buffer mismatch
    so one bad frame cannot take the acquisition loop down.
    """
    n = min(int(swig_scan.size), len(swig_scan.points))
    for i in range(n):
        p = swig_scan.points[i]
        yield float(p.angle), float(p.range), float(p.intensity)


def canonical_option_block(port: str) -> dict[int, object]:
    """The tri_test.py option block for the X3.

    Fixed resolution is intentionally disabled: the SDK's fixed-size buffer
    is derived from an assumed motor speed and can be smaller than the real
    point count when the motor runs a little faster.
    """
    return {
        LidarPropSerialPort: port,
        LidarPropSerialBaudrate: X3_BAUDRATE,
        LidarPropLidarType: TYPE_TRIANGLE,
        LidarPropDeviceType: YDLIDAR_TYPE_SERIAL,
        LidarPropScanFrequency: 10.0,
        LidarPropSampleRate: 3,
        LidarPropSingleChannel: True,
        LidarPropAbnormalCheckCount: 4,
        LidarPropSupportMotorDtrCtrl: True,
        LidarPropFixedResolution: False,
        LidarPropMaxAngle: 180.0,
        LidarPropMinAngle: -180.0,
        LidarPropMaxRange: 16.0,
        LidarPropMinRange: 0.08,
        LidarPropIntenstiy: False,
    }


class RealLidarSource(LidarSource):
    """Real X3 via the installed ydlidar SWIG module (lazy import).

    The canonical option block mirrors tri_test.cpp with variable-resolution
    scans enabled. doProcessSimple copies the SWIG scan into the caller's
    duck-typed LaserScan so consumers never touch SWIG types.
    """

    def __init__(
        self,
        scene: SceneConfig,
        noise: NoiseConfig,
        port: str | None = None,
        n_avg: int = _N_AVG,
    ):
        self.scene = scene
        self.noise = noise
        self.port = port or "/dev/ttyUSB0"
        self.n_avg = n_avg
        self._ydlidar = None
        self._laser = None
        self._swig_scan = None
        # Canonical block, overridable via setlidaropt before initialize().
        self._opts: dict[int, object] = canonical_option_block(self.port)
        self._calibrated = False

    # --- lazy import -------------------------------------------------------
    def _import(self):
        if self._ydlidar is None:
            import ydlidar  # lazy: sim runs without it

            self._ydlidar = ydlidar
            self._laser = ydlidar.CYdLidar()
            self._swig_scan = ydlidar.LaserScan()

    def _apply_options(self) -> None:
        """Push the option block into the SWIG laser before initialize()."""
        for prop, value in self._opts.items():
            self._laser.setlidaropt(prop, value)

    # -- SWIG-mirroring surface -------------------------------------------
    def setlidaropt(self, prop: int, value) -> bool:
        self._opts[prop] = value
        if self._laser is not None:
            return bool(self._laser.setlidaropt(prop, value))
        return True  # applied later, in initialize()

    def getlidaropt_toInt(self, prop: int) -> tuple[bool, int]:
        self._import()
        ok, v = self._laser.getlidaropt_toInt(prop)
        return bool(ok), int(v)

    def getlidaropt_toBool(self, prop: int) -> tuple[bool, bool]:
        self._import()
        ok, v = self._laser.getlidaropt_toBool(prop)
        return bool(ok), bool(v)

    def getlidaropt_toFloat(self, prop: int) -> tuple[bool, float]:
        self._import()
        ok, v = self._laser.getlidaropt_toFloat(prop)
        return bool(ok), float(v)

    def getlidaropt_toString(self, prop: int) -> tuple[bool, str]:
        self._import()
        ok, v = self._laser.getlidaropt_toString(prop)
        return bool(ok), str(v)

    def initialize(self) -> bool:
        self._import()
        y = self._ydlidar
        y.os_init()
        self._apply_options()
        ok = bool(self._laser.initialize())
        if not ok:
            y.os_shutdown()
        return ok

    def turnOn(self) -> bool:
        self._import()
        return bool(self._laser.turnOn())

    def doProcessSimple(self, scan: LaserScan) -> bool:
        """Fill scan in place from the real device (blocks up to 1 s).

        The caller passes our duck-typed LaserScan; internally we use the
        SWIG LaserScan and copy the fields across (the SWIG module rejects
        foreign scan objects).
        """
        self._import()
        ok = bool(self._laser.doProcessSimple(self._swig_scan))
        if not ok:
            return False
        src = self._swig_scan
        pts = [
            LaserPoint(angle=a, range=r, intensity=i)
            for a, r, i in _iter_points(src)
        ]
        scan.stamp = int(src.stamp)
        scan.scanFreq = float(src.scanFreq)
        scan.sampleRate = float(src.sampleRate)
        scan.size = len(pts)
        scan.points = pts
        # The SDK reports moduleNum == 0 for the X3; consumers expect 6.
        scan.moduleNum = scan_mod.MODULE_NUM_X3
        return True

    def turnOff(self) -> bool:
        self._import()
        return bool(self._laser.turnOff())

    def disconnecting(self) -> None:
        self._import()
        self._laser.disconnecting()
        self._ydlidar.os_shutdown()

    def set_object_pose(self, xy_board: tuple[float, float]) -> None:
        pass  # sim-only; real source no-ops

    # -- calibration --------------------------------------------------------
    def calibrate_background(self) -> BackgroundTable:
        """N-averaged empty-board scans, median per quantized angle bin,
        then interpolated (real angles are jittered/non-uniform)."""
        self._import()
        y = self._ydlidar
        scan = y.LaserScan()
        angles: list[float] = []
        ranges: list[float] = []
        for _ in range(self.n_avg):
            if self._laser.doProcessSimple(scan):
                for a, r, _intensity in _iter_points(scan):
                    angles.append(a)
                    ranges.append(r)
            time.sleep(0.05)
        if not angles:
            raise RuntimeError("no scans captured for background calibration")
        theta = np.asarray(angles)
        r = np.asarray(ranges)
        ok = r > 0.0
        if ok.sum() < 10:
            raise RuntimeError("too few valid background points")
        theta, r = theta[ok], r[ok]
        # Quantize to the fixed grid, median per bin.
        bins = np.arange(scan_mod.SCAN_POINT_COUNT + 1) - 0.5
        edges = scan_mod.SCAN_MIN_ANGLE + bins * scan_mod.SCAN_ANGLE_INCREMENT
        idx = np.clip(np.digitize(theta, edges) - 1, 0, scan_mod.SCAN_POINT_COUNT - 1)
        med = np.full(scan_mod.SCAN_POINT_COUNT, np.nan)
        for i in range(scan_mod.SCAN_POINT_COUNT):
            sel = r[idx == i]
            if sel.size:
                med[i] = np.median(sel)
        grid = scan_angles()
        # Fill gaps by interpolation over valid bins.
        valid = ~np.isnan(med)
        if valid.sum() < 2:
            raise RuntimeError("too few valid background bins")
        filled = np.interp(grid, grid[valid], med[valid])
        self._scan_was_calibrated = True
        return BackgroundTable(grid, filled)
