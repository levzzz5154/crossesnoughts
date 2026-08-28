"""RealLidarSource: wraps ydlidar.CYdLidar (lazy import) behind the
LidarSource ABC. Calibrates background from N averaged empty-board scans.
"""
from __future__ import annotations

import time

import numpy as np

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable, LidarSource
from core import scan as scan_mod
from core.scan import (
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


class RealLidarSource(LidarSource):
    """Real X3 via the installed ydlidar SWIG module (lazy import).

    Canonical option block mirrors tri_test.cpp + the FixedResolution fix
    (without it the real SDK emits variable-size scans).
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
        self._opts: dict[int, object] = {}
        self._calibrated = False

    # --- lazy import -------------------------------------------------------
    def _import(self):
        if self._ydlidar is None:
            import ydlidar  # lazy: sim runs without it

            self._ydlidar = ydlidar
            self._laser = ydlidar.CYdLidar()

    # -- SWIG-mirroring surface -------------------------------------------
    def setlidaropt(self, prop: int, value) -> bool:
        self._opts[prop] = value
        self._import()
        return bool(self._laser.setlidaropt(prop, value))

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
        ok = bool(self._laser.initialize())
        if not ok:
            y.os_shutdown()
        return ok

    def turnOn(self) -> bool:
        self._import()
        return bool(self._laser.turnOn())

    def doProcessSimple(self, scan: LaserScan) -> bool:
        """Fill scan in place from the real device (blocks up to 1 s)."""
        self._import()
        ok = bool(self._laser.doProcessSimple(scan))
        if ok:
            scan.moduleNum = scan_mod.MODULE_NUM_X3
        return ok

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
                for i in range(scan.size):
                    angles.append(float(scan.points[i].angle))
                    ranges.append(float(scan.points[i].range))
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