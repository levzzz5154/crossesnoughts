"""Laser scan dataclasses mirroring the ydlidar SWIG field names, plus the
verified LidarProp enum (from YDLidar-SDK/core/common/ydlidar_def.h).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --- LidarProp enum, header-verified (YDLidar-SDK/core/common/ydlidar_def.h) ---
LidarPropSerialPort = 0
LidarPropIgnoreArray = 1
LidarPropSerialBaudrate = 10
LidarPropLidarType = 11
LidarPropDeviceType = 12
LidarPropSampleRate = 13
LidarPropAbnormalCheckCount = 14
LidarPropIntenstiyBit = 15
LidarPropMaxRange = 20
LidarPropMinRange = 21
LidarPropMaxAngle = 22
LidarPropMinAngle = 23
LidarPropScanFrequency = 24
LidarPropFixedResolution = 30
LidarPropReversion = 31
LidarPropInverted = 32
LidarPropAutoReconnect = 33
LidarPropSingleChannel = 34
LidarPropIntenstiy = 35
LidarPropSupportMotorDtrCtrl = 36
LidarPropSupportHeartBeat = 37

# LidarTypeID / DeviceTypeID (ydlidar_def.h)
TYPE_TOF = 0
TYPE_TRIANGLE = 1  # X3 / S2 / G-series triangular protocol
TYPE_TOF_NET = 2
TYPE_GS = 3
TYPE_SCL = 4
TYPE_SDM = 5
TYPE_SDM18 = 6
TYPE_TIA = 7

YDLIDAR_TYPE_SERIAL = 0x0

# X3 model code reported in LaserScan.moduleNum
MODULE_NUM_X3 = 6

# Canonical scan geometry: N=300 points, -pi..+pi inclusive, increment 2pi/299
# (SDK: FOV/(count-1), CYdLidar.cpp:634-635). (max-min)/inc + 1 == 300 exactly.
SCAN_POINT_COUNT = 300
SCAN_MIN_ANGLE = -np.pi
SCAN_MAX_ANGLE = np.pi
SCAN_ANGLE_INCREMENT = 2.0 * np.pi / (SCAN_POINT_COUNT - 1)
SCAN_FREQ = 10.0  # Hz
SCAN_SAMPLE_RATE = 3.0  # k samples/s
SCAN_MIN_RANGE = 0.10  # m
SCAN_MAX_RANGE = 8.0  # m


def scan_angles() -> np.ndarray:
    """Fixed inclusive angle grid: -pi + i*2pi/299, i = 0..299."""
    return SCAN_MIN_ANGLE + np.arange(SCAN_POINT_COUNT) * SCAN_ANGLE_INCREMENT


@dataclass
class LaserPoint:
    """One range sample, SWIG field names (angle in rad, CW-positive)."""

    angle: float = 0.0
    range: float = 0.0  # m; 0.0 = invalid
    intensity: float = 0.0  # X3 has no intensity


@dataclass
class LaserConfig:
    min_angle: float = SCAN_MIN_ANGLE
    max_angle: float = SCAN_MAX_ANGLE
    angle_increment: float = SCAN_ANGLE_INCREMENT
    time_increment: float = 0.0
    scan_time: float = 0.1
    min_range: float = SCAN_MIN_RANGE
    max_range: float = SCAN_MAX_RANGE


@dataclass
class LaserScan:
    stamp: int = 0  # ns of first point
    scanFreq: float = SCAN_FREQ
    sampleRate: float = SCAN_SAMPLE_RATE
    points: list = field(default_factory=list)  # list[LaserPoint]
    size: int = 0
    config: LaserConfig = field(default_factory=LaserConfig)
    moduleNum: int = MODULE_NUM_X3
    envFlag: int = 0

    @classmethod
    def blank(cls) -> "LaserScan":
        """A full 300-point scan with invalid (0-range) points, ready to fill."""
        return cls(
            points=[LaserPoint(angle=float(a), range=0.0) for a in scan_angles()],
            size=SCAN_POINT_COUNT,
        )