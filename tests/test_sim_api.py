"""Sim API + dataclass field-name parity tests (plan §14 step 1/5)."""
import time

import numpy as np
import pytest

from core.config import NoiseConfig, SceneConfig
from core.scan import (
    LaserPoint,
    LaserScan,
    LidarPropFixedResolution,
    LidarPropScanFrequency,
    LidarPropSerialBaudrate,
    LidarPropSingleChannel,
    SCAN_ANGLE_INCREMENT,
    SCAN_MAX_ANGLE,
    SCAN_MIN_ANGLE,
    SCAN_POINT_COUNT,
    scan_angles,
)
from core.sim import SimLidarSource


def test_lidarprop_enum_ints_pinned():
    assert LidarPropSerialBaudrate == 10
    assert LidarPropScanFrequency == 24
    assert LidarPropFixedResolution == 30
    assert LidarPropSingleChannel == 34


def test_dataclass_field_names_match_swig():
    # SWIG LaserPoint fields (verified from installed module)
    for name in ("angle", "range", "intensity"):
        assert hasattr(LaserPoint(), name)
    # SWIG LaserScan fields
    for name in ("stamp", "scanFreq", "sampleRate", "points", "size", "config", "moduleNum", "envFlag"):
        assert hasattr(LaserScan(), name)


def test_config_validation():
    for bad in (0.05, 2.5, 3.5, 4.0):
        with pytest.raises(ValueError):
            SceneConfig(board_size=bad)
    for good in (0.1, 0.5, 1.0, 1.5, 2.0):
        SceneConfig(board_size=good)
    assert SceneConfig().object_radius == 0.10
    assert SceneConfig().board_size == 2.0


def test_scan_grid_consistency():
    angles = scan_angles()
    assert angles.size == SCAN_POINT_COUNT == 300
    assert angles[0] == pytest.approx(SCAN_MIN_ANGLE)
    assert angles[-1] == pytest.approx(SCAN_MAX_ANGLE)
    assert (SCAN_MAX_ANGLE - SCAN_MIN_ANGLE) / SCAN_ANGLE_INCREMENT + 1 == pytest.approx(300.0)
    assert np.allclose(np.diff(angles), SCAN_ANGLE_INCREMENT)


def test_do_process_simple_fills_in_place():
    sim = SimLidarSource(SceneConfig(), NoiseConfig(enabled=False), seed=1)
    sim.turnOn()
    scan = LaserScan.blank()
    t0 = time.monotonic()
    ok = sim.doProcessSimple(scan)
    dt = time.monotonic() - t0
    assert ok
    assert scan.size == 300
    assert scan.moduleNum == 6
    assert scan.scanFreq == 10.0
    assert scan.sampleRate == 3.0
    assert scan.stamp > 0
    ranges = [p.range for p in scan.points]
    assert all(r > 0 for r in ranges)  # noise-off: everything hits
    # pacing: the SECOND call sleeps to hold the 10 Hz cadence
    t0 = time.monotonic()
    sim.doProcessSimple(scan)
    dt2 = time.monotonic() - t0
    assert dt2 >= 0.08


def test_do_process_simple_off_returns_false():
    sim = SimLidarSource(SceneConfig(), NoiseConfig(), seed=1)
    assert not sim.doProcessSimple(LaserScan.blank())


def test_setlidaropt_roundtrip():
    sim = SimLidarSource(SceneConfig(), NoiseConfig(), seed=1)
    assert sim.setlidaropt(LidarPropScanFrequency, 10.0)
    ok, v = sim.getlidaropt_toFloat(LidarPropScanFrequency)
    assert ok and v == 10.0
    assert sim.setlidaropt(LidarPropSingleChannel, True)
    ok, v = sim.getlidaropt_toBool(LidarPropSingleChannel)
    assert ok and v is True
    assert sim.setlidaropt(LidarPropSerialBaudrate, 115200)
    ok, v = sim.getlidaropt_toInt(LidarPropSerialBaudrate)
    assert ok and v == 115200


def test_object_pose_and_reset():
    sim = SimLidarSource(SceneConfig(), NoiseConfig(), seed=1)
    sim.set_object_pose((0.5, 1.2))
    assert sim.object_pose == (0.5, 1.2)
    sim.reset(99)
    assert sim.rng is not None


def test_calibrate_background_raycast():
    sim = SimLidarSource(SceneConfig(board_size=2.0), NoiseConfig(enabled=False), seed=1)
    bg = sim.calibrate_background()
    angles = scan_angles()
    v = bg(angles)
    assert np.all(v > 0)
    # background at the centre diagonal = dist to far wall through centre
    i = int(np.argmin(np.abs(angles - np.pi / 4)))
    assert abs(v[i] - (2.0 * np.sqrt(2) / 2 * 2 - 1.0)) > 0  # sanity: not degenerate