"""Swap-in parity tests (plan §8, §17.6): ABC surface vs installed ydlidar."""
import numpy as np
import pytest

from core.lidar_source import BackgroundTable, LidarSource, make_lidar_source
from core.scan import (
    LidarPropFixedResolution,
    LidarPropScanFrequency,
    SCAN_POINT_COUNT,
)
from core.sim import SimLidarSource

SWIG_METHODS = [
    "setlidaropt",
    "getlidaropt_toInt",
    "getlidaropt_toBool",
    "getlidaropt_toFloat",
    "getlidaropt_toString",
    "initialize",
    "turnOn",
    "doProcessSimple",
    "turnOff",
    "disconnecting",
]


def test_abc_surface_parity_with_swig():
    import ydlidar

    for m in SWIG_METHODS:
        assert hasattr(ydlidar.CYdLidar, m), f"SWIG missing {m}"
        assert hasattr(SimLidarSource, m), f"Sim missing {m}"
        assert m in LidarSource.__abstractmethods__ or hasattr(LidarSource, m)


def test_factory_kinds():
    from core.config import NoiseConfig, SceneConfig

    scene = SceneConfig()
    noise = NoiseConfig()
    sim = make_lidar_source("sim", scene=scene, noise=noise, seed=1)
    assert isinstance(sim, SimLidarSource)
    with pytest.raises(ValueError):
        make_lidar_source("bogus", scene=scene, noise=noise)


def test_background_table_interpolation_roundtrip():
    angles = np.linspace(-np.pi, np.pi, 300)
    ranges = 2.0 + 0.5 * np.cos(angles)
    bt = BackgroundTable(angles, ranges)
    # query at shifted angles -> interpolated
    q = angles + 0.001
    got = bt(q)
    assert np.allclose(got, 2.0 + 0.5 * np.cos(q), atol=1e-4)
    # jittered angles (real-scenario)
    rng = np.random.default_rng(0)
    q2 = angles + rng.normal(0, 0.01, angles.shape)
    got2 = bt(q2)
    assert np.all(np.isfinite(got2))


def test_canonical_option_block():
    sim = SimLidarSource.__new__(SimLidarSource)  # avoid scene init
    # The canonical block is exercised by real.py; here we pin the constants
    # that real.py uses (tri_test.cpp values + FixedResolution fix).
    from core import real

    assert real.TYPE_TRIANGLE == 1
    assert real.YDLIDAR_TYPE_SERIAL == 0
    # FixedResolution must be set in the canonical block
    import inspect

    src = inspect.getsource(real)
    assert "LidarPropFixedResolution" in src


def test_scan_size_formula():
    # (max - min) / inc + 1 == len(points) == 300
    from core.scan import SCAN_ANGLE_INCREMENT, SCAN_MAX_ANGLE, SCAN_MIN_ANGLE

    n = int((SCAN_MAX_ANGLE - SCAN_MIN_ANGLE) / SCAN_ANGLE_INCREMENT) + 1
    assert n == SCAN_POINT_COUNT == 300


def test_real_mode_tracker_path_with_recorded_scan():
    """Real-mode tracker path: a recorded scan + captured-background
    calibration drives the same Tracker code as sim (zero consumer changes)."""
    from core.config import NoiseConfig, SceneConfig
    from core.scan import LaserPoint, LaserScan
    from core.tracking import Tracker

    scene = SceneConfig(board_size=2.0)
    noise = NoiseConfig()
    tr = Tracker(scene, noise)

    # synthetic "recorded" background: empty board + walls, no object
    sim = SimLidarSource(scene, NoiseConfig(enabled=False), seed=1)
    tr.set_background(sim.calibrate_background())

    # synthetic recorded scan with an object echo (angles jittered like real)
    rng = np.random.default_rng(3)
    angles = np.linspace(-np.pi, np.pi, 300) + rng.normal(0, 0.002, 300)
    scan = LaserScan()
    scan.points = [LaserPoint(angle=float(a)) for a in angles]
    scan.size = 300
    scan.stamp = 1_000_000_000
    # object at (1.0, 1.0): fill the arc around pi/4 with ~1.31 m hits
    for i, a in enumerate(angles):
        if abs(a - np.pi / 4) < 0.05:
            scan.points[i].range = 1.31
        else:
            scan.points[i].range = 0.0  # invalid
    res = tr.process(scan)
    assert res is not None
    assert abs(res.x - 1.0) < 0.3
    assert abs(res.y - 1.0) < 0.3