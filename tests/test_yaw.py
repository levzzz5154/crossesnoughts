"""RotatedLidarSource tests: angle rotation, background remap, end-to-end
tracking through a rotated window, delegation of sim extras."""
import numpy as np
import pytest

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import LidarSource
from core.scan import LaserPoint, LaserScan, scan_angles
from core.sim import SimLidarSource
from core.tracking import Tracker
from core.yaw import RotatedLidarSource, wrap_angle


class StubSource(LidarSource):
    """Deterministic source: always the same scan (no jitter)."""

    def __init__(self, angles, ranges):
        self._angles = list(angles)
        self._ranges = list(ranges)
        self.calls = 0

    def setlidaropt(self, prop, value): return True
    def getlidaropt_toInt(self, prop): return False, 0
    def getlidaropt_toBool(self, prop): return False, False
    def getlidaropt_toFloat(self, prop): return False, 0.0
    def getlidaropt_toString(self, prop): return False, ""
    def initialize(self): return True
    def turnOn(self): return True
    def turnOff(self): return True
    def disconnecting(self): return None
    def calibrate_background(self):
        from core.lidar_source import BackgroundTable
        return BackgroundTable(np.asarray(self._angles), np.asarray(self._ranges))

    def doProcessSimple(self, scan: LaserScan) -> bool:
        self.calls += 1
        scan.points = [
            LaserPoint(angle=a, range=r)
            for a, r in zip(self._angles, self._ranges)
        ]
        scan.size = len(scan.points)
        scan.stamp = 1_000_000_000 + self.calls
        return True


def test_wrap_angle():
    assert wrap_angle(0.0) == pytest.approx(0.0)
    assert wrap_angle(np.pi) == pytest.approx(-np.pi)
    assert wrap_angle(3 * np.pi / 2) == pytest.approx(-np.pi / 2)
    assert wrap_angle(-3 * np.pi) == pytest.approx(-np.pi)
    arr = wrap_angle(np.array([2 * np.pi, -2 * np.pi]))
    assert np.allclose(arr, 0.0, atol=1e-12)


def test_yaw_zero_is_identity():
    angles = scan_angles()
    src = RotatedLidarSource(StubSource(angles, np.full_like(angles, 2.0)))
    src.yaw = 0.0
    got = LaserScan.blank()
    assert src.doProcessSimple(got)
    for i in range(got.size):
        assert got.points[i].angle == pytest.approx(float(angles[i]))
        assert got.points[i].range == pytest.approx(2.0)


def test_yaw_rotates_angles():
    angles = scan_angles()
    yaw = np.pi / 2
    src = RotatedLidarSource(StubSource(angles, np.full_like(angles, 2.0)))
    src.yaw = yaw
    got = LaserScan.blank()
    assert src.doProcessSimple(got)
    for i in range(got.size):
        assert got.points[i].angle == pytest.approx(
            float(wrap_angle(angles[i] - yaw)), abs=1e-12
        )
        assert got.points[i].range == pytest.approx(2.0)


def _fast_sim(scene, seed=2):
    sim = SimLidarSource(scene, NoiseConfig(), seed=seed)
    sim._scan_freq = 500.0  # speed only; positions converge via alpha=0.6
    return sim


def test_background_remap_matches_raw():
    """rotated_bg(th') == raw_bg(wrap(th' + yaw)) for every grid angle."""
    scene = SceneConfig()
    sim = _fast_sim(scene, seed=1)
    src = RotatedLidarSource(sim)
    src.yaw = 1.234
    assert src.initialize() and src.turnOn()
    rotated = src.calibrate_background()
    raw = src._raw_bg
    grid = scan_angles()
    assert np.allclose(rotated(grid), raw(wrap_angle(grid + 1.234)), atol=1e-9)


def test_yaw_change_remaps_without_recalibration():
    """rotated_background() after a yaw change needs no new capture."""
    scene = SceneConfig()
    sim = _fast_sim(scene, seed=1)
    src = RotatedLidarSource(sim)
    assert src.initialize() and src.turnOn()
    src.calibrate_background()
    raw = src._raw_bg
    src.yaw = np.pi / 2
    remapped = src.rotated_background()
    grid = scan_angles()
    assert np.allclose(remapped(grid), raw(wrap_angle(grid + np.pi / 2)), atol=1e-9)


def test_tracking_through_rotated_window():
    """An object at sim-frame (1.5, 0.5) with yaw=+90deg is tracked at the
    rotated position (-0.5, 1.5) in the downstream board frame."""
    scene = SceneConfig(board_size=2.0)
    sim = _fast_sim(scene, seed=3)
    src = RotatedLidarSource(sim)
    src.yaw = np.pi / 2
    tracker = Tracker(scene, NoiseConfig())
    assert src.initialize() and src.turnOn()
    tracker.set_background(src.calibrate_background())
    sim.set_object_pose((1.5, 0.5))
    scan = LaserScan.blank()
    res = None
    for _ in range(30):
        assert src.doProcessSimple(scan)
        res = tracker.process(scan)
    assert res is not None
    assert res.x == pytest.approx(-0.5, abs=0.10)
    assert res.y == pytest.approx(1.5, abs=0.10)


def test_rotated_background_before_calibration_raises():
    src = RotatedLidarSource(_fast_sim(SceneConfig()))
    with pytest.raises(RuntimeError):
        src.rotated_background()


def test_delegation_of_sim_extras():
    scene = SceneConfig()
    sim = _fast_sim(scene, seed=4)
    src = RotatedLidarSource(sim)
    # capability detection keeps working through the wrapper
    assert hasattr(src, "set_object_pose") and hasattr(src, "object_pose")
    src.set_object_pose((0.5, 1.0))
    assert src.object_pose == (0.5, 1.0)
    assert src.scan_freq == sim.scan_freq
    # and set_object_pose reached the inner sim
    assert sim.object_pose == (0.5, 1.0)
