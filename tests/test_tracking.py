"""Tracker tests (plan §14 step 6, §17.3/4/9)."""
import numpy as np
import pytest

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable
from core.scan import LaserScan
from core.sim import SimLidarSource
from core.tracking import BG_THRESHOLD, Tracker


def _tracker(scene=None, seed=42):
    scene = scene or SceneConfig(board_size=2.0)
    noise = NoiseConfig()
    sim = SimLidarSource(scene, noise, seed=seed)
    sim.turnOn()
    tr = Tracker(scene, noise)
    tr.set_background(sim.calibrate_background())
    return sim, tr


def test_static_centroid_median_err():
    """Accuracy over a continuous path (<= 3 cm/scan motion, the tracker's
    gate design envelope): median < 2.5 cm, p95 < 5 cm."""
    sim, tr = _tracker()
    scan = LaserScan.blank()
    errs = []
    # slow sweep covering the board, ~2 cm/scan
    t = np.arange(120) * 0.1
    xs = 0.3 + 1.4 * t / 12.0
    ys = 0.3 + 1.4 * np.sin(np.pi * t / 12.0 * 0.5)
    for k, (x, y) in enumerate(zip(xs, ys)):
        sim.set_object_pose((x, y))
        sim.doProcessSimple(scan)
        res = tr.process(scan)
        if res is not None and k >= 3:  # skip acquisition transient
            errs.append(np.hypot(res.x - x, res.y - y))
    assert np.median(errs) < 0.025  # 2.5 cm
    assert np.percentile(errs, 95) < 0.05


def test_static_far_corner_accuracy():
    sim, tr = _tracker()
    sim.set_object_pose((1.9, 1.9))
    scan = LaserScan.blank()
    errs = []
    for _ in range(30):
        sim.doProcessSimple(scan)
        res = tr.process(scan)
        if res:
            errs.append(np.hypot(res.x - 1.9, res.y - 1.9))
    assert np.median(errs) < 0.03
    assert np.percentile(errs, 95) < 0.05


def test_track_never_lost_long():
    sim, tr = _tracker()
    scan = LaserScan.blank()
    # far-corner hold: object at (1.9, 1.9) for 60 s
    sim.set_object_pose((1.9, 1.9))
    max_miss = 0
    for _ in range(60):
        sim.doProcessSimple(scan)
        res = tr.process(scan)
        if res:
            max_miss = max(max_miss, res.missed)
    assert max_miss <= 3


def test_background_fp_rate_below_1pct():
    """Background false-positive rate < 1% per point with the 0.10 m
    threshold, across the board (S=2.0 worst case). The sim object is moved
    off-board (behind the near wall) so only noise creates FPs."""
    scene = SceneConfig(board_size=2.0)
    noise = NoiseConfig()
    sim = SimLidarSource(scene, noise, seed=11)
    sim.turnOn()
    sim.set_object_pose((-5.0, -5.0))  # occluded by the near wall
    tr = Tracker(scene, noise)
    tr.set_background(sim.calibrate_background())
    scan = LaserScan.blank()
    total = 0
    fp = 0
    for _ in range(50):
        sim.doProcessSimple(scan)
        angles = np.array([p.angle for p in scan.points], dtype=float)
        ranges = np.array([p.range for p in scan.points], dtype=float)
        valid = (ranges > noise.min_range) & (ranges <= noise.max_range)
        bg = tr._background(angles[valid])
        mask = np.abs(ranges[valid] - bg) > BG_THRESHOLD
        total += valid.sum()
        fp += mask.sum()
    assert fp / total < 0.01


def test_one_point_cluster_accepted_within_gate():
    """A 1-point cluster within the prediction gate is accepted."""
    scene = SceneConfig(board_size=2.0)
    noise = NoiseConfig(enabled=False)
    sim = SimLidarSource(scene, noise, seed=2)
    sim.turnOn()
    tr = Tracker(scene, noise)
    tr.set_background(sim.calibrate_background())
    scan = LaserScan.blank()
    sim.set_object_pose((1.0, 1.0))
    sim.doProcessSimple(scan)
    res = tr.process(scan)
    assert res is not None
    # force a single-point scan: only one angle echoes the object
    pts = scan.points
    for i, p in enumerate(pts):
        if abs(p.angle - np.pi / 4) > 0.01:
            p.range = 0.0
    res2 = tr.process(scan)
    assert res2 is not None  # accepted within gate


def test_mixed_pixel_rejection():
    """Mixed-pixel pulls are rejected by range-consistency."""
    scene = SceneConfig(board_size=2.0)
    noise = NoiseConfig(enabled=False)
    sim = SimLidarSource(scene, noise, seed=3)
    sim.turnOn()
    tr = Tracker(scene, noise)
    tr.set_background(sim.calibrate_background())
    scan = LaserScan.blank()
    sim.set_object_pose((1.0, 1.0))
    sim.doProcessSimple(scan)
    tr.process(scan)
    # inject a mixed-pixel pull: one silhouette bin jumps to the far wall
    pts = scan.points
    for p in pts:
        if abs(p.angle - np.pi / 4) < 0.02 and p.range > 1.2:
            p.range = 2.8  # pulled toward the wall
            break
    res = tr.process(scan)
    assert res is not None
    assert abs(res.x - 1.0) < 0.15
    assert abs(res.y - 1.0) < 0.15


def test_alpha_beta_tracks_motion():
    sim, tr = _tracker()
    scan = LaserScan.blank()
    sim.set_object_pose((1.0, 1.0))
    for _ in range(5):
        sim.doProcessSimple(scan)
        tr.process(scan)
    # move 10 cm toward the far corner
    sim.set_object_pose((1.05, 1.05))
    sim.doProcessSimple(scan)
    res = tr.process(scan)
    assert res is not None
    assert res.x > 1.0
    assert res.y > 1.0


def test_board_size_parameterization():
    for s in (1.0, 2.0):
        sim, tr = _tracker(SceneConfig(board_size=s))
        scan = LaserScan.blank()
        sim.set_object_pose((s / 2, s / 2))
        errs = []
        for k in range(15):
            sim.doProcessSimple(scan)
            res = tr.process(scan)
            if res is not None and k >= 3:
                errs.append(np.hypot(res.x - s / 2, res.y - s / 2))
        assert np.median(errs) < 0.03


# Positions where the old largest-cluster acquisition rule was observed to fail.
# (1.0, 1.0) and (0.333, 1.0) are the two reproduced cases at seed 5.
_ACQ_POSITIONS = [(1.0, 1.0), (0.333, 1.0), (1.2, 0.8), (1.667, 1.667)]


@pytest.mark.parametrize("seed", [5, 11, 23])
def test_acquisition_picks_the_object_not_its_mixed_pixel_fragment(seed):
    """Regression: acquisition used to take the LARGEST cluster and break size
    ties by angle order.

    Mixed pixels split one object into a clean fragment at the true range plus
    a fragment pulled halfway toward whatever lies behind it. The two fragments
    regularly tie in size, and on a tie the old code took whichever came first
    in angle order. At seed 5 that put the track ~0.7 m BEHIND an object
    sitting at (1.0, 1.0) -- and it then held that position with missed == 0
    and conf == 1.0 forever, because the real object was outside the 0.5 m gate
    and could never be re-associated.

    Acquisition now takes the NEAREST cluster, the same rule tracking uses:
    mixed-pixel pulls, wall echoes and board bleed-through all lie beyond the
    object. Single acquisition scan, so this pins the selection rule itself.
    """
    for pose in _ACQ_POSITIONS:
        sim, tr = _tracker(seed=seed)
        sim.set_object_pose(pose)
        tr.reset()
        scan = LaserScan.blank()
        sim.doProcessSimple(scan)
        res = tr.process(scan)
        assert res is not None, f"no acquisition at {pose} (seed {seed})"
        err = float(np.hypot(res.x - pose[0], res.y - pose[1]))
        assert err < 0.20, (
            f"acquired {err * 100:.1f} cm off at {pose} (seed {seed}); "
            "looks like a mixed-pixel fragment again"
        )


def test_static_object_converges_after_a_cold_acquisition():
    """The two positions that were 70 cm / 57 cm wrong must now converge to
    within a cell's worth of accuracy, not merely acquire in the right place.
    """
    for pose in [(1.0, 1.0), (0.333, 1.0)]:
        sim, tr = _tracker(seed=5)
        sim.set_object_pose(pose)
        tr.reset()
        scan = LaserScan.blank()
        res = None
        for _ in range(15):
            sim.doProcessSimple(scan)
            res = tr.process(scan)
        assert res is not None, f"lost the object at {pose}"
        err = float(np.hypot(res.x - pose[0], res.y - pose[1]))
        assert err < 0.05, f"converged {err * 100:.1f} cm off at {pose}"
        assert res.missed == 0, "a static object should never coast"