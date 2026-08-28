"""Raycast exactness vs analytic ground truth (noise-off, plan §17.2)."""
import numpy as np
import pytest

from core.config import SceneConfig
from core.raycast import raycast_ranges
from core.scan import (
    SCAN_ANGLE_INCREMENT,
    SCAN_MAX_ANGLE,
    SCAN_MIN_ANGLE,
    SCAN_POINT_COUNT,
)


@pytest.mark.parametrize("s", [1.0, 2.0])
def test_wall_at_known_distance(s):
    sc = SceneConfig(board_size=s)
    # theta=0 -> ray along +y_B from the lidar corner: grazes the board and
    # hits the back-left corner (0, S) at t=S
    r = raycast_ranges(np.array([0.0]), sc)
    assert abs(r[0] - s) < 1e-9
    # theta=pi/2 -> along +x_B: hits the front-right corner (S, 0) at t=S
    r = raycast_ranges(np.array([np.pi / 2]), sc)
    assert abs(r[0] - s) < 1e-9
    # theta=-pi/2 -> along -x_B: hits the left wall x=-1 at t=1
    r = raycast_ranges(np.array([-np.pi / 2]), sc)
    assert abs(r[0] - 1.0) < 1e-9
    # small negative theta (port side): misses the board, hits the far wall
    # y=S+1 at t=(S+1)/cos(theta)
    th = -0.05
    r = raycast_ranges(np.array([th]), sc)
    assert abs(r[0] - (s + 1.0) / np.cos(th)) < 1e-9


def test_object_moves_with_center():
    sc = SceneConfig(board_size=2.0)
    th = np.array([0.0])
    # object at (1, 1): ray theta=0 (along +y) passes at distance 1 from the
    # centre -> no occlusion; the corner hit (0, S) at t=S remains
    r = raycast_ranges(th, sc, object_center=(1.0, 1.0))
    assert abs(r[0] - 2.0) < 1e-9
    # object at (0.5, 0.0): the ray (along +y) passes at distance 0.5 from
    # the centre (> radius 0.1) -> no occlusion; corner hit remains
    r = raycast_ranges(th, sc, object_center=(0.5, 0.0))
    assert abs(r[0] - 2.0) < 1e-9
    # object ON the ray: centre (0.0, 0.5) -> nearest hit 0.5 - radius
    r = raycast_ranges(th, sc, object_center=(0.0, 0.5))
    assert abs(r[0] - (0.5 - sc.object_radius)) < 1e-9


def test_blind_spot_and_no_hit():
    sc = SceneConfig(board_size=2.0)
    # theta=-pi/2 -> along -x_B: hits left wall x=-1 at t=1
    r = raycast_ranges(np.array([-np.pi / 2]), sc)
    assert abs(r[0] - 1.0) < 1e-9
    # every angle hits something (board or box), so no 0.0 in the grid
    angles = np.linspace(-np.pi, np.pi, 361)
    r = raycast_ranges(angles, sc)
    assert np.all(r > 0.0)


def test_inclusive_grid_300():
    angles = np.linspace(SCAN_MIN_ANGLE, SCAN_MAX_ANGLE, SCAN_POINT_COUNT)
    assert angles.size == 300
    # (max - min) / inc + 1 == 300 exactly
    assert (SCAN_MAX_ANGLE - SCAN_MIN_ANGLE) / SCAN_ANGLE_INCREMENT + 1 == pytest.approx(300.0)
    assert angles[-1] == pytest.approx(SCAN_MAX_ANGLE)


def test_background_matches_raycast_empty():
    from core.sim import SimLidarSource
    from core.config import NoiseConfig

    sc = SceneConfig(board_size=2.0)
    sim = SimLidarSource(sc, NoiseConfig(enabled=False), seed=1)
    bg = sim.calibrate_background()
    angles = np.linspace(-np.pi, np.pi, 300)
    direct = raycast_ranges(angles, sc, object_center=None)
    interp = bg(angles)
    assert np.allclose(interp, direct, atol=1e-9)