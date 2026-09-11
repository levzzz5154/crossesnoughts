import math

import pytest

from core.transforms import BoardAlignment
from game.auto_calibration import solve_three_corners, undo_current_transform


def _placed_board(origin, size, yaw_deg):
    yaw = math.radians(yaw_deg)
    ex = (math.cos(yaw), -math.sin(yaw))
    ey = (math.sin(yaw), math.cos(yaw))
    o = origin
    return [(o[0] + size * ex[0], o[1] + size * ex[1]),
            (o[0] + size * (ex[0] + ey[0]), o[1] + size * (ex[1] + ey[1])),
            (o[0] + size * ey[0], o[1] + size * ey[1])]


def test_three_corner_solver_recovers_square_pose():
    points = _placed_board((0.2, 0.35), 1.4, 37.0)
    size, yaw, ox, oy = solve_three_corners(*points)
    assert size == pytest.approx(1.4)
    assert yaw == pytest.approx(37.0)
    expected_x = .2 * math.cos(math.radians(37)) - .35 * math.sin(math.radians(37))
    expected_y = .35 * math.cos(math.radians(37)) + .2 * math.sin(math.radians(37))
    assert (ox, oy) == pytest.approx((expected_x, expected_y))


def test_bad_corner_geometry_is_rejected():
    with pytest.raises(ValueError):
        solve_three_corners((1, 0), (2, 0), (3, 0))


def test_undo_current_transform_round_trip():
    alignment = BoardAlignment(.2, .3, True, False)
    raw = (1.1, .7)
    yaw = math.radians(25)
    rotated = (raw[0] * math.cos(yaw) - raw[1] * math.sin(yaw),
               raw[1] * math.cos(yaw) + raw[0] * math.sin(yaw))
    shown = alignment.to_board(*rotated, 1.5)
    assert undo_current_transform(shown, 1.5, 25, alignment) == pytest.approx(raw)
