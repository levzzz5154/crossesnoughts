"""Coordinate frame tests (plan §14 step 2, §4 pinned examples)."""
import numpy as np
import pytest

from core import transforms as tf
from core.config import SceneConfig


@pytest.mark.parametrize("s", [1.0, 2.0, 3.0])
def test_polar_board_roundtrip(s):
    rng = np.random.default_rng(0)
    for _ in range(50):
        r = rng.uniform(0.1, 4.0)
        th = rng.uniform(-np.pi, np.pi)
        x, y = tf.polar_to_board(np.array([r]), np.array([th]))
        r2, th2 = tf.board_to_polar(x, y)
        assert abs(r2[0] - r) < 1e-12
        # angle equivalence mod 2pi
        d = abs(th2[0] - th) % (2 * np.pi)
        assert d < 1e-12 or abs(d - 2 * np.pi) < 1e-12


def test_theta_zero_is_forward():
    x, y = tf.polar_to_board(np.array([1.0]), np.array([0.0]))
    assert x[0] == pytest.approx(0.0, abs=1e-12)
    assert y[0] == pytest.approx(1.0, abs=1e-12)  # straight into the board


def test_theta_plus_pi2_starboard_along_front_edge():
    x, y = tf.polar_to_board(np.array([1.0]), np.array([np.pi / 2]))
    assert x[0] == pytest.approx(1.0, abs=1e-12)
    assert y[0] == pytest.approx(0.0, abs=1e-12)


def test_theta_minus_pi2_port_off_board():
    x, y = tf.polar_to_board(np.array([1.0]), np.array([-np.pi / 2]))
    assert x[0] == pytest.approx(-1.0, abs=1e-12)
    assert y[0] == pytest.approx(0.0, abs=1e-12)


@pytest.mark.parametrize("s", [1.0, 2.0, 3.0])
def test_board_center_angle_plus_pi4(s):
    # sign-fix pin: atan2(x_B, y_B) = +pi/4 for the board centre, any S
    r, th = tf.board_to_polar(np.array([s / 2]), np.array([s / 2]))
    assert th[0] == pytest.approx(np.pi / 4, abs=1e-12)
    assert r[0] == pytest.approx(s * np.sqrt(2) / 2, abs=1e-12)


def test_board_to_world_center_and_lidar():
    s = 2.0
    xw, zw = tf.board_to_world(np.array([s / 2]), np.array([s / 2]), s)
    assert xw[0] == pytest.approx(0.0)
    assert zw[0] == pytest.approx(0.0)
    xw, zw = tf.board_to_world(np.array([0.0]), np.array([0.0]), s)
    assert xw[0] == pytest.approx(-s / 2)
    assert zw[0] == pytest.approx(-s / 2)


def test_lidar_to_board_identity_swap():
    xb, yb = tf.lidar_to_board(np.array([1.0]), np.array([2.0]))
    assert xb[0] == pytest.approx(2.0)
    assert yb[0] == pytest.approx(1.0)


def test_world_board_roundtrip():
    s = 1.5
    xw, zw = tf.board_to_world(np.array([0.3]), np.array([1.2]), s)
    xb, yb = tf.world_to_board(xw, zw, s)
    assert xb[0] == pytest.approx(0.3)
    assert yb[0] == pytest.approx(1.2)


def test_ray_dir_board_matches_polar():
    th = np.array([0.3, 1.2, -0.8])
    dx, dy = tf.ray_dir_board(th)
    x, y = tf.polar_to_board(np.ones(3), th)
    assert np.allclose(dx, x)
    assert np.allclose(dy, y)