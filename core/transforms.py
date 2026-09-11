"""Pure numpy coordinate transforms.

Convention: theta=0 is forward, angles increase CLOCKWISE viewed from top;
ray direction d(theta) = (cos theta, sin theta) in the lidar frame
(SDK ydlidar_def.h:130-133).

Frames:
  L (lidar): origin at lidar rotation centre on the scan plane; +X_L forward,
             +Y_L starboard (90 deg CW from forward).
  B (board): origin at the board FRONT-RIGHT corner (the lidar mount corner);
             +X_B along the front edge away from the lidar, +Y_B into the
             board. Footprint x_B,y_B in [0, S].
  W (world): right-handed, origin at box floor centre; +X_W east, +Y_W up,
             +Z_W south. Board centred at origin spanning x,z in [-S/2, S/2].

L->B is the identity swap (no sign flip): x_B = y_L, y_B = x_L.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.config import SceneConfig


def polar_to_board(r: np.ndarray, theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Polar (r, theta) -> board (x_B = r sin theta, y_B = r cos theta)."""
    return r * np.sin(theta), r * np.cos(theta)


def board_to_polar(x: np.ndarray, y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Board (x_B, y_B) -> (r, theta), theta in [-pi, pi], CW-positive."""
    return np.hypot(x, y), np.arctan2(x, y)


def lidar_to_board(x_l: np.ndarray, y_l: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Lidar frame -> board frame: identity swap, no sign flip."""
    return np.asarray(y_l), np.asarray(x_l)


def board_to_world(x_b: np.ndarray, y_b: np.ndarray, s: float) -> tuple[np.ndarray, np.ndarray]:
    """Board -> world (translation only): x_W = x_B - S/2, z_W = y_B - S/2."""
    return np.asarray(x_b) - s / 2.0, np.asarray(y_b) - s / 2.0


def world_to_board(x_w: np.ndarray, z_w: np.ndarray, s: float) -> tuple[np.ndarray, np.ndarray]:
    return np.asarray(x_w) + s / 2.0, np.asarray(z_w) + s / 2.0


def ray_dir_board(theta: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Ray direction in BOARD frame for angle theta: d_B = (sin theta, cos theta)."""
    return np.sin(theta), np.cos(theta)

@dataclass(frozen=True)
class BoardAlignment:
    """Board origin in yaw-aligned lidar axes (metres), then center mirrors."""

    offset_x: float = 0.0
    offset_y: float = 0.0
    flip_horizontal: bool = False
    flip_vertical: bool = False

    def to_board(self, x, y, size):
        x, y = x - self.offset_x, y - self.offset_y
        return (size - x if self.flip_horizontal else x,
                size - y if self.flip_vertical else y)

    def from_board(self, x, y, size):
        x = size - x if self.flip_horizontal else x
        y = size - y if self.flip_vertical else y
        return x + self.offset_x, y + self.offset_y
