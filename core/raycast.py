"""Vectorized raycasting against board edges, box walls, and the object circle.

All primitives live in the BOARD frame. Rays originate at the lidar (board
origin, the front-right corner). For angle theta the ray direction in board
frame is d_B(theta) = (sin theta, cos theta).

Returns ranges in metres; no-hit rays return 0.0 (invalid), matching the
lidar convention where range == 0 means invalid.
"""
from __future__ import annotations

import numpy as np

from core.config import SceneConfig


def _ray_segment_t(
    ox: np.ndarray, oy: np.ndarray,
    dx: np.ndarray, dy: np.ndarray,
    x1: float, y1: float, x2: float, y2: float,
) -> np.ndarray:
    """Parametric t of ray (o + t*d, t>0) hitting segment (x1,y1)-(x2,y2).

    Returns inf where no hit. Vectorized over rays.
    """
    sx, sy = x2 - x1, y2 - y1
    denom = dx * sy - dy * sx
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(denom != 0.0, ((x1 - ox) * sy - (y1 - oy) * sx) / denom, np.inf)
        u = np.where(denom != 0.0, ((x1 - ox) * dy - (y1 - oy) * dx) / denom, np.inf)
    hit = (denom != 0.0) & (t > 1e-9) & (u >= 0.0) & (u <= 1.0)
    return np.where(hit, t, np.inf)


def raycast_ranges(
    angles: np.ndarray,
    scene: SceneConfig,
    object_center: tuple[float, float] | None = None,
) -> np.ndarray:
    """Ranges (m) for each angle; 0.0 where no hit.

    Primitives: board edges (x=0, x=S, y=0, y=S over [0,S]), box walls
    (x=-1, x=S+1, y=-1, y=S+1; box (S+2)x(S+2) centred on board centre),
    object circle (radius object_radius at object_center, default board centre).
    """
    s = scene.board_size
    dx, dy = np.sin(angles), np.cos(angles)
    ox = np.zeros_like(angles)
    oy = np.zeros_like(angles)

    ts = np.full_like(angles, np.inf)

    # Board edges (front edge y=0 from x=0..S, back edge y=S, left x=0, right x=S).
    for (x1, y1, x2, y2) in (
        (0.0, 0.0, s, 0.0),   # front edge (y=0)
        (0.0, s, s, s),       # back edge (y=S)
        (0.0, 0.0, 0.0, s),   # left edge (x=0)
        (s, 0.0, s, s),       # right edge (x=S)
    ):
        ts = np.minimum(ts, _ray_segment_t(ox, oy, dx, dy, x1, y1, x2, y2))

    # Box walls: inner faces at -gap and S+gap (gap = 1.0), spanning the box.
    g = scene.box_wall_inner
    for (x1, y1, x2, y2) in (
        (-g, -g, s + g, -g),        # near wall (y=-1)
        (-g, s + g, s + g, s + g),  # far wall (y=S+1)
        (-g, -g, -g, s + g),        # left wall (x=-1)
        (s + g, -g, s + g, s + g),  # right wall (x=S+1)
    ):
        ts = np.minimum(ts, _ray_segment_t(ox, oy, dx, dy, x1, y1, x2, y2))

    # Object circle (skipped when object_center is None).
    if object_center is not None:
        cx, cy = object_center
        b = dx * (ox - cx) + dy * (oy - cy)
        c = (ox - cx) ** 2 + (oy - cy) ** 2 - scene.object_radius**2
        disc = b * b - c
        with np.errstate(invalid="ignore"):
            t_circ = -b - np.sqrt(disc)
        circ_hit = (disc >= 0.0) & (t_circ >= 0.0)
        ts = np.where(circ_hit, np.minimum(ts, t_circ), ts)

    ranges = np.where(np.isfinite(ts), ts, 0.0)
    return ranges