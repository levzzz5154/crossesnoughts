"""Three-corner square-board calibration geometry."""
from __future__ import annotations

import math


def solve_three_corners(front_left, back_left, back_right):
    """Return ``(size, yaw_deg, offset_x, offset_y)`` from raw board-axis points.

    Points are captured in this order because the lidar occupies (or is
    nearest) the omitted front-right corner.
    """
    ax, ay = front_left
    bx, by = back_left
    cx, cy = back_right
    vx, vy = bx - cx, by - cy       # origin -> front-left: +board X
    wx, wy = bx - ax, by - ay       # front-left -> back-left: +board Y
    sx, sy = math.hypot(vx, vy), math.hypot(wx, wy)
    if min(sx, sy) < 0.10:
        raise ValueError("corners are too close together")
    size = (sx + sy) / 2.0
    if abs(sx - sy) / size > 0.25:
        raise ValueError("the two board edges have different lengths")
    dot = abs(vx * wx + vy * wy) / (sx * sy)
    if dot > 0.30:
        raise ValueError("the captured edges are not perpendicular")

    # Average the two measured edge directions after rotating the Y edge
    # clockwise into an X edge. This damps ordinary hand-position error.
    ex1 = (vx / sx, vy / sx)
    ex2 = (wy / sy, -wx / sy)
    ex_len = math.hypot(ex1[0] + ex2[0], ex1[1] + ex2[1])
    ex = ((ex1[0] + ex2[0]) / ex_len, (ex1[1] + ex2[1]) / ex_len)
    yaw = math.atan2(-ex[1], ex[0])

    # The missing front-right origin is A + C - B. Rotate it into the solved
    # board axes; these become the alignment offsets.
    ox, oy = ax + cx - bx, ay + cy - by
    offset_x = ox * math.cos(yaw) - oy * math.sin(yaw)
    offset_y = oy * math.cos(yaw) + ox * math.sin(yaw)
    return size, math.degrees(yaw) % 360.0, offset_x, offset_y


def undo_current_transform(point, size, yaw_deg, alignment):
    """Convert a published board point back to the yaw=0 lidar-board axes."""
    x, y = alignment.from_board(point[0], point[1], size)
    yaw = math.radians(yaw_deg)
    return (x * math.cos(yaw) + y * math.sin(yaw),
            -x * math.sin(yaw) + y * math.cos(yaw))
