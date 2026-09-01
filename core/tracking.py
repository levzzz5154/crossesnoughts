"""Tracker: background subtraction -> clustering -> centroid + arc-centroid
bias correction -> alpha-beta association.

Plan §7. Background threshold fixed at 0.10 m = 5*sigma_max at the S=2.0 far
corner (sigma_r(2.83 m) = 0.020 m) — board size is capped at 2.0 m, so 0.10 m
is >= 5 sigma everywhere.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import BackgroundTable
from core.scan import LaserScan, SCAN_ANGLE_INCREMENT
from core import transforms as tf

# Background-subtraction threshold (m). 5*sigma at S=2.0 far corner.
BG_THRESHOLD = 0.10
# Cluster linkage: angular gap < 3 bins AND radial gap < 0.25 m.
CLUSTER_ANGLE_GAP = 3 * SCAN_ANGLE_INCREMENT
CLUSTER_RADIAL_GAP = 0.25
# Alpha-beta filter.
ALPHA = 0.6
BETA = 0.3
GATE = 0.5  # m
MISSED_LIMIT = 10
DT_MIN = 0.05
DT_MAX = 0.5
# Coast velocity clamp: 1.2 m/s max (pointer speeds are far lower).
V_MAX = 1.2


@dataclass
class TrackResult:
    x: float  # board x
    y: float  # board y
    vx: float
    vy: float
    conf: float
    missed: int
    age: int


class SimpleTracker:
    """Small board-only tracker for setup and debugging.

    This is deliberately a detector rather than a filter.  It projects valid
    ranges into board coordinates, drops anything outside the configured board
    footprint, trims a tiny perimeter occupied by the empty board edge, and
    reports the median remaining point.  There is no calibration,
    clustering, prediction, association, or coasting; when there are no
    interior board hits the result is simply ``None``.

    The median keeps a single return from moving the preview as much as a mean
    would, while keeping the implementation useful as a deliberately dumb
    baseline next to :class:`Tracker`.  Board orientation is already handled
    by ``RotatedLidarSource`` before this class sees a scan.
    """

    EDGE_MARGIN = 0.03  # m; reject the board rim, not the usable board area

    def __init__(self, scene: SceneConfig, noise: NoiseConfig):
        self.scene = scene
        self.noise = noise
        self._age = 0

    def reset(self) -> None:
        self._age = 0

    def process(self, scan: LaserScan) -> TrackResult | None:
        pts = scan.points[: scan.size]
        if not pts:
            return None
        angles = np.asarray([p.angle for p in pts], dtype=float)
        ranges = np.asarray([p.range for p in pts], dtype=float)
        valid = (ranges > self.noise.min_range) & (ranges <= self.noise.max_range)
        if not np.any(valid):
            return None

        angles, ranges = angles[valid], ranges[valid]
        x, y = tf.polar_to_board(ranges, angles)
        margin = min(self.EDGE_MARGIN, self.scene.board_size / 4.0)
        inside = (
            (x >= margin) & (x <= self.scene.board_size - margin) &
            (y >= margin) & (y <= self.scene.board_size - margin)
        )
        if not np.any(inside):
            return None

        x, y = x[inside], y[inside]
        self._age += 1
        return TrackResult(
            x=float(np.median(x)), y=float(np.median(y)),
            vx=0.0, vy=0.0, conf=1.0, missed=0, age=self._age,
        )


class Tracker:
    def __init__(self, scene: SceneConfig, noise: NoiseConfig):
        self.scene = scene
        self.noise = noise
        self._background: BackgroundTable | None = None
        self._tracked: tuple[float, float, float, float] | None = None  # x,y,vx,vy
        self._last_t: float | None = None
        self._missed = 0
        self._age = 0

    def set_background(self, table: BackgroundTable) -> None:
        self._background = table

    def reset(self) -> None:
        """Drop the track: next scan re-acquires the largest cluster."""
        self._tracked = None
        self._last_t = None
        self._missed = 0
        self._age = 0

    # -- public -------------------------------------------------------------
    def process(self, scan: LaserScan) -> TrackResult | None:
        if self._background is None:
            return None
        angles = np.array([p.angle for p in scan.points], dtype=float)
        ranges = np.array([p.range for p in scan.points], dtype=float)
        now = scan.stamp / 1e9
        return self._process(angles, ranges, now)

    # -- internals ----------------------------------------------------------
    def _process(self, angles: np.ndarray, ranges: np.ndarray, now: float) -> TrackResult | None:
        # (1) FILTER: keep points with min_range < r <= max_range (0 = invalid).
        valid = (ranges > self.noise.min_range) & (ranges <= self.noise.max_range)
        a, r = angles[valid], ranges[valid]
        if a.size == 0:
            return self._miss_step(now)

        # (2) BACKGROUND SUBTRACTION.
        bg = self._background(a)
        mask = np.abs(r - bg) > BG_THRESHOLD
        a, r = a[mask], r[mask]
        if a.size == 0:
            return self._miss_step(now)

        # polar -> board
        xb, yb = tf.polar_to_board(r, a)

        # (3) CLUSTERING on angle adjacency.
        clusters = self._cluster(a, r, xb, yb)

        # (4) CLUSTER ACCEPTANCE + range-consistency rejection + centroid.
        if self._tracked is None:
            return self._acquire(clusters, now)

        px, py, _, _ = self._tracked
        best: tuple[float, float] | None = None
        best_r = np.inf
        for (ca, cr, cx, cy) in clusters:
            if len(cr) == 1:
                # 1-point cluster accepted only within prediction gate.
                if np.hypot(cx[0] - px, cy[0] - py) > GATE:
                    continue
            # (4-6) range-consistency filter, polar mean, arc-centroid bias.
            got = self._centroid(ca, cr)
            if got is None:
                continue
            cand, r_m = (got[0], got[1]), got[2]
            if np.hypot(cand[0] - px, cand[1] - py) > GATE:
                continue
            # Prefer the NEAREST gate-passing cluster: mixed-pixel pulls and
            # wall echoes always lie BEYOND the object, so the object is the
            # smallest-range candidate inside the gate.
            if r_m < best_r:
                best = cand
                best_r = r_m
        if best is None:
            return self._miss_step(now)
        return self._associate(best, now)

    def _centroid(self, ca: np.ndarray, cr: np.ndarray):
        """One cluster -> (x, y, r_m) in board coords, or None if it rejects.

        Plan §7 steps 4-6: range-consistency filter (median +- 3*sigma),
        inverse-variance weighted polar mean, arc-centroid bias correction.
        Shared by acquisition and tracking so the two cannot drift apart.
        """
        med = float(np.median(cr))
        sigma = self.noise.range_std(med)
        keep = np.abs(cr - med) <= 3 * sigma
        if keep.sum() == 0:
            return None
        ca2, cr2 = ca[keep], cr[keep]
        # (5) inverse-variance weighted polar mean.
        w = 1.0 / (self.noise.range_std(cr2) ** 2 + 1e-6)
        theta_m = float(np.sum(w * ca2) / np.sum(w))
        r_m = float(np.sum(w * cr2) / np.sum(w))
        # (6) ARC-CENTROID BIAS CORRECTION.
        beta = np.arcsin(np.clip(self.scene.object_radius / r_m, 0.0, 1.0))
        r_hat = r_m + self.scene.object_radius * (1.0 + np.cos(beta)) / 2.0
        x, y = tf.polar_to_board(np.array([r_hat]), np.array([theta_m]))
        return float(x[0]), float(y[0]), r_m

    def _acquire(self, clusters, now: float) -> TrackResult | None:
        """Untracked: acquire the NEAREST plausible cluster.

        Deliberately NOT the largest. Mixed pixels split one object into a
        clean fragment at the true range plus a fragment pulled halfway toward
        whatever is behind it, so the two fragments frequently tie in size --
        and a size tie was broken by angle order, which is arbitrary. The near
        fragment is always the object: mixed-pixel pulls, wall echoes and board
        bleed-through all lie BEYOND it.

        Measured on a static object, 15 scans, seed 5: largest-pick lands
        70.4 cm off at (1.0, 1.0) and 56.6 cm off at (0.333, 1.0); nearest-pick
        lands 0.9 cm and 1.0 cm. Across 8 positions x 3 seeds the rule change
        never made any position worse.
        """
        if not clusters:
            return None
        best: tuple[float, float] | None = None
        best_r = np.inf
        for (ca, cr, cx, cy) in clusters:
            got = self._centroid(ca, cr)
            if got is None:
                continue
            x, y, r_m = got
            if r_m < best_r:
                best, best_r = (x, y), r_m
        if best is None:
            return None
        return self._associate(best, now)

    def _cluster(
        self,
        a: np.ndarray, r: np.ndarray, xb: np.ndarray, yb: np.ndarray,
    ) -> list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
        order = np.argsort(a)
        a, r, xb, yb = a[order], r[order], xb[order], yb[order]
        clusters: list[tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = []
        if a.size == 0:
            return clusters
        start = 0
        for i in range(1, a.size):
            da = a[i] - a[i - 1]
            dr = np.hypot(xb[i] - xb[i - 1], yb[i] - yb[i - 1])
            if da > CLUSTER_ANGLE_GAP or dr > CLUSTER_RADIAL_GAP:
                clusters.append((a[start:i], r[start:i], xb[start:i], yb[start:i]))
                start = i
        clusters.append((a[start:], r[start:], xb[start:], yb[start:]))
        return clusters

    def _predict(self, now: float) -> tuple[float, float]:
        x, y, vx, vy = self._tracked
        dt = self._dt(now)
        return x + vx * dt, y + vy * dt

    def _dt(self, now: float) -> float:
        if self._last_t is None:
            return 0.1
        return float(np.clip(now - self._last_t, DT_MIN, DT_MAX))

    def _associate(self, z: tuple[float, float], now: float) -> TrackResult:
        if self._tracked is None:
            self._tracked = (z[0], z[1], 0.0, 0.0)
            self._missed = 0
            self._age = 1
            self._last_t = now
            return TrackResult(z[0], z[1], 0.0, 0.0, 1.0, 0, 1)
        x, y, vx, vy = self._tracked
        px, py = self._predict(now)
        dt = self._dt(now)
        dx, dy = z[0] - px, z[1] - py
        if np.hypot(dx, dy) <= GATE:
            x = px + ALPHA * dx
            y = py + ALPHA * dy
            vx = vx + BETA * dx / dt
            vy = vy + BETA * dy / dt
            self._missed = 0
        else:
            x, y = px, py
            vx, vy = vx * 0.9, vy * 0.9
            self._missed += 1
        # clamp coast velocity
        sp = np.hypot(vx, vy)
        if sp > V_MAX:
            vx, vy = vx / sp * V_MAX, vy / sp * V_MAX
        self._tracked = (x, y, vx, vy)
        self._age += 1
        self._last_t = now
        conf = float(np.exp(-self._missed / 3.0))
        return TrackResult(x, y, vx, vy, conf, self._missed, self._age)

    def _miss_step(self, now: float) -> TrackResult | None:
        if self._tracked is None:
            return None
        x, y = self._predict(now)
        vx, vy = self._tracked[2] * 0.9, self._tracked[3] * 0.9
        sp = np.hypot(vx, vy)
        if sp > V_MAX:
            vx, vy = vx / sp * V_MAX, vy / sp * V_MAX
        self._missed += 1
        self._tracked = (x, y, vx, vy)
        self._last_t = now
        conf = float(np.exp(-self._missed / 3.0))
        if self._missed > MISSED_LIMIT:
            self._tracked = None
            self._missed = 0
            self._age = 0
            return None
        return TrackResult(x, y, vx, vy, conf, self._missed, self._age)
