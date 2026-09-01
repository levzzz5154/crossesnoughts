"""Frame dataclass + SceneModel (latest-wins store + control queue).

SceneModel is the shared boundary between the scan thread (writer) and the
WS/event-loop thread (reader): single-slot latest-wins frame store with a
seq counter; stale frames are dropped, never queued. A control queue
(maxsize=1) carries client commands to the scan thread, drained between
doProcessSimple calls so resets apply at the next scan boundary.
"""
from __future__ import annotations

import json
import queue
import threading
from dataclasses import dataclass, field

import numpy as np

from core.config import SceneConfig
from core.scan import SCAN_MAX_RANGE, SCAN_MIN_RANGE, SCAN_POINT_COUNT


@dataclass
class Frame:
    seq: int
    stamp: int  # ns
    scan_freq: float
    points: list = field(default_factory=list)  # valid hits in board coords
    points_invalid: list = field(default_factory=list)  # indices of 0-range
    tracked: dict | None = None
    true_pose: dict | None = None

    def to_json(self) -> str:
        return json.dumps(
            {
                "type": "frame",
                "seq": self.seq,
                "stamp": self.stamp,
                "scan_freq": self.scan_freq,
                "points": self.points,
                "points_invalid": self.points_invalid,
                "tracked": self.tracked,
                "true_pose": self.true_pose,
            }
        )


def clip_to_footprint(x: np.ndarray, y: np.ndarray, s: float) -> tuple[np.ndarray, np.ndarray]:
    """Drop points outside the board footprint [0, S]^2 (wall echoes)."""
    keep = (x >= 0.0) & (x <= s) & (y >= 0.0) & (y <= s)
    return x[keep], y[keep]


def build_frame(
    seq: int,
    scan,
    scene: SceneConfig,
    tracked=None,
    true_pose=None,
) -> Frame:
    """Convert a LaserScan + optional TrackResult into a wire Frame.

    points: valid hits precomputed in BOARD coords, clipped to [0,S]^2.
    points_invalid: indices of the 0-range points in the full scan.
    """
    pts = scan.points
    n = len(pts)
    angles = np.array([p.angle for p in pts], dtype=float)
    ranges = np.array([p.range for p in pts], dtype=float)
    invalid_idx = [i for i in range(n) if ranges[i] <= 0.0]
    valid = ranges > 0.0
    x, y = ranges[valid] * np.sin(angles[valid]), ranges[valid] * np.cos(angles[valid])
    x, y = clip_to_footprint(x, y, scene.board_size)
    points = [{"x": float(xx), "y": float(yy)} for xx, yy in zip(x, y)]
    trk = None
    if tracked is not None:
        trk = {
            "x": tracked.x,
            "y": tracked.y,
            "vx": tracked.vx,
            "vy": tracked.vy,
            "conf": tracked.conf,
            "missed": tracked.missed,
            "age": tracked.age,
        }
    tp = None
    if true_pose is not None:
        tp = {"x": float(true_pose[0]), "y": float(true_pose[1])}
    return Frame(
        seq=seq,
        stamp=int(scan.stamp),
        scan_freq=float(scan.scanFreq),
        points=points,
        points_invalid=invalid_idx,
        tracked=trk,
        true_pose=tp,
    )


class SceneModel:
    """Latest-wins frame store + control queue (thread-safe)."""

    def __init__(self, scene: SceneConfig, scan_freq: float = 10.0):
        self.scene = scene
        self.scan_freq = scan_freq
        self._lock = threading.Lock()
        self._seq = 0
        self._frame: Frame | None = None
        self.control: queue.Queue = queue.Queue(maxsize=1)

    # -- scan-thread side ---------------------------------------------------
    def publish(self, frame: Frame) -> None:
        with self._lock:
            self._seq += 1
            frame.seq = self._seq
            self._frame = frame

    def drain_control(self) -> dict | None:
        """Scan thread: pop one control command (or None)."""
        try:
            return self.control.get_nowait()
        except queue.Empty:
            return None

    # -- WS-thread side -----------------------------------------------------
    def latest(self, last_seq: int) -> Frame | None:
        """Return the newest frame if it is newer than last_seq, else None."""
        with self._lock:
            if self._frame is None or self._frame.seq <= last_seq:
                return None
            f = self._frame
            # shallow copy so the reader can hold it safely
            return Frame(
                seq=f.seq, stamp=f.stamp, scan_freq=f.scan_freq,
                points=list(f.points), points_invalid=list(f.points_invalid),
                tracked=f.tracked, true_pose=f.true_pose,
            )

    def current_seq(self) -> int:
        with self._lock:
            return self._seq

    def send_control(self, op: str, **kw) -> bool:
        """WS thread: queue a control command (latest-wins, maxsize=1)."""
        try:
            self.control.put_nowait({"op": op, **kw})
            return True
        except queue.Full:
            return False


def hello_message(scene: SceneConfig, scan_freq: float,
                  tracking_mode: str = "advanced") -> str:
    """Server->client hello (once per connect)."""
    return json.dumps(
        {
            "type": "hello",
            "version": "1.0",
            "config": {
                "board_size": scene.board_size,
                "box_w": scene.box_w,
                "box_d": scene.box_w,
                "box_h": scene.box_height,
                "lidar_pos_board": [0.0, 0.0],
                "scan_freq": scan_freq,
                "sample_rate": 3.0,
                "point_count": SCAN_POINT_COUNT,
                "min_range": SCAN_MIN_RANGE,
                "max_range": SCAN_MAX_RANGE,
                "object_radius": scene.object_radius,
                "tracking_mode": tracking_mode,
            },
        }
    )


def scene_message(scene: SceneConfig) -> str:
    """Server->client scene geometry (once per connect)."""
    s = scene.board_size
    g = scene.box_wall_inner
    segments = [
        # box walls at -g and S+g
        {"x1": -g, "y1": -g, "x2": -g, "y2": s + g},
        {"x1": s + g, "y1": -g, "x2": s + g, "y2": s + g},
        {"x1": -g, "y1": -g, "x2": s + g, "y2": -g},
        {"x1": -g, "y1": s + g, "x2": s + g, "y2": s + g},
        # board edges 0..S
        {"x1": 0.0, "y1": 0.0, "x2": 0.0, "y2": s},
        {"x1": s, "y1": 0.0, "x2": s, "y2": s},
        {"x1": 0.0, "y1": 0.0, "x2": s, "y2": 0.0},
        {"x1": 0.0, "y1": s, "x2": s, "y2": s},
    ]
    return json.dumps(
        {
            "type": "scene",
            "segments": segments,
            "board": {"x": 0.0, "y": 0.0, "size": s},
        }
    )
