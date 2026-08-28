"""Wire tests (plan §14 step 7, §17.8): Frame JSON schema, clipping, seq."""
import json

import numpy as np
import pytest

from core.config import SceneConfig
from core.scan import LaserPoint, LaserScan
from core.tracking import TrackResult
from web.frames import SceneModel, build_frame, hello_message, scene_message


def _scan():
    scan = LaserScan()
    scan.points = [LaserPoint(angle=float(a)) for a in np.linspace(-np.pi, np.pi, 300)]
    scan.size = 300
    scan.stamp = 1724800000000000000
    scan.scanFreq = 10.0
    for i, p in enumerate(scan.points):
        p.range = 0.0 if i % 10 == 0 else 2.0
    return scan


def test_frame_to_json_parses_per_schema():
    scene = SceneConfig(board_size=2.0)
    scan = _scan()
    trk = TrackResult(0.52, 1.18, 0.01, 0.0, 0.95, 0, 42)
    f = build_frame(1, scan, scene, tracked=trk, true_pose=(0.5, 1.2))
    d = json.loads(f.to_json())
    assert d["type"] == "frame"
    assert d["seq"] == 1
    assert d["stamp"] == 1724800000000000000
    assert d["scan_freq"] == 10.0
    assert isinstance(d["points"], list)
    assert all({"x", "y"} <= set(p) for p in d["points"])
    assert d["tracked"]["x"] == 0.52
    assert d["tracked"]["conf"] == 0.95
    assert d["true_pose"] == {"x": 0.5, "y": 1.2}


def test_points_clipped_to_footprint():
    scene = SceneConfig(board_size=2.0)
    scan = LaserScan()
    scan.points = [
        LaserPoint(angle=0.0, range=3.0),      # far wall -> outside footprint
        LaserPoint(angle=np.pi / 4, range=1.31),  # object -> inside
        LaserPoint(angle=-np.pi / 2, range=1.0),  # left wall -> outside
    ]
    scan.size = 3
    scan.stamp = 1
    f = build_frame(1, scan, scene)
    assert len(f.points) == 1
    x, y = f.points[0]["x"], f.points[0]["y"]
    assert 0.0 <= x <= 2.0 and 0.0 <= y <= 2.0
    assert f.points_invalid == []


def test_points_invalid_indices():
    scene = SceneConfig(board_size=2.0)
    scan = LaserScan()
    scan.points = [
        LaserPoint(angle=0.0, range=0.0),
        LaserPoint(angle=0.1, range=2.0),
        LaserPoint(angle=0.2, range=0.0),
    ]
    scan.size = 3
    scan.stamp = 1
    f = build_frame(1, scan, scene)
    assert f.points_invalid == [0, 2]


def test_seq_monotonic_latest_wins():
    model = SceneModel(SceneConfig())
    f1 = build_frame(0, _scan(), SceneConfig())
    f2 = build_frame(0, _scan(), SceneConfig())
    model.publish(f1)
    model.publish(f2)
    assert f1.seq == 1
    assert f2.seq == 2
    got = model.latest(0)
    assert got.seq == 2  # latest-wins: f1 dropped
    assert model.latest(2) is None  # stale
    assert model.current_seq() == 2


def test_hello_and_scene_messages():
    scene = SceneConfig(board_size=2.0)
    h = json.loads(hello_message(scene, 10.0))
    assert h["type"] == "hello"
    assert h["version"] == "1.0"
    assert h["config"]["board_size"] == 2.0
    assert h["config"]["box_w"] == 4.0
    assert h["config"]["box_d"] == 4.0
    assert h["config"]["box_h"] == 2.4
    assert h["config"]["lidar_pos_board"] == [0.0, 0.0]
    assert h["config"]["point_count"] == 300
    assert h["config"]["min_range"] == 0.1
    assert h["config"]["max_range"] == 8.0
    assert h["config"]["object_radius"] == 0.10
    s = json.loads(scene_message(scene))
    assert s["type"] == "scene"
    assert len(s["segments"]) == 8
    assert s["board"] == {"x": 0.0, "y": 0.0, "size": 2.0}
    # walls at -1 and 3 for S=2
    xs = {seg["x1"] for seg in s["segments"]} | {seg["x2"] for seg in s["segments"]}
    assert -1.0 in xs and 3.0 in xs


def test_control_queue_latest_wins():
    model = SceneModel(SceneConfig())
    assert model.send_control("reset", seed=7)
    assert not model.send_control("reset", seed=8)  # queue full -> latest wins
    cmd = model.drain_control()
    assert cmd == {"op": "reset", "seed": 7}
    assert model.drain_control() is None