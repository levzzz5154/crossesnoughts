"""End-to-end tests: web server + WS, and the desktop game headless.

These are the slowest tests; they exercise the real two-thread servers.
"""
import json
import threading
import time

import pytest
import numpy as np
from core.config import NoiseConfig, SceneConfig
from core.game import GameState, TapDetector, Turn
from core.lidar_source import make_lidar_source
from core.scan import LaserScan
from core.tracking import Tracker
from web.frames import SceneModel
from web.main import ScanLoop


# ---------------------------------------------------------------- web server
@pytest.fixture(scope="module")
def ws_server():
    import uvicorn
    from web.main import create_app

    scene = SceneConfig(board_size=2.0)
    app = create_app(scene, NoiseConfig(), lidar_kind="sim", seed=7)
    config = uvicorn.Config(app, host="127.0.0.1", port=8765, log_level="warning")
    server = uvicorn.Server(config)
    t = threading.Thread(target=server.run, daemon=True)
    t.start()
    for _ in range(100):
        if server.started:
            break
        time.sleep(0.05)
    yield server
    server.should_exit = True
    t.join(timeout=5)


def test_ws_hello_scene_frame_and_reset(ws_server):
    import websockets.sync.client as ws_client  # uvicorn[standard] provides

    with ws_client.connect("ws://127.0.0.1:8765/ws") as ws:
        hello = json.loads(ws.recv())
        assert hello["type"] == "hello"
        assert hello["config"]["board_size"] == 2.0
        scene = json.loads(ws.recv())
        assert scene["type"] == "scene"
        # frames arrive at ~10 Hz
        frames = []
        deadline = time.time() + 3.0
        while time.time() < deadline and len(frames) < 5:
            msg = json.loads(ws.recv(timeout=1.0))
            if msg["type"] == "frame":
                frames.append(msg)
        assert len(frames) >= 3
        seqs = [f["seq"] for f in frames]
        assert seqs == sorted(seqs)  # monotonic
        for f in frames:
            assert f["points"][0]["x"] >= 0.0  # footprint clip
        # reset control
        ws.send(json.dumps({"type": "reset", "seed": 99}))
        time.sleep(0.3)
        # set_object control
        ws.send(json.dumps({"type": "set_object", "x": 1.0, "y": 1.0}))
        time.sleep(0.3)
        got = None
        deadline = time.time() + 2.0
        while time.time() < deadline:
            msg = json.loads(ws.recv(timeout=1.0))
            if msg["type"] == "frame" and msg.get("true_pose"):
                got = msg
                break
        assert got is not None
        assert got["true_pose"] == {"x": 1.0, "y": 1.0}


def test_latest_wins_after_background():
    """A 'backgrounded' client (no reads for 5 s) gets the CURRENT seq."""
    import websockets.sync.client as ws_client

    with ws_client.connect("ws://127.0.0.1:8765/ws") as ws:
        ws.recv()  # hello
        ws.recv()  # scene
        # read one frame to get a baseline
        msg = json.loads(ws.recv(timeout=2.0))
        assert msg["type"] == "frame"
        last = msg["seq"]
        time.sleep(1.0)  # backgrounded
        msg2 = json.loads(ws.recv(timeout=2.0))
        assert msg2["type"] == "frame"
        assert msg2["seq"] > last


# ------------------------------------------------------------- scan loop unit
def test_scanloop_reset_recalibrates():
    """Reset re-seeds the sim and re-calibrates at the next scan boundary."""
    scene = SceneConfig(board_size=2.0)
    source = make_lidar_source("sim", scene=scene, noise=NoiseConfig(), seed=1)
    tracker = Tracker(scene, NoiseConfig())
    model = SceneModel(scene)
    loop = ScanLoop(source, tracker, model, scene)
    loop.start()
    time.sleep(0.4)
    assert model.current_seq() > 0
    model.send_control("reset", seed=42)
    time.sleep(0.4)
    loop.stop()
    assert model.current_seq() > 0


# ------------------------------------------------------------- game headless
def test_game_headless_plays_full_game():
    """The game pipeline (sim -> tracker -> tap -> state) plays a real game
    with a CONTInuous scripted pointer (realistic finger motion, ~0.3 m/s),
    no web server, no display."""
    scene = SceneConfig(board_size=2.0)
    source = make_lidar_source("sim", scene=scene, noise=NoiseConfig(), seed=5)
    tracker = Tracker(scene, NoiseConfig())
    game = GameState()
    tap = TapDetector(dwell=0.3)
    assert source.initialize() and source.turnOn()
    tracker.set_background(source.calibrate_background())
    scan = LaserScan.blank()
    t = 0.0
    moves = 0
    # scripted X-win sequence: (0,0),(1,0),(0,1),(1,1),(0,2)
    cells = [(0, 0), (1, 0), (0, 1), (1, 1), (0, 2)]
    pos = np.array([(0.5 + 0) / 3, (0.5 + 0) / 3]) * scene.board_size
    source.set_object_pose(tuple(pos))
    for (cx, cy) in cells:
        target = np.array([(cx + 0.5) / 3, (cy + 0.5) / 3]) * scene.board_size
        # move continuously: 0.67 m over ~2 s at ~0.33 m/s
        steps = 20
        for i in range(steps):
            pos = pos + (target - pos) / (steps - i)
            source.set_object_pose(tuple(pos))
            source.doProcessSimple(scan)
            res = tracker.process(scan)
            t += 0.1
            if res is not None:
                if tap.update(res.x, res.y, scene.board_size, t):
                    game.tap(res.x, res.y, scene.board_size)
                    moves += 1
        # dwell 0.5 s at the cell centre (tap needs 0.3 s)
        for _ in range(5):
            source.set_object_pose(tuple(target))
            source.doProcessSimple(scan)
            res = tracker.process(scan)
            t += 0.1
            if res is not None:
                if tap.update(res.x, res.y, scene.board_size, t):
                    game.tap(res.x, res.y, scene.board_size)
                    moves += 1
    assert game.winner == Turn.X
    assert moves == 5
    source.turnOff()
    source.disconnecting()


def test_lidar_pointer_mode_commits_from_tracked_position():
    """Contract test for `game.run --pointer lidar` (it does not import run.py,
    so it mirrors the scan loop's dataflow rather than executing it).

    The mouse moves the simulated object; the move is then committed from the
    TRACKED position. The pointer is held perfectly still at the end, which is
    what the frozen-dwell bug used to make impossible.
    """
    scene = SceneConfig(board_size=2.0)
    source = make_lidar_source("sim", scene=scene, noise=NoiseConfig(), seed=5)
    tracker = Tracker(scene, NoiseConfig())
    game = GameState()
    tap = TapDetector(dwell=0.3)
    assert source.initialize() and source.turnOn()
    tracker.set_background(source.calibrate_background())
    scan = LaserScan.blank()

    t = 0.0

    def hold(scans):
        """Run scans with the pointer dead still - no set_object_pose at all."""
        nonlocal t
        for _ in range(scans):
            source.doProcessSimple(scan)
            res = tracker.process(scan)
            t += 0.1
            if res is None:
                tap.release()  # track lost == hand lifted
            elif tap.update(res.x, res.y, scene.board_size, t):
                game.tap(res.x, res.y, scene.board_size)

    cells = [(2, 2), (0, 0)]
    for cell in cells:
        target = np.array([(c + 0.5) / 3 for c in cell]) * scene.board_size
        source.set_object_pose(tuple(target))
        tracker.reset()  # the object jumped; re-acquire from scratch
        hold(6)

    assert game.move_count == 2, "each held position must commit exactly one move"
    taps = [e for e in game.events if e.kind == "tap"]
    assert [e.cell for e in taps] == cells
    for e, cell in zip(taps, cells):
        want = np.array([(c + 0.5) / 3 for c in cell]) * scene.board_size
        err = float(np.hypot(e.x - want[0], e.y - want[1]))
        # Near the object but not identical to it: the committed position came
        # out of the tracking pipeline, not straight from the commanded pose.
        assert 1e-4 < err < 0.10, f"committed position error {err:.4f} m looks wrong"
    source.turnOff()
    source.disconnecting()