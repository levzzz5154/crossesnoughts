"""Pipeline integration tests: source lifecycle, live frame, yaw remap,
board resize, and the full tap -> own-cell-no-op flow through the game
state machine at real 10 Hz sim cadence."""
import time

import pytest

from core.game import Turn
from game.pipeline import Pipeline


def wait_for(p: Pipeline, pred, timeout=15.0, what="pipeline state"):
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        last = p.snapshot()
        if pred(last):
            return last
        time.sleep(0.05)
    raise AssertionError(f"timeout waiting for {what}: {last}")


def wait_game(p: Pipeline, pred, timeout=15.0, what="game state"):
    t0 = time.time()
    while time.time() - t0 < timeout:
        if pred(p.game):
            return
        time.sleep(0.05)
    raise AssertionError(f"timeout waiting for {what}: grid={p.game.grid} "
                         f"moves={p.game.move_count} turn={p.game.turn}")


@pytest.fixture()
def pipeline():
    p = Pipeline()
    p.configure_source("sim", seed=5)
    p.start()
    yield p
    p.stop()


def test_sim_source_comes_up(pipeline):
    snap = wait_for(pipeline,
                    lambda s: s["source_ok"] and s["bg_ready"] and s["seq"] > 0,
                    what="sim source up + calibrated")
    assert snap["source_error"] is None
    assert snap["hz"] == pytest.approx(10.0, abs=0.5)
    assert snap["pts_per_scan"] == 300
    assert len(snap["points"]) > 100  # valid hits in board coords
    for (x, y) in snap["points"][:20]:
        assert isinstance(x, float)


def test_simple_mode_is_ready_without_calibration():
    p = Pipeline(board_size=1.0, tracking_mode="simple")
    p.configure_source("sim", seed=5)
    p.start()
    try:
        snap = wait_for(
            p,
            lambda s: s["source_ok"] and s["bg_ready"] and
            s["tracking_mode"] == "simple",
            what="simple source up",
        )
        assert not snap["calibrating"]
        p.set_pointer(0.7, 0.2)
        snap = wait_for(p, lambda s: s["track"] is not None,
                        what="simple board-area track")
        x, y, conf = snap["track"]
        assert 0.0 <= x <= 1.0
        assert 0.0 <= y <= 1.0
        assert conf == 1.0
    finally:
        p.stop()


def test_pointer_drives_simulated_hand(pipeline):
    wait_for(pipeline, lambda s: s["source_ok"] and s["bg_ready"])
    # (0.7, 1.2) is NOT the sim default pose (board centre) - this test is
    # only meaningful away from the default
    pipeline.set_pointer(0.7, 1.2)
    snap = wait_for(pipeline, lambda s: s["track"] is not None,
                    what="track acquisition")
    x, y, conf = snap["track"]
    assert x == pytest.approx(0.7, abs=0.15)
    assert y == pytest.approx(1.2, abs=0.15)
    assert conf > 0.5


def test_yaw_keeps_pointer_target_stable(pipeline):
    """The pointer target is in DOWNSTREAM coords: rotating the window while
    holding the target steady keeps the tracked position steady (the sim
    object is re-placed in its own rotated frame every scan)."""
    wait_for(pipeline, lambda s: s["source_ok"] and s["bg_ready"])
    pipeline.set_pointer(1.0, 0.5)
    wait_for(pipeline, lambda s: s["track"] is not None and
             abs(s["track"][0] - 1.0) < 0.2 and abs(s["track"][1] - 0.5) < 0.2)
    pipeline.set_yaw(137.0)
    snap = wait_for(pipeline, lambda s: s["yaw_deg"] == 137.0 and
                    s["track"] is not None, what="yaw remap")
    x, y, _ = snap["track"]
    assert x == pytest.approx(1.0, abs=0.25)
    assert y == pytest.approx(0.5, abs=0.25)


def test_board_resize_recalibrates_sim(pipeline):
    wait_for(pipeline, lambda s: s["source_ok"] and s["bg_ready"])
    pipeline.set_board(1.5)
    snap = wait_for(pipeline, lambda s: s["board_size"] == 1.5 and
                    s["bg_ready"], what="board resize + recalibration")
    assert snap["board_size"] == 1.5


def test_full_tap_flow_with_own_cell_noop(pipeline):
    wait_for(pipeline, lambda s: s["source_ok"] and s["bg_ready"])
    pipeline.phase = "game"

    def cell_center(cx, cy, S):
        return ((cx + 0.5) / 3 * S, (cy + 0.5) / 3 * S)

    # move 1: X at (0, 0)
    pipeline.set_pointer(*cell_center(0, 0, 2.0))
    wait_game(pipeline, lambda g: g.grid[0][0] is Turn.X,
              what="X move at (0,0)")
    # move 2: O at (1, 1)
    pipeline.set_pointer(None, None)  # player disappears: re-arm only
    time.sleep(0.2)
    pipeline.set_pointer(*cell_center(1, 1, 2.0))
    wait_game(pipeline, lambda g: g.grid[1][1] is Turn.O,
              what="O move at (1,1)")
    assert pipeline.game.move_count == 2
    assert pipeline.game.turn is Turn.X
    # move 3 attempt: X re-taps its OWN cell (0,0) -> no-op, turn stays X
    pipeline.set_pointer(None, None)
    time.sleep(0.2)
    pipeline.set_pointer(*cell_center(0, 0, 2.0))
    time.sleep(0.5)  # active presence must not re-fire
    assert pipeline.game.move_count == 2, "own-cell appearance must be a no-op"
    assert pipeline.game.turn is Turn.X
    assert pipeline.game.grid[0][0] is Turn.X
    # move 3: X overwrites O's cell (1,1) -> allowed
    pipeline.set_pointer(None, None)
    time.sleep(0.2)
    pipeline.set_pointer(*cell_center(1, 1, 2.0))
    wait_game(pipeline, lambda g: g.grid[1][1] is Turn.X,
              what="X overwriting O at (1,1)")
    assert pipeline.game.move_count == 3
    assert pipeline.game.turn is Turn.O


def test_new_game_clears(pipeline):
    wait_for(pipeline, lambda s: s["source_ok"] and s["bg_ready"])
    pipeline.phase = "game"
    pipeline.set_pointer(0.333, 0.333)
    wait_game(pipeline, lambda g: g.move_count >= 1)
    pipeline.new_game()
    wait_game(pipeline, lambda g: g.move_count == 0 and g.winner is None
              and g.turn is Turn.X, what="new game reset")


def test_scan_loop_exception_reconnects_instead_of_killing_thread():
    """A lost device (serial exception mid-stream) must be reported and
    re-initialized; the scan thread has to survive it."""
    import threading

    class Flaky:
        def __init__(self):
            self.inits = 0
            self.reads = 0

        def initialize(self):
            self.inits += 1
            return True

        def turnOn(self):
            return True

        def doProcessSimple(self, scan):
            self.reads += 1
            if self.reads == 1:
                raise ConnectionError("serial read failed: unplugged")
            time.sleep(0.01)
            return False

    src = Flaky()
    p = Pipeline(tracking_mode="simple")
    p.source = src
    seen = []
    orig = p._publish
    p._publish = lambda **kw: (seen.append(kw), orig(**kw))
    t = threading.Thread(target=p._run, daemon=True)
    t.start()
    deadline = time.monotonic() + 5.0
    while src.inits < 2 and time.monotonic() < deadline:
        time.sleep(0.05)
    p.stop_event.set()
    t.join(2.0)
    assert src.inits >= 2
    assert any("connection lost" in (kw.get("source_error") or "") for kw in seen)
