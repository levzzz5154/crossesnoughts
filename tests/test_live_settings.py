"""Settings screen -> child window live sync (game/live_settings.py)."""
import io
import os
import time

from core.settings import GameSettings
from game import live_settings
from game.live_settings import LiveSettingsReceiver, LiveSettingsSender


class FakePipeline:
    def __init__(self):
        self.calls = []

    def __getattr__(self, name):
        return lambda *a, **kw: self.calls.append((name, a, kw))


def wait_poll(rx, want, timeout=2.0):
    deadline = time.monotonic() + timeout
    changed = set()
    while time.monotonic() < deadline:
        changed |= rx.poll()
        if want <= changed:
            return changed
        time.sleep(0.01)
    return changed


def connect(child_settings, pipeline):
    r, w = os.pipe()
    tx_stream = os.fdopen(w, "w", encoding="utf-8")
    rx = LiveSettingsReceiver(os.fdopen(r, "rb"), child_settings, pipeline)
    return LiveSettingsSender(tx_stream), rx, tx_stream


def test_parent_edits_reach_child_pipeline_over_a_pipe():
    parent = GameSettings(source_kind="real", tracking_mode="simple")
    child = GameSettings(source_kind="real", tracking_mode="simple")
    pipe = FakePipeline()
    tx, rx, stream = connect(child, pipe)
    try:
        parent.board_yaw_deg = 42.0
        parent.board_offset_x = 0.25
        parent.image_x = "C:/Users/Данил/logo.png"  # non-ASCII survives
        tx.send(parent)
        changed = wait_poll(rx, {"board_yaw_deg", "board_offset_x", "image_x"})
        assert {"board_yaw_deg", "board_offset_x", "image_x"} <= changed
        assert child.board_yaw_deg == 42.0 and child.image_x == parent.image_x
        names = [c[0] for c in pipe.calls]
        assert "set_yaw" in names and "set_alignment" in names
        assert "configure_source" not in names  # source unchanged: no restart
    finally:
        stream.close()


def test_unchanged_settings_are_not_resent():
    buf = io.StringIO()
    tx = LiveSettingsSender(buf)
    s = GameSettings()
    tx.send(s)
    tx.send(s)
    assert buf.getvalue().count("\n") == 1
    s.board_size = 1.0
    tx.send(s)
    assert buf.getvalue().count("\n") == 2


def test_window_geometry_is_not_synced():
    child = GameSettings(window_size=(800, 800))
    pipe = FakePipeline()
    rx = LiveSettingsReceiver(io.BytesIO(b""), child, pipe)
    rx.apply({"window_size": [320, 320], "fullscreen": True})
    assert child.window_size == (800, 800) and not child.fullscreen


def test_sim_board_resize_rebuilds_once_after_drag_settles(monkeypatch):
    monkeypatch.setattr(live_settings, "SIM_REBUILD_DELAY", 0.05)
    child = GameSettings(source_kind="sim")
    pipe = FakePipeline()
    rx = LiveSettingsReceiver(io.BytesIO(b""), child, pipe)
    for size in (1.0, 1.1, 1.2):  # a slider drag
        rx.apply({"board_size": size})
    assert all(c == ("set_board", (size,), {"rebuild": False})
               for c, size in zip(pipe.calls, (1.0, 1.1, 1.2)))
    time.sleep(0.06)
    rx.poll()
    assert pipe.calls[-1] == ("set_board", (1.2,), {"rebuild": True})
    rx.poll()
    assert sum(c[2].get("rebuild") is True for c in pipe.calls) == 1


def test_source_change_restarts_child_source():
    child = GameSettings(source_kind="sim")
    pipe = FakePipeline()
    rx = LiveSettingsReceiver(io.BytesIO(b""), child, pipe)
    rx.apply({"source_kind": "real", "source_port": "COM8"})
    assert ("configure_source", ("real",),
            {"port": "COM8", "replay_file": None, "seed": None}) in pipe.calls


def test_sender_survives_child_exit():
    r, w = os.pipe()
    os.close(r)
    tx = LiveSettingsSender(os.fdopen(w, "w"))
    tx.send(GameSettings())  # broken pipe must not raise
    assert tx.stream is None
