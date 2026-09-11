"""GameSettings persistence tests: round trip, validation, corrupt file."""
from pathlib import Path

import core.settings as settings_module
from core.settings import GameSettings


def test_round_trip(tmp_path: Path):
    s = GameSettings(
        board_size=1.5, board_yaw_deg=47.0, source_kind="real",
        source_port="/dev/ttyUSB0", team_x_name="Red", team_o_name="Blue",
        image_x="/tmp/x.png", image_o="/tmp/o.png",
        window_size=(1280, 1024), fullscreen=True, seed=None,
        tracking_mode="simple",
    )
    path = s.save(tmp_path / "settings.json")
    t = GameSettings.load(path)
    assert t.board_size == 1.5
    assert t.board_yaw_deg == 47.0
    assert t.source_kind == "real"
    assert t.source_port == "/dev/ttyUSB0"
    assert t.team_x_name == "Red"
    assert t.team_o_name == "Blue"
    assert t.image_x == "/tmp/x.png"
    assert t.image_o == "/tmp/o.png"
    assert t.window_size == (1280, 1024)
    assert t.fullscreen is True
    assert t.seed is None
    assert t.tracking_mode == "simple"


def test_missing_file_gives_defaults(tmp_path: Path):
    t = GameSettings.load(tmp_path / "nope.json")
    assert t.board_size == 2.0
    assert t.source_kind == "sim"
    assert t.team_x_name == "Player X"


def test_corrupt_file_gives_defaults(tmp_path: Path):
    p = tmp_path / "bad.json"
    p.write_text("{not json at all", encoding="utf-8")
    t = GameSettings.load(p)
    assert t.board_size == 2.0


def test_unknown_keys_ignored(tmp_path: Path):
    p = tmp_path / "s.json"
    p.write_text('{"board_size": 1.2, "future_field": 42}', encoding="utf-8")
    t = GameSettings.load(p)
    assert t.board_size == 1.2


def test_values_normalized_on_load_and_construct():
    s = GameSettings(board_size=99.0, board_yaw_deg=370.0, source_kind="bogus",
                     team_x_name="", tracking_mode="bogus")
    assert s.board_size == 2.0  # clamped to max
    assert s.board_yaw_deg == 10.0
    assert s.source_kind == "sim"
    assert s.team_x_name == "Player X"
    assert s.tracking_mode == "advanced"
    s2 = GameSettings(board_size=0.05)
    assert s2.board_size == 0.1  # clamped to min (now 0.1)
    s3 = GameSettings(window_size=(99999, 10))
    assert s3.window_size == (3840, 320)


def test_window_size_list_from_json(tmp_path: Path):
    p = tmp_path / "s.json"
    p.write_text('{"window_size": [800, 600]}', encoding="utf-8")
    t = GameSettings.load(p)
    assert t.window_size == (800, 600)


def test_frozen_settings_live_beside_executable(monkeypatch, tmp_path: Path):
    executable = tmp_path / "CrossesNoughts.exe"
    monkeypatch.setattr(settings_module.sys, "frozen", True, raising=False)
    monkeypatch.setattr(settings_module.sys, "executable", str(executable))
    assert settings_module._settings_path() == tmp_path / "settings.json"
