"""Regression coverage for the settings/SDK race on Windows serial ports."""
from types import SimpleNamespace

import pytest

from core.config import NoiseConfig, SceneConfig
from core.real import RealLidarSource
from core.yaw import RotatedLidarSource
from game import devstatus, settings_screen
from game.pipeline import Pipeline
from game.settings_screen import SettingsScreen


@pytest.mark.parametrize("source_ok", [False, True])
def test_settings_never_open_port_during_connect_or_stream(monkeypatch, source_ok):
    monkeypatch.setattr(settings_screen, "EXCLUSIVE_PORTS", True)

    def forbidden(*args, **kwargs):
        pytest.fail("settings must not open the pipeline's serial port")

    monkeypatch.setattr(devstatus, "open_port", forbidden)
    monkeypatch.setattr(settings_screen, "list_serial_ports", lambda: [
        {"path": "COM8", "kind": "usb"}])
    configured = []
    screen = SettingsScreen.__new__(SettingsScreen)
    screen.kind = "real"
    screen.mode = "simple"
    screen.port = "COM8"
    screen.pipeline = SimpleNamespace(
        snapshot=lambda: {"source_ok": source_ok},
        configure_source=lambda *a, **kw: configured.append((a, kw)))
    screen.settings = SimpleNamespace(save=lambda: None, seed=None)
    screen.start_probe()
    screen._select_source("real")
    screen.start_probe()  # Re-scan ports
    assert screen.ports[0]["path"] == "COM8"
    assert not screen.probing
    assert len(configured) == 1
    lines = screen._status_lines({"source_ok": source_ok})
    assert not any("no lidar on serial ports" in text for text, _ in lines)


def test_empty_port_selection_uses_enumeration_without_sniffing(monkeypatch):
    monkeypatch.setattr(settings_screen, "EXCLUSIVE_PORTS", True)
    monkeypatch.setattr(settings_screen, "list_serial_ports", lambda: [
        {"path": "COM8", "kind": "usb"}])
    pipeline = Pipeline()
    screen = SettingsScreen.__new__(SettingsScreen)
    screen.kind, screen.port = "real", ""
    screen.pipeline = pipeline
    screen.f_port = SimpleNamespace(text="", commit=lambda: None)
    screen.settings = SimpleNamespace(source_port="", seed=7, save=lambda: None)
    screen.start_probe()
    assert screen.port == screen.f_port.text == screen.settings.source_port == "COM8"
    assert pipeline.control == [("set_source", "real", "COM8", None, 7)]


@pytest.mark.parametrize("stage", ["initialize", "turnOn"])
def test_sdk_error_reaches_pipeline_before_disconnect(monkeypatch, stage):
    source = RealLidarSource(SceneConfig(), NoiseConfig(), port="COM8")
    shutdowns = []
    source._ydlidar = SimpleNamespace(
        os_init=lambda: None, os_shutdown=lambda: shutdowns.append(True))
    source._laser = SimpleNamespace(
        setlidaropt=lambda *args: True,
        initialize=lambda: stage != "initialize",
        turnOn=lambda: stage != "turnOn",
        DescribeError=lambda: b"Access is denied",
        turnOff=lambda: True,
        disconnecting=lambda: None)
    pipeline = Pipeline(tracking_mode="simple")
    wrapped = RotatedLidarSource(source)
    assert not pipeline._setup(wrapped)
    error = pipeline.snapshot()["source_error"]
    assert f"{stage}() failed on COM8" in error
    assert "Access is denied" in error
    assert not shutdowns  # shutdown belongs to teardown, after error capture
    pipeline._teardown(wrapped)
    assert shutdowns == [True]


def test_rejected_sdk_option_is_reported_before_connecting():
    source = RealLidarSource(SceneConfig(), NoiseConfig(), port="COM8")
    source._ydlidar = SimpleNamespace(os_init=lambda: None)
    source._laser = SimpleNamespace(setlidaropt=lambda *args: False)
    pipeline = Pipeline()
    assert not pipeline._setup(RotatedLidarSource(source))
    assert "SDK rejected option 0='COM8'" in pipeline.snapshot()["source_error"]


def test_new_source_clears_previous_connection_status(monkeypatch):
    pipeline = Pipeline()
    pipeline._publish(source_ok=True, hz=10.0, pts_per_scan=300,
                      points=[(0.1, 0.2)], track=(0.1, 0.2, 1.0), bg_ready=True)
    monkeypatch.setattr(pipeline, "_build_source", lambda: object())
    pipeline._handle_control(None, ("set_source", "real", "COM8", None, None))
    snap = pipeline.snapshot()
    assert not snap["source_ok"] and not snap["bg_ready"]
    assert snap["hz"] == snap["pts_per_scan"] == 0
    assert snap["points"] == [] and snap["track"] is None


def _posix_screen(monkeypatch, source_ok, port=""):
    monkeypatch.setattr(settings_screen, "EXCLUSIVE_PORTS", False)
    monkeypatch.setattr(settings_screen, "list_serial_ports", lambda: [
        {"path": "/dev/ttyUSB0", "kind": "usb"}])
    configured = []
    screen = SettingsScreen.__new__(SettingsScreen)
    screen.kind, screen.mode, screen.port = "real", "simple", port
    screen.probing, screen.probe_result, screen._probe_thread = False, None, None
    screen.pipeline = SimpleNamespace(
        snapshot=lambda: {"source_ok": source_ok, "hz": 10.0},
        configure_source=lambda *a, **kw: configured.append((a, kw)))
    screen.f_port = SimpleNamespace(text=port, commit=lambda: None)
    screen.settings = SimpleNamespace(source_port=port, seed=None, save=lambda: None)
    return screen, configured


def test_posix_probe_detects_and_adopts_streaming_lidar(monkeypatch):
    sniffed = []
    monkeypatch.setattr(settings_screen, "probe", lambda port, baud, secs: (
        sniffed.append(port) or {"ok": True, "hz": 10.0, "points_per_rev": 334,
                                 "error": None, "port": port}))
    screen, configured = _posix_screen(monkeypatch, source_ok=False)
    screen.start_probe()
    screen._probe_thread.join(2.0)
    assert sniffed == ["/dev/ttyUSB0"]
    assert screen.port == screen.f_port.text == "/dev/ttyUSB0"
    assert configured[0][1]["port"] == "/dev/ttyUSB0"
    lines = screen._status_lines({"source_ok": False})
    assert "lidar detected: /dev/ttyUSB0" in lines[0][0]


def test_posix_probe_skips_port_the_pipeline_is_streaming(monkeypatch):
    monkeypatch.setattr(settings_screen, "probe", lambda *a: pytest.fail(
        "must not sniff the live pipeline's port"))
    screen, _ = _posix_screen(monkeypatch, source_ok=True, port="/dev/ttyUSB0")
    screen.start_probe()
    assert screen.probe_result["live"] and screen._probe_thread is None
