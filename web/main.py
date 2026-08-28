"""FastAPI app: static mount / -> static/, WebSocket /ws, scan thread.

Two threads: (1) main thread runs uvicorn; (2) scan thread owns the
LidarSource exclusively (single-writer shutdown discipline — it calls
turnOff()/disconnecting() in its own finally). SceneModel is the shared
boundary. WS endpoint: per-connection asyncio task; hello + scene on
connect, then frames only when seq changed.
"""
from __future__ import annotations

import asyncio
import json
import threading
from contextlib import asynccontextmanager
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

from core.config import NoiseConfig, SceneConfig
from core.lidar_source import make_lidar_source
from core.scan import LaserScan
from core.tracking import Tracker
from web.frames import (
    SceneModel,
    build_frame,
    hello_message,
    scene_message,
)

STATIC_DIR = Path(__file__).parent / "static"

WS_POLL_TIMEOUT = 0.05  # 50 ms receive timeout
RECONNECT_BACKOFF = 1.0  # s (client side)


class ScanLoop:
    """Owns the LidarSource; runs in a dedicated thread."""

    def __init__(self, source, tracker: Tracker, model: SceneModel, scene: SceneConfig):
        self.source = source
        self.tracker = tracker
        self.model = model
        self.scene = scene
        self.stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        # Capability detection, not identity checks: a sim exposes a movable
        # object_pose, a replay exposes seek(). The recording wrapper delegates
        # both to whatever it wraps, so recording a sim keeps the viz working.
        self._sim = source if (
            hasattr(source, "set_object_pose") and hasattr(source, "object_pose")
        ) else None
        self._replay = source if hasattr(source, "seek") else None

    # -- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="scan-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    def _run(self) -> None:
        src = self.source
        try:
            if not src.initialize():
                return
            if not src.turnOn():
                return
            self.tracker.set_background(src.calibrate_background())
            scan = LaserScan.blank()
            while not self.stop_event.is_set():
                # Drain control commands between doProcessSimple calls.
                cmd = self.model.drain_control()
                if cmd is not None:
                    self._handle_control(cmd)
                if not src.doProcessSimple(scan):
                    if getattr(src, "exhausted", False):
                        break  # non-looping replay finished
                    if self.stop_event.wait(0.1):
                        break
                    continue
                tracked = self.tracker.process(scan)
                true_pose = None
                if self._sim is not None:
                    true_pose = self._sim.object_pose
                frame = build_frame(
                    self.model.current_seq(), scan, self.scene,
                    tracked=tracked, true_pose=true_pose,
                )
                self.model.publish(frame)
                if getattr(src, "exhausted", False):
                    break  # last frame of a non-looping replay was emitted
                # pacing sleep is stop_event.wait so shutdown is prompt
                self.stop_event.wait(0.005)
        finally:
            # single-writer shutdown: the scan thread owns the source
            for meth in ("turnOff", "disconnecting", "close"):
                fn = getattr(src, meth, None)
                if fn is None:
                    continue
                try:
                    fn()
                except Exception:
                    pass

    def _handle_control(self, cmd: dict) -> None:
        op = cmd.get("op")
        if op == "reset" and self._sim is not None:
            seed = int(cmd.get("seed", 0))
            self._sim.reset(seed)
            # re-calibrate background at the next scan boundary
            self.tracker.set_background(self.source.calibrate_background())
        elif op == "reset" and self._replay is not None:
            # restart the recording from the first scan and re-acquire
            self._replay.seek(0)
            self.tracker.reset()
        elif op == "set_object" and self._sim is not None:
            self._sim.set_object_pose((float(cmd["x"]), float(cmd["y"])))
            # the object jumped: drop the track so it re-acquires at the
            # new position (the 0.5 m association gate would otherwise
            # coast/latch a stale or mixed-pixel cluster)
            self.tracker.reset()


def create_app(
    scene: SceneConfig,
    noise: NoiseConfig,
    lidar_kind: str = "sim",
    seed: int | None = None,
    port: str | None = None,
    replay_path: str | None = None,
    replay_loop: bool = True,
    replay_speed: float = 1.0,
    record_path: str | None = None,
    record_note: str = "",
) -> FastAPI:
    source = make_lidar_source(
        lidar_kind, scene=scene, noise=noise, seed=seed, port=port,
        replay_path=replay_path, replay_loop=replay_loop, replay_speed=replay_speed,
        record_path=record_path, record_note=record_note,
    )
    tracker = Tracker(scene, noise)
    model = SceneModel(scene, scan_freq=source.scan_freq if hasattr(source, "scan_freq") else 10.0)
    loop = ScanLoop(source, tracker, model, scene)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        loop.start()
        yield
        loop.stop()

    app = FastAPI(lifespan=lifespan)

    @app.websocket("/ws")
    async def ws_endpoint(ws: WebSocket):
        await ws.accept()
        await ws.send_text(hello_message(scene, model.scan_freq))
        await ws.send_text(scene_message(scene))
        last_seq = 0
        try:
            while True:
                # poll for client commands with a short timeout
                try:
                    raw = await asyncio.wait_for(ws.receive_text(), timeout=WS_POLL_TIMEOUT)
                except asyncio.TimeoutError:
                    raw = None
                if raw is not None:
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        msg = None
                    if isinstance(msg, dict):
                        op = msg.get("type")
                        if op == "reset":
                            model.send_control("reset", seed=msg.get("seed", 0))
                        elif op == "set_object":
                            model.send_control(
                                "set_object",
                                x=msg.get("x", 0.0),
                                y=msg.get("y", 0.0),
                            )
                frame = model.latest(last_seq)
                if frame is not None:
                    last_seq = frame.seq
                    await ws.send_text(frame.to_json())
        except WebSocketDisconnect:
            pass

    app.mount("/", StaticFiles(directory=STATIC_DIR, html=True), name="static")

    return app