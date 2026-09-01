"""Pipeline: the one scan thread shared by settings, preview, and game.

Owns the LidarSource (wrapped in RotatedLidarSource), the selected tracker,
the scene and the game state. The UI thread never touches the source — it sends
control ops (drained between scans) and reads a latest-wins frame snapshot:

    frame = {
        "seq": int, "points": [(x, y), ...],   # downstream board coords
        "track": (x, y, conf) | None,
        "tracking_mode": "advanced" | "simple",
        "hz": float, "pts_per_scan": int,
        "source_ok": bool, "source_error": str | None,
        "bg_ready": bool, "calibrating": bool,
        "board_size": float, "yaw_deg": float,
    }

Control ops (tuples):
    ("set_source", kind, port, replay_file, seed)  -> rebuild source
    ("set_board", size)        -> new SceneConfig; sim rebuilt, others keep bg
    ("set_yaw", deg)           -> instant remap, no recalibration
    ("set_tracking_mode", mode)-> swap advanced/simple tracker
    ("calibrate",)             -> re-capture the background
    ("new_game",)              -> clear the grid
    ("quit",)                  -> stop the thread

The mouse-as-hand trick from the original game loop is preserved: in sim
mode the pointer target (downstream board coords) is rotated into the sim's
own frame and drives the simulated object, so a tap still comes out of the
full raycast -> noise -> tracking pipeline.
"""
from __future__ import annotations

import math
import threading
import time

from core.config import NoiseConfig, SceneConfig
from core.game import AppearanceDetector, GameState
from core.lidar_source import make_lidar_source
from core.scan import LaserScan, SCAN_MIN_RANGE, SCAN_MAX_RANGE
from core.tracking import SimpleTracker, Tracker
from core.yaw import RotatedLidarSource

# A mouse flick that moves the simulated object further than this in one scan
# cannot be followed by association (the tracker's gate is 0.5 m), so the
# track is dropped and re-acquired - the same thing the web viz does on
# click-drag.
MOUSE_JUMP_RESET = 0.5
# Ceiling for the reconnect backoff when a source fails to come up.
BACKOFF_MAX = 10.0


def cell_of(x: float, y: float, board_size: float) -> tuple[int, int]:
    """Board coords -> 3x3 cell, clamped. Mirrors core.game.GameState.tap."""
    return (
        min(2, max(0, int(x * 3 / board_size))),
        min(2, max(0, int(y * 3 / board_size))),
    )


class Pipeline:
    """One process, one scan thread, two consumers (settings screen, game)."""

    def __init__(
        self,
        pointer_mode: str = "lidar",
        record_path: str | None = None,
        record_note: str = "",
        replay_loop: bool = True,
        replay_speed: float = 1.0,
        board_size: float = 2.0,
        tracking_mode: str = "advanced",
    ):
        self.scene = SceneConfig(board_size=board_size)
        self.noise = NoiseConfig()
        if tracking_mode not in ("advanced", "simple"):
            raise ValueError("tracking_mode must be 'advanced' or 'simple'")
        self.tracking_mode = tracking_mode
        self.pointer_mode = pointer_mode
        self.record_path = record_path
        self.record_note = record_note
        self.replay_loop = replay_loop
        self.replay_speed = replay_speed

        self.source: RotatedLidarSource | None = None
        self.tracker = self._new_tracker()
        self.game = GameState()
        # Game moves are presence edges: first appearance commits instantly;
        # continued presence and disappearance do not mutate game state.
        self.tap = AppearanceDetector()
        self.phase = "settings"  # settings | preview | game (no taps outside game)

        self.source_spec: dict = {"kind": None}  # set via configure_source
        self.board_yaw_deg = 0.0

        # thread boundaries
        self.pointer: dict = {"x": None, "y": None}  # UI -> scan thread
        self._pointer_lock = threading.Lock()
        self._frame: dict = {
            "seq": 0, "points": [], "track": None, "hz": 0.0,
            "pts_per_scan": 0, "source_ok": False, "source_error": None,
            "bg_ready": False, "calibrating": False,
            "board_size": self.scene.board_size, "yaw_deg": 0.0,
            "tracking_mode": self.tracking_mode,
        }
        self._frame_lock = threading.Lock()
        self.control: list = []
        self.stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    # -- lifecycle -----------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="scan-loop", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self.control.append(("quit",))
        self.stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)

    # -- control ops (thread-safe: list append / pop) -------------------------
    def configure_source(
        self,
        kind: str,
        port: str | None = None,
        replay_file: str | None = None,
        seed: int | None = None,
    ) -> None:
        """Select the input source (takes effect at the next scan boundary)."""
        self.control.append(("set_source", kind, port, replay_file, seed))

    def set_board(self, size: float, rebuild: bool = True) -> None:
        """Queue a board-size change. When *rebuild* is False (live slider
        drag), only update the scene/tracker and publish — skip the sim-source
        rebuild until release fires set_board(rebuild=True)."""
        self.control.append(("set_board", float(size), rebuild))

    def set_yaw(self, deg: float) -> None:
        self.control.append(("set_yaw", float(deg)))

    def set_tracking_mode(self, mode: str) -> None:
        """Queue a tracker switch; it is applied between scans."""
        if mode not in ("advanced", "simple"):
            raise ValueError("tracking mode must be 'advanced' or 'simple'")
        self.control.append(("set_tracking_mode", mode))

    def recalibrate(self) -> None:
        self.control.append(("calibrate",))

    def new_game(self) -> None:
        self.control.append(("new_game",))

    def set_pointer(self, x: float | None, y: float | None) -> None:
        with self._pointer_lock:
            self.pointer["x"], self.pointer["y"] = x, y

    def snapshot(self) -> dict:
        with self._frame_lock:
            return dict(self._frame)

    def _publish(self, **kw) -> None:
        with self._frame_lock:
            self._frame.update(kw)

    def _new_tracker(self):
        if self.tracking_mode == "simple":
            return SimpleTracker(self.scene, self.noise)
        return Tracker(self.scene, self.noise)

    # -- source construction (scan thread only) -------------------------------
    def _build_source(self) -> RotatedLidarSource:
        spec = self.source_spec
        inner = make_lidar_source(
            spec["kind"], scene=self.scene, noise=self.noise,
            seed=spec.get("seed"), port=spec.get("port") or None,
            replay_path=spec.get("replay_file") or None,
            replay_loop=self.replay_loop,
            replay_speed=self.replay_speed, record_path=self.record_path,
            record_note=self.record_note,
        )
        w = RotatedLidarSource(inner)
        w.yaw = math.radians(self.board_yaw_deg)
        return w

    def _teardown(self, src) -> None:
        for meth in ("turnOff", "disconnecting", "close"):
            fn = getattr(src, meth, None)
            if fn is None:
                continue
            try:
                fn()
            except Exception:
                pass

    # -- scan thread -----------------------------------------------------------
    def _run(self) -> None:
        backoff = 0.0
        while not self.stop_event.is_set():
            # drain control BEFORE requiring a source: the first
            # set_source op is what creates it
            while self.control:
                op = self.control.pop(0)
                self._handle_control(self.source, op)
                if self.stop_event.is_set():
                    return
            src = self.source
            if src is None:
                if self.source_spec.get("kind"):
                    self._publish(source_ok=False,
                                  source_error="starting source...")
                else:
                    self._publish(source_ok=False, source_error=None)
                if self.stop_event.wait(0.2):
                    break
                continue
            try:
                if not self._setup(src):
                    # Back off instead of reconnecting once a second forever:
                    # a device that is unplugged, busy or returning garbage
                    # must not be re-initialized in a tight loop. The error is
                    # published, so the settings screen shows it and the user
                    # can switch to Sim without restarting the app.
                    backoff = min(backoff * 2 if backoff else 1.0, BACKOFF_MAX)
                    if self.stop_event.wait(backoff):
                        break
                    continue
                backoff = 0.0
                self._scan_loop(src)
            finally:
                self._teardown(src)

    def _setup(self, src) -> bool:
        try:
            if not src.initialize():
                self._publish(source_ok=False, bg_ready=False,
                              source_error="initialize() failed - port busy, missing, or wrong permissions")
                return False
            if not src.turnOn():
                self._publish(source_ok=False, bg_ready=False,
                              source_error="turnOn() failed")
                return False
        except Exception as e:
            self._publish(source_ok=False, bg_ready=False,
                          source_error=f"{type(e).__name__}: {e}")
            return False
        source_hz = float(getattr(src, "scan_freq", 0.0) or 0.0)
        if self.tracking_mode == "simple":
            # Simple mode is purely geometric: board size + yaw define the
            # usable area, so it must not wait for or consume a calibration.
            self._publish(source_ok=True, source_error=None, hz=source_hz,
                          bg_ready=True,
                          calibrating=False)
        else:
            self._publish(source_ok=True, source_error=None, hz=source_hz,
                          bg_ready=False,
                          calibrating=True)
            try:
                self.tracker.set_background(src.calibrate_background())
            except Exception as e:
                self._publish(calibrating=False, bg_ready=False,
                              source_error=f"background calibration failed: {e}")
                return False
            self._publish(calibrating=False, bg_ready=True)
        return True

    def _scan_loop(self, src) -> None:
        scan = LaserScan.blank()
        last_pose: tuple[float, float] | None = None
        while not self.stop_event.is_set():
            # control ops drained between doProcessSimple calls
            while self.control:
                op = self.control.pop(0)
                if self._handle_control(src, op):
                    return  # rebuild: teardown + re-init from self.source
            # --- mouse as hand (sim only): move the object BEFORE capturing
            # the scan, exactly like the original game loop - otherwise the
            # tracker sees the pre-jump pose after a jump reset and
            # re-acquires at the OLD position.
            sim = src if (hasattr(src, "set_object_pose")
                          and hasattr(src, "object_pose")) else None
            with self._pointer_lock:
                mx, my = self.pointer["x"], self.pointer["y"]
            mouse_drives_hand = (self.pointer_mode == "mouse") or (sim is not None)
            if sim is not None and mx is not None:
                r = math.hypot(mx, my)
                th = math.atan2(mx, my) + src.yaw
                sx, sy = r * math.sin(th), r * math.cos(th)
                if last_pose is None or (
                    (sx - last_pose[0]) ** 2 + (sy - last_pose[1]) ** 2
                ) > MOUSE_JUMP_RESET ** 2:
                    # jumped further than the tracker can associate: drop the
                    # track so the next scan re-acquires instead of coasting
                    # onto mixed-pixel outliers
                    sim.set_object_pose((sx, sy))
                    self.tracker.reset()
                else:
                    sim.set_object_pose((sx, sy))
                last_pose = (sx, sy)
            hand_on_board = (not mouse_drives_hand) or (mx is not None)

            if not src.doProcessSimple(scan):
                if getattr(src, "exhausted", False):
                    return  # non-looping replay finished
                if self.stop_event.wait(0.05):
                    return
                continue
            res = self.tracker.process(scan)
            now = time.monotonic()
            if res is None or not hand_on_board:
                self.tap.release()
                track = None
            else:
                # clamp the tracked position to the board bounds so the
                # pointer ring / cell highlight / tap can never land outside
                # the board — the tracker can coast past the edge on mixed-
                # pixel pulls or prediction overshoot.
                cx = max(0.0, min(self.scene.board_size, res.x))
                cy = max(0.0, min(self.scene.board_size, res.y))
                track = (cx, cy, res.conf)
                # taps only fire in the game phase
                if self.phase == "game" and self.pointer_mode == "lidar":
                    # A missed scan is still part of the current presence;
                    # only a fully lost track above rearms the next one.
                    if res.missed == 0 and self.tap.update(
                            cx, cy, self.scene.board_size, now):
                        self.game.tap(cx, cy, self.scene.board_size)
                elif self.phase == "game" and self.pointer_mode == "mouse":
                    if (mx is not None and res.missed == 0 and
                            self.tap.update(mx, my, self.scene.board_size, now)):
                        self.game.tap(mx, my, self.scene.board_size)

            # --- publish the frame (downstream board coords)
            pts = []
            for p in scan.points[: scan.size]:
                if SCAN_MIN_RANGE < p.range <= SCAN_MAX_RANGE:
                    pts.append((p.range * math.sin(p.angle),
                                p.range * math.cos(p.angle)))
            with self._frame_lock:
                self._frame["seq"] += 1
                self._frame["points"] = pts
                self._frame["track"] = track
                self._frame["hz"] = float(scan.scanFreq or 0.0)
                self._frame["pts_per_scan"] = int(scan.size)
                self._frame["board_size"] = self.scene.board_size
                self._frame["yaw_deg"] = self.board_yaw_deg
            if getattr(src, "exhausted", False):
                return
            self.stop_event.wait(0.005)

    def _handle_control(self, src, op) -> bool:
        """Returns True when the caller must rebuild (teardown + re-init)."""
        cmd = op[0]
        if cmd == "quit":
            self.stop_event.set()
            return True
        if cmd == "set_source":
            _, kind, port, replay_file, seed = op
            self.source_spec = {"kind": kind, "port": port,
                                "replay_file": replay_file, "seed": seed}
            self.scene = SceneConfig(board_size=self.scene.board_size)
            self.tracker = self._new_tracker()
            self.game.new_game()
            self.tap.release()
            self.source = self._build_source()
            self._publish(source_error=None, bg_ready=False)
            return True
        if cmd == "set_board":
            size = op[1]
            rebuild = op[2] if len(op) > 2 else True
            self.scene = SceneConfig(board_size=size)
            self.tracker = self._new_tracker()
            self._publish(board_size=size)
            if rebuild and self.source_spec.get("kind") == "sim":
                # the sim raycasts the scene geometry: rebuild it
                self.source = self._build_source()
                return True
            # Real/replay background does not depend on board size.  The
            # simple tracker has no background at all.
            if src is not None and self.tracking_mode == "advanced":
                self.tracker.set_background(src.rotated_background())
            return False
        if cmd == "set_tracking_mode":
            self.tracking_mode = op[1]
            self.tracker = self._new_tracker()
            if src is not None and self.tracking_mode == "advanced":
                try:
                    self.tracker.set_background(src.rotated_background())
                except RuntimeError:
                    # The source may not have completed its first calibration
                    # yet.  If it is already live, capture one now; otherwise
                    # the next setup will provide it.
                    try:
                        self.tracker.set_background(src.calibrate_background())
                    except Exception:
                        pass
            self._publish(tracking_mode=self.tracking_mode, track=None,
                          bg_ready=(self.tracking_mode == "simple"))
            return False
        if cmd == "set_yaw":
            deg = op[1] % 360.0
            self.board_yaw_deg = deg
            if src is not None:
                src.yaw = math.radians(deg)
                if self.tracking_mode == "advanced":
                    self.tracker.set_background(src.rotated_background())
            self._publish(yaw_deg=deg)
            return False
        if cmd == "calibrate":
            if src is None:
                return False
            if self.tracking_mode == "simple":
                # There is intentionally no calibration step in simple mode.
                self._publish(calibrating=False, bg_ready=True)
                return False
            self._publish(calibrating=True, bg_ready=False)
            try:
                self.tracker.set_background(src.calibrate_background())
                self._publish(calibrating=False, bg_ready=True)
            except Exception as e:
                self._publish(calibrating=False, bg_ready=False,
                              source_error=f"background calibration failed: {e}")
            return False
        if cmd == "new_game":
            self.game.new_game()
            self.tap.release()
            return False
        return False
