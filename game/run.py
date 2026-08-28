"""CLI: python -m game.run --lidar sim|real|replay [--pointer lidar|mouse]
[--board-size S] [--image-x FILE] [--image-o FILE] [--resolution PX] [--seed].

Standalone desktop app: imports ONLY core/ (lidar + tracker + game state).
Two threads: main = pygame event loop; scan thread owns the LidarSource.
The thread boundary is a latest-wins pointer store + a control queue
(new_game, quit — drained between doProcessSimple calls).

Pointer source (--pointer, default lidar):
  lidar  moves are committed from the TRACKED position. In sim mode the mouse
         acts as the hand: it moves the simulated object, and the tap still
         comes out of the raycast -> noise -> tracking pipeline, exactly as a
         real finger would. In real mode the mouse is ignored entirely.
  mouse  legacy: the tap position is the raw mouse position. Handy for testing
         game rules without the tracker in the loop.
"""
from __future__ import annotations

import argparse
import threading
import time

import pygame

from core.config import NoiseConfig, SceneConfig
from core.game import GameState, TapDetector, Turn
from core.lidar_source import make_lidar_source
from core.scan import LaserScan
from core.tracking import Tracker

RESOLUTIONS = (480, 720, 1080, 1440, 2160)
# A mouse flick that moves the simulated object further than this in one scan
# cannot be followed by association (the tracker's gate is 0.5 m), so the track
# is dropped and re-acquired - the same thing the web viz does on click-drag.
MOUSE_JUMP_RESET = 0.5


def cell_of(x: float, y: float, board_size: float) -> tuple[int, int]:
    """Board coords -> 3x3 cell, clamped. Mirrors core.game.GameState.tap."""
    return (
        min(2, max(0, int(x * 3 / board_size))),
        min(2, max(0, int(y * 3 / board_size))),
    )


def main() -> None:
    p = argparse.ArgumentParser(description="Crosses & Noughts — YDLidar X3 desktop game")
    p.add_argument("--lidar", choices=["sim", "real", "replay"], default="sim")
    p.add_argument("--board-size", type=float, default=2.0)
    p.add_argument("--image-x", default=None)
    p.add_argument("--image-o", default=None)
    p.add_argument("--resolution", type=int, default=1080, choices=RESOLUTIONS)
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--port-dev", default=None, help="real lidar serial port")
    p.add_argument("--pointer", choices=["lidar", "mouse"], default="lidar",
                   help="commit moves from the tracked position (default) or the mouse")
    rec = p.add_argument_group("record / replay")
    rec.add_argument("--record", default=None, metavar="FILE",
                     help="record every scan from sim/real to a .npz file")
    rec.add_argument("--record-note", default="", help="free-text note stored in the recording")
    rec.add_argument("--replay-file", default=None, metavar="FILE",
                     help=".npz to play back (requires --lidar replay)")
    rec.add_argument("--replay-loop", dest="replay_loop", action="store_true", default=True)
    rec.add_argument("--no-replay-loop", dest="replay_loop", action="store_false",
                     help="play once, then stop")
    rec.add_argument("--replay-speed", type=float, default=1.0,
                     help="playback rate multiplier; 0 = free-run")
    args = p.parse_args()

    scene = SceneConfig(board_size=args.board_size)
    noise = NoiseConfig()
    source = make_lidar_source(
        args.lidar, scene=scene, noise=noise, seed=args.seed, port=args.port_dev,
        replay_path=args.replay_file, replay_loop=args.replay_loop,
        replay_speed=args.replay_speed, record_path=args.record,
        record_note=args.record_note,
    )
    tracker = Tracker(scene, noise)
    game = GameState()
    tap = TapDetector()
    pointer_mode = args.pointer
    # Sim sources expose a movable object; real and replay sources do not.
    sim_object = source if (
        hasattr(source, "set_object_pose") and hasattr(source, "object_pose")
    ) else None

    # thread boundaries: latest-wins stores + a control queue
    pointer: dict = {"x": None, "y": None}          # UI -> scan thread
    track: dict = {"x": None, "y": None, "conf": 0.0}  # scan thread -> UI
    pointer_lock = threading.Lock()
    track_lock = threading.Lock()
    control: list = []  # commands from UI thread: ("new_game",) | ("quit",)

    stop_event = threading.Event()

    def scan_loop() -> None:
        try:
            if not source.initialize():
                return
            if not source.turnOn():
                return
            tracker.set_background(source.calibrate_background())
            scan = LaserScan.blank()
            last_pose: tuple[float, float] | None = None
            while not stop_event.is_set():
                # control queue drained between doProcessSimple calls
                while control:
                    cmd = control.pop(0)
                    if cmd[0] == "new_game":
                        game.new_game()
                        tap.release()
                    elif cmd[0] == "quit":
                        return
                # In sim mode the mouse IS the hand: move the object so the tap
                # still comes out of the full pipeline. Off-board = hand lifted.
                # With real hardware (or a replay) there is no mouse at all, so
                # "hand on board" simply means "a track exists".
                with pointer_lock:
                    mx, my = pointer["x"], pointer["y"]
                mouse_drives_hand = (pointer_mode == "mouse") or (
                    pointer_mode == "lidar" and sim_object is not None
                )
                if mouse_drives_hand and pointer_mode == "lidar" and mx is not None:
                    if last_pose is None or (
                        (mx - last_pose[0]) ** 2 + (my - last_pose[1]) ** 2
                    ) > MOUSE_JUMP_RESET ** 2:
                        # Jumped further than the tracker can associate: drop
                        # the track so the next scan re-acquires the object
                        # instead of coasting onto mixed-pixel outliers.
                        tracker.reset()
                    last_pose = (mx, my)
                    sim_object.set_object_pose((mx, my))
                hand_on_board = (not mouse_drives_hand) or (mx is not None)
                if not source.doProcessSimple(scan):
                    if getattr(source, "exhausted", False):
                        return  # non-looping replay finished
                    stop_event.wait(0.05)
                    continue
                res = tracker.process(scan)
                # Dwell is measured against SCAN time, not the pointer's sample
                # timestamp: a perfectly still pointer produces no new events,
                # so a sample-stamped clock would freeze and never fire a tap.
                now = time.monotonic()
                if res is None or not hand_on_board:
                    # no track, or hand off the board: treat as a lifted finger
                    tap.release()
                    with track_lock:
                        track["x"], track["y"], track["conf"] = None, None, 0.0
                else:
                    with track_lock:
                        track["x"], track["y"], track["conf"] = res.x, res.y, res.conf
                    if pointer_mode == "lidar":
                        if res.missed > 0:
                            # Coasting on a prediction, not an observation. A
                            # jump puts a phantom dwell on the OLD cell, so a
                            # tap may only be committed from a live detection.
                            tap.release()
                        elif tap.update(res.x, res.y, scene.board_size, now):
                            game.tap(res.x, res.y, scene.board_size)
                    else:
                        with pointer_lock:
                            px, py = pointer["x"], pointer["y"]
                        if px is not None and tap.update(px, py, scene.board_size, now):
                            game.tap(px, py, scene.board_size)
                if getattr(source, "exhausted", False):
                    return
                stop_event.wait(0.005)
        finally:
            for meth in ("turnOff", "disconnecting", "close"):
                fn = getattr(source, meth, None)
                if fn is None:
                    continue
                try:
                    fn()
                except Exception:
                    pass

    thread = threading.Thread(target=scan_loop, name="scan-loop", daemon=True)
    thread.start()

    from game.render import GameRenderer

    renderer = GameRenderer(args.resolution, args.image_x, args.image_o)
    clock = pygame.time.Clock()
    running = True
    hover: tuple[int, int] | None = None
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                control.append(("quit",))
                running = False
            elif ev.type == pygame.KEYDOWN and ev.key == pygame.K_n:
                control.append(("new_game",))
            elif ev.type == pygame.MOUSEMOTION:
                mx, my = ev.pos
                # map window coords back to canvas coords
                ww, wh = renderer.window.get_size()
                scale = min(ww / renderer.resolution, wh / renderer.resolution)
                ox = (ww - renderer.resolution * scale) / 2
                oy = (wh - renderer.resolution * scale) / 2
                cx = (mx - ox) / scale
                cy = (my - oy) / scale
                if 0 <= cx < renderer.resolution and 0 <= cy < renderer.resolution:
                    bx = cx / renderer.resolution * scene.board_size
                    by = cy / renderer.resolution * scene.board_size
                    with pointer_lock:
                        pointer["x"] = bx
                        pointer["y"] = by
                    if pointer_mode == "mouse":
                        hover = cell_of(bx, by, scene.board_size)
                else:
                    with pointer_lock:
                        pointer["x"] = None
                        pointer["y"] = None
                    if pointer_mode == "mouse":
                        hover = None
                        tap.release()

        # In lidar mode the highlight follows what the tracker actually sees,
        # lag and all - not the raw mouse.
        if pointer_mode == "lidar":
            with track_lock:
                tx, ty = track["x"], track["y"]
            hover = cell_of(tx, ty, scene.board_size) if tx is not None else None
        with track_lock:
            tx, ty, conf = track["x"], track["y"], track["conf"]
        marker = None
        if tx is not None:
            px = tx / scene.board_size * renderer.resolution
            py = ty / scene.board_size * renderer.resolution
            marker = (px, py, conf)

        status = f"Turn: {game.turn.value}" if game.winner is None else f"{game.winner.value} wins!"
        renderer.draw(game.grid, game.turn, game.winner, hover, status, marker)
        clock.tick(60)

    stop_event.set()
    thread.join(timeout=3.0)
    pygame.quit()


if __name__ == "__main__":
    main()