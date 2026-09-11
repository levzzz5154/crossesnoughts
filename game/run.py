"""CLI: python -m game.run [flags]

Standalone desktop app. Two phases in one process, sharing one scan thread
(game/pipeline.py):
  1. SETTINGS — source picker with live device detection, calibration
     (board size + window angle + tracking check), team names/logos.
  2. GAME — the crosses & noughts board; ESC returns to settings, F toggles
     fullscreen, N starts a new game.
  3. PREVIEW — optional board-only tracking view; ESC returns to settings.

Settings persist to settings.json (core/settings.py); CLI flags override on
launch. --skip-settings jumps straight into the game (automation/tests).

Pointer source (--pointer, default lidar):
  lidar  moves are committed from the TRACKED position. In sim mode the mouse
         acts as the hand (on the game canvas, or on the settings radar).
  mouse  legacy: the tap position is the raw mouse position. Handy for
         testing game rules without the tracker in the loop.
"""
from __future__ import annotations

import argparse
import subprocess
import sys

import pygame

from core.settings import GameSettings
from core.transforms import BoardAlignment
from game.pipeline import Pipeline, cell_of


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Crosses & Noughts — YDLidar X3 desktop game")
    p.add_argument("--lidar", choices=["sim", "real", "replay"], default=None,
                   help="preselect the input source (settings screen can change it)")
    p.add_argument("--board-size", type=float, default=None)
    p.add_argument("--image-x", default=None)
    p.add_argument("--image-o", default=None)
    p.add_argument("--resolution", type=int, default=None,
                   help="square window size in px (any monitor resolution)")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--port-dev", default=None, help="real lidar serial port")
    p.add_argument("--pointer", choices=["lidar", "mouse"], default="lidar",
                   help="commit moves from the tracked position (default) or the mouse")
    p.add_argument("--tracking-mode", choices=["advanced", "simple"], default=None,
                   help="tracker implementation (simple = board area only)")
    p.add_argument("--preview", action="store_true",
                   help="open the board-only tracking preview instead of the game")
    p.add_argument("--skip-settings", action="store_true",
                   help="start in the game phase directly")
    p.add_argument("--game-window", action="store_true", help=argparse.SUPPRESS)
    p.add_argument("--settings-file", default=None, metavar="FILE",
                   help="alternative settings.json path (default: repo root)")
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
    return p.parse_args()


def apply_cli_overrides(settings: GameSettings, args) -> None:
    if args.lidar:
        settings.source_kind = args.lidar
    if args.board_size is not None:
        settings.board_size = args.board_size
    if args.image_x is not None:
        settings.image_x = args.image_x
    if args.image_o is not None:
        settings.image_o = args.image_o
    if args.resolution is not None:
        settings.window_size = (args.resolution, args.resolution)
    if args.seed is not None:
        settings.seed = args.seed
    if args.tracking_mode is not None:
        settings.tracking_mode = args.tracking_mode
    if args.port_dev is not None:
        settings.source_port = args.port_dev
    if args.replay_file is not None:
        settings.replay_file = args.replay_file
        if args.lidar is None:
            settings.source_kind = "replay"
    settings.normalize()


def game_phase(window, pipeline: Pipeline, settings: GameSettings) -> str:
    """Run the game until ESC (back to settings) or QUIT. Returns 'settings'
    or 'quit'."""
    from game.render import GameRenderer

    renderer = GameRenderer(window, settings.image_x or None,
                            settings.image_o or None)
    clock = pygame.time.Clock()
    scene_size = settings.board_size
    running = True
    while running:
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                pipeline.set_pointer(None, None)
                return "quit"
            if ev.type == pygame.VIDEORESIZE:
                settings.window_size = (ev.w, ev.h)
                settings.save()
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_n:
                    pipeline.new_game()
                elif ev.key == pygame.K_f:
                    pygame.display.toggle_fullscreen()
                    settings.fullscreen = not settings.fullscreen
                    settings.save()
                elif ev.key == pygame.K_ESCAPE:
                    pipeline.set_pointer(None, None)
                    return "settings"
            elif ev.type == pygame.MOUSEMOTION:
                mx, my = ev.pos
                # map window coords back to canvas coords
                ww, wh = window.get_size()
                res = renderer.resolution
                scale = min(ww / res, wh / res)
                ox = (ww - res * scale) / 2
                oy = (wh - res * scale) / 2
                cx = (mx - ox) / scale
                cy = (my - oy) / scale
                if 0 <= cx < res and 0 <= cy < res:
                    bx = cx / res * scene_size
                    by = cy / res * scene_size
                    pipeline.set_pointer(bx, by)
                else:
                    pipeline.set_pointer(None, None)

        snap = pipeline.snapshot()
        scene_size = snap.get("board_size", scene_size)
        if scene_size != settings.board_size:
            settings.board_size = scene_size  # keep in sync for next launch

        track = snap.get("track")
        hover = None
        if track is not None:
            tx, ty, _conf = track
            hover = cell_of(tx, ty, scene_size)
        marker = None
        if track is not None:
            tx, ty, conf = track
            res = renderer.resolution
            marker = (tx / scene_size * res, ty / scene_size * res, conf)

        game = pipeline.game
        if game.winner is None:
            turn_name = settings.team_x_name if game.turn.value == "X" \
                else settings.team_o_name
            status = f"{turn_name}'s turn  ({game.turn.value})"
        else:
            winner_name = settings.team_x_name if game.winner.value == "X" \
                else settings.team_o_name
            status = f"{winner_name} wins!"
        renderer.draw(game.grid, game.turn, game.winner, game.win_line,
                      hover, status, marker,
                      team_x=settings.team_x_name,
                      team_o=settings.team_o_name)
        clock.tick(60)
    return "quit"


def tracking_preview_phase(window, pipeline: Pipeline,
                           settings: GameSettings) -> str:
    """Run the live board-only tracking preview until ESC or QUIT."""
    from game.preview import TrackingPreview

    preview = TrackingPreview(window, pipeline, settings)
    return preview.run()


def main() -> None:
    args = parse_args()
    settings = GameSettings.load(args.settings_file)
    apply_cli_overrides(settings, args)
    settings.save()

    pygame.init()
    pygame.display.set_caption("Crosses & Noughts — YDLidar X3")
    window = pygame.display.set_mode(settings.window_size, pygame.RESIZABLE)

    pipeline = Pipeline(pointer_mode=args.pointer,
                        record_path=args.record,
                        record_note=args.record_note,
                        replay_loop=args.replay_loop,
                        replay_speed=args.replay_speed,
                        board_size=settings.board_size,
                        tracking_mode=settings.tracking_mode,
                        board_yaw_deg=settings.board_yaw_deg,
                        alignment=BoardAlignment(settings.board_offset_x, settings.board_offset_y,
                                                 settings.tracking_flip_horizontal,
                                                 settings.tracking_flip_vertical))
    pipeline.configure_source(
        settings.source_kind, port=settings.source_port or None,
        replay_file=settings.replay_file or None, seed=settings.seed)
    pipeline.start()

    from game.settings_screen import SettingsScreen

    phase = ("preview" if args.preview else
             ("game" if args.skip_settings else "settings"))
    pipeline.phase = phase
    if phase == "game":
        pipeline.new_game()

    try:
        while True:
            if phase == "settings":
                child_process = None

                def resume_settings_pipeline():
                    pipeline.phase = "settings"
                    pipeline.configure_source(
                        settings.source_kind,
                        port=settings.source_port or None,
                        replay_file=settings.replay_file or None,
                        seed=settings.seed)
                    pipeline.start()

                def launch_window(preview=False):
                    nonlocal child_process
                    if child_process is not None and child_process.poll() is None:
                        return
                    settings.save()
                    pipeline.stop()
                    if getattr(sys, "frozen", False):
                        command = [sys.executable, "--skip-settings",
                                   "--game-window", "--pointer", args.pointer]
                    else:
                        command = [sys.executable, "-m", "game.run",
                                   "--skip-settings", "--game-window",
                                   "--pointer", args.pointer]
                    if preview:
                        command.append("--preview")
                    if args.settings_file:
                        command += ["--settings-file", args.settings_file]
                    child_process = subprocess.Popen(command)

                def launch_game_window():
                    launch_window(preview=False)

                def launch_preview_window():
                    launch_window(preview=True)

                def poll_child_window():
                    nonlocal child_process
                    if child_process is not None and child_process.poll() is not None:
                        child_process = None
                        resume_settings_pipeline()

                screen = SettingsScreen(window, pipeline, settings,
                                        on_start=launch_game_window,
                                        on_preview=launch_preview_window,
                                        on_tick=poll_child_window)
                result = screen.run()
                if child_process is not None and child_process.poll() is None:
                    child_process.terminate()
                    child_process.wait(timeout=3)
                if result == "quit":
                    break
                if result == "preview":
                    phase = "preview"
                    pipeline.phase = "preview"
                else:
                    phase = "game"
                    pipeline.phase = "game"
                    pipeline.new_game()
            elif phase == "preview":
                result = tracking_preview_phase(window, pipeline, settings)
                if args.game_window:
                    break
                if result == "quit":
                    break
                phase = "settings"
                pipeline.phase = "settings"
            else:
                result = game_phase(window, pipeline, settings)
                if args.game_window:
                    break
                if result == "quit":
                    break
                phase = "settings"
                pipeline.phase = "settings"
    finally:
        pipeline.stop()
        settings.save()
        pygame.quit()


if __name__ == "__main__":
    main()
