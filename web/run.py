"""CLI: python -m web.run [--lidar sim|real|replay] [--board-size S] [--port]
[--seed] [--host] [--record FILE] [--replay-file FILE]
"""
from __future__ import annotations

import argparse

import uvicorn

from core.config import NoiseConfig, SceneConfig
from web.main import create_app


def main() -> None:
    p = argparse.ArgumentParser(description="YDLidar X3 web visualizer")
    p.add_argument("--lidar", choices=["sim", "real", "replay"], default="sim")
    p.add_argument("--board-size", type=float, default=2.0)
    p.add_argument("--port", type=int, default=8000)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--seed", type=int, default=None)
    p.add_argument("--port-dev", default=None, help="real lidar serial port")
    rec = p.add_argument_group("record / replay")
    rec.add_argument("--record", default=None, metavar="FILE",
                     help="record every scan from sim/real to a .npz file")
    rec.add_argument("--record-note", default="", help="free-text note stored in the recording")
    rec.add_argument("--replay-file", default=None, metavar="FILE",
                     help=".npz to play back (requires --lidar replay)")
    rec.add_argument("--replay-loop", dest="replay_loop", action="store_true", default=True,
                     help="loop the recording forever (default)")
    rec.add_argument("--no-replay-loop", dest="replay_loop", action="store_false",
                     help="play once, then stop")
    rec.add_argument("--replay-speed", type=float, default=1.0,
                     help="playback rate multiplier; 0 = free-run as fast as possible")
    args = p.parse_args()

    if args.lidar == "replay" and args.replay_speed <= 0.0:
        print(
            "note: --replay-speed 0 free-runs the recording faster than the "
            "browser can poll, so the viz will sample only a fraction of the "
            "scans. Use it for headless analysis, not for watching."
        )

    scene = SceneConfig(board_size=args.board_size)
    noise = NoiseConfig()
    app = create_app(
        scene, noise,
        lidar_kind=args.lidar, seed=args.seed, port=args.port_dev,
        replay_path=args.replay_file,
        replay_loop=args.replay_loop,
        replay_speed=args.replay_speed,
        record_path=args.record,
        record_note=args.record_note,
    )
    uvicorn.run(app, host=args.host, port=args.port, log_level="info")


if __name__ == "__main__":
    main()
