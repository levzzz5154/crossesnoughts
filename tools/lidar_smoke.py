r"""Check 100 actual frames through the desktop pipeline without opening a UI.

Usage: .venv\Scripts\python tools/lidar_smoke.py COM8
Close the application first; this check owns the selected serial port.
"""
from __future__ import annotations

import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from game.pipeline import Pipeline


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "COM8"
    pipeline = Pipeline(tracking_mode="simple")
    pipeline.configure_source("real", port=port)
    pipeline.start()
    start = time.monotonic()
    snapshot = {}
    try:
        while time.monotonic() - start < 15.0:
            snapshot = pipeline.snapshot()
            if snapshot["source_ok"] and snapshot["seq"] >= 100:
                break
            time.sleep(0.05)
        result = {key: snapshot.get(key) for key in
                  ("source_ok", "source_error", "seq", "hz", "pts_per_scan", "bg_ready")}
        result["board_points"] = len(snapshot.get("points", []))
        result["elapsed"] = round(time.monotonic() - start, 2)
        print(json.dumps(result), flush=True)
        if not result["source_ok"] or result["source_error"] or result["seq"] < 100:
            raise RuntimeError("real lidar pipeline did not capture 100 frames")
    finally:
        pipeline.stop()


if __name__ == "__main__":
    main()
