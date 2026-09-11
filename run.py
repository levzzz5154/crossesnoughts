"""Cross-platform launcher: ``python run.py [game options]``."""
from __future__ import annotations

import os
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parent
_DLL_HANDLES = []


def configure_bundled_sdk() -> None:
    """Expose a locally built YDLidar module to this and child processes."""
    candidates = [
        ROOT / "YDLidar-SDK" / "build" / "python",
        ROOT / "YDLidar-SDK" / "build" / "python" / "Release",
        ROOT / "YDLidar-SDK" / "build" / "Release",
    ]
    existing = [str(path) for path in candidates if path.is_dir()]
    for path in reversed(existing):
        if path not in sys.path:
            sys.path.insert(0, path)
    if existing:
        inherited = os.environ.get("PYTHONPATH")
        os.environ["PYTHONPATH"] = os.pathsep.join(
            existing + ([inherited] if inherited else []))
    if os.name == "nt" and hasattr(os, "add_dll_directory"):
        for path in candidates:
            if path.is_dir():
                _DLL_HANDLES.append(os.add_dll_directory(str(path)))
        if existing:
            os.environ["PATH"] = os.pathsep.join(
                existing + [os.environ.get("PATH", "")])


if __name__ == "__main__":
    os.chdir(ROOT)
    configure_bundled_sdk()
    from game.run import main
    main()
