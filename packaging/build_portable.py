"""Build a native, one-folder portable ZIP on the current operating system."""
from __future__ import annotations

import argparse
import importlib.util
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parent.parent
BUILD = ROOT / "build" / "portable"
DIST = ROOT / "dist"
APP_NAME = "CrossesNoughts"


def sdk_paths():
    return [
        ROOT / "YDLidar-SDK" / "build" / "python",
        ROOT / "YDLidar-SDK" / "build" / "python" / "Release",
        ROOT / "YDLidar-SDK" / "build" / "Release",
    ]


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-lidar", action="store_true",
                        help="require and bundle this platform's native YDLidar module")
    parser.add_argument("--clean", action="store_true")
    args = parser.parse_args()

    if args.clean:
        shutil.rmtree(BUILD, ignore_errors=True)
        shutil.rmtree(DIST / APP_NAME, ignore_errors=True)
    BUILD.mkdir(parents=True, exist_ok=True)
    DIST.mkdir(parents=True, exist_ok=True)

    paths = [path for path in sdk_paths() if path.is_dir()]
    env = os.environ.copy()
    env["PYTHONPATH"] = os.pathsep.join(
        [str(path) for path in paths] + [env.get("PYTHONPATH", "")])
    if args.with_lidar:
        search = [str(path) for path in paths] + sys.path
        if importlib.util.find_spec("ydlidar") is None and not any(
                (path / "ydlidar.py").exists() for path in paths):
            raise SystemExit(
                "No native YDLidar Python build found for this platform. "
                "Build the SDK first; see WINDOWS.md.")

    command = [
        sys.executable, "-m", "PyInstaller", str(ROOT / "run.py"),
        "--name", APP_NAME, "--onedir", "--noconfirm", "--windowed",
        "--distpath", str(DIST), "--workpath", str(BUILD),
        "--specpath", str(BUILD),
    ]
    for path in paths:
        command += ["--paths", str(path)]
    if args.with_lidar:
        command += ["--hidden-import", "ydlidar", "--hidden-import", "_ydlidar"]
        if os.name == "nt":
            separator = ";"
            for path in paths:
                for dll in path.glob("*.dll"):
                    command += ["--add-binary", f"{dll}{separator}."]
    subprocess.run(command, cwd=ROOT, env=env, check=True)

    system = {"Windows": "windows", "Linux": "linux",
              "Darwin": "macos"}.get(platform.system(), platform.system().lower())
    arch = platform.machine().lower().replace("amd64", "x86_64")
    suffix = "-lidar" if args.with_lidar else "-sim"
    archive = DIST / f"crosses-noughts-{system}-{arch}{suffix}"
    zip_path = Path(shutil.make_archive(str(archive), "zip", DIST, APP_NAME))
    print(f"Portable bundle: {zip_path}")


if __name__ == "__main__":
    main()
