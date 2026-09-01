"""GameSettings: everything the settings screen edits, persisted to JSON at
the repo root (settings.json) so the next launch starts where you left off.

Web-free, UI-free — game/ and web/ can both read it. Unknown keys in the
file are ignored (forward compatibility); values are validated on load so a
hand-edited file cannot poison the pipeline.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

from core.config import BOARD_SIZE_MAX, BOARD_SIZE_MIN

SETTINGS_FILENAME = "settings.json"


def _settings_path() -> Path:
    return Path(__file__).resolve().parent.parent / SETTINGS_FILENAME


@dataclass
class GameSettings:
    # calibration
    board_size: float = 2.0          # m, square, [BOARD_SIZE_MIN, BOARD_SIZE_MAX]
    board_yaw_deg: float = 0.0       # 0..360, lidar-frame angle of the board window
    # input source: "sim" | "real" | "replay"
    source_kind: str = "sim"
    source_port: str = ""            # real: serial port ("" = default /dev/ttyUSB0)
    replay_file: str = ""            # replay: path to .npz
    seed: int | None = None          # sim RNG seed
    tracking_mode: str = "advanced"  # "advanced" background tracker | "simple" board-only
    # teams
    team_x_name: str = "Player X"
    team_o_name: str = "Player O"
    image_x: str = ""                # path to X logo/glyph image
    image_o: str = ""                # path to O logo/glyph image
    # window
    window_size: tuple[int, int] = (1080, 1080)
    fullscreen: bool = False

    def __post_init__(self) -> None:
        self.normalize()

    def normalize(self) -> None:
        self.board_size = float(
            min(BOARD_SIZE_MAX, max(BOARD_SIZE_MIN, float(self.board_size)))
        )
        self.board_yaw_deg = float(self.board_yaw_deg % 360.0)
        if self.source_kind not in ("sim", "real", "replay"):
            self.source_kind = "sim"
        if self.tracking_mode not in ("advanced", "simple"):
            self.tracking_mode = "advanced"
        self.team_x_name = str(self.team_x_name)[:40] or "Player X"
        self.team_o_name = str(self.team_o_name)[:40] or "Player O"
        w, h = self.window_size
        self.window_size = (
            int(min(3840, max(320, int(w)))),
            int(min(3840, max(320, int(h)))),
        )

    # -- persistence ---------------------------------------------------------
    def save(self, path: Path | str | None = None) -> Path:
        path = Path(path) if path is not None else _settings_path()
        path.write_text(
            json.dumps(asdict(self), indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        return path

    @classmethod
    def load(cls, path: Path | str | None = None) -> "GameSettings":
        path = Path(path) if path is not None else _settings_path()
        data: dict = {}
        if path.exists():
            try:
                raw = json.loads(path.read_text(encoding="utf-8"))
                if isinstance(raw, dict):
                    data = raw
            except (json.JSONDecodeError, OSError):
                data = {}  # corrupt file -> defaults, never a crash
        known = {f.name for f in fields(cls)}
        kwargs = {k: v for k, v in data.items() if k in known}
        if isinstance(kwargs.get("window_size"), list):
            kwargs["window_size"] = tuple(kwargs["window_size"])
        if kwargs.get("seed") == "":
            kwargs["seed"] = None
        return cls(**kwargs)
