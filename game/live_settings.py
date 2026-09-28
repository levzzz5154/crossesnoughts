"""Live settings sync from the settings screen to its child game/preview window.

The child window is a separate process that owns the lidar, so the settings
screen cannot drive its pipeline directly. Instead the parent writes the full
settings as one JSON line to the child's stdin whenever they change (checked
every frame, so slider drags stream through), and the child applies each new
line to its own settings and pipeline.

Window geometry is per-process and never synced.
"""
from __future__ import annotations

import json
import threading
import time
from dataclasses import asdict, fields

from core.settings import GameSettings
from core.transforms import BoardAlignment

NOT_SYNCED = frozenset({"window_size", "fullscreen"})
# Sim raycasts the board geometry, so a size change rebuilds the sim source.
# While a slider is being dragged, wait for the value to settle first.
SIM_REBUILD_DELAY = 0.3


def _payload(settings: GameSettings) -> dict:
    return {k: v for k, v in asdict(settings).items() if k not in NOT_SYNCED}


class LiveSettingsSender:
    """Parent side: push settings to the child whenever they change."""

    def __init__(self, stream):
        self.stream = stream
        self._last: str | None = None

    def send(self, settings: GameSettings) -> None:
        # ASCII-escaped, so the pipe is immune to the child's locale code page.
        line = json.dumps(_payload(settings))
        if line == self._last or self.stream is None:
            return
        try:
            self.stream.write(line + "\n")
            self.stream.flush()
            self._last = line
        except (OSError, ValueError):
            # Child exited; nothing left to update. Close now so a later GC
            # flush of the unsent line cannot raise from a finalizer.
            stream, self.stream = self.stream, None
            try:
                stream.close()
            except (OSError, ValueError):
                pass


class LiveSettingsReceiver:
    """Child side: read settings lines from the parent and apply the newest."""

    def __init__(self, stream, settings: GameSettings, pipeline):
        self.settings = settings
        self.pipeline = pipeline
        self._latest: dict | None = None
        self._lock = threading.Lock()
        self._rebuild_at: float | None = None
        threading.Thread(target=self._read, args=(stream,),
                         name="live-settings", daemon=True).start()

    def _read(self, stream) -> None:
        for line in stream:
            try:
                data = json.loads(line)
            except ValueError:
                continue
            if isinstance(data, dict):
                with self._lock:
                    self._latest = data

    def poll(self) -> set[str]:
        """Apply any pending update; return the names of changed settings."""
        with self._lock:
            data, self._latest = self._latest, None
        changed = self.apply(data) if data is not None else set()
        if self._rebuild_at is not None and time.monotonic() >= self._rebuild_at:
            self._rebuild_at = None
            self.pipeline.set_board(self.settings.board_size, rebuild=True)
        return changed

    def apply(self, data: dict) -> set[str]:
        s = self.settings
        known = {f.name for f in fields(GameSettings)} - NOT_SYNCED
        incoming = {k: v for k, v in data.items() if k in known}
        # Validate through the dataclass so a bad line cannot poison anything.
        new = GameSettings(**{**asdict(s), **incoming})
        changed = {k for k in known if getattr(new, k) != getattr(s, k)}
        for k in changed:
            setattr(s, k, getattr(new, k))
        p = self.pipeline
        if {"source_kind", "source_port", "replay_file", "seed"} & changed:
            p.configure_source(s.source_kind, port=s.source_port or None,
                               replay_file=s.replay_file or None, seed=s.seed)
        if "tracking_mode" in changed:
            p.set_tracking_mode(s.tracking_mode)
        if "board_size" in changed:
            p.set_board(s.board_size, rebuild=False)
            if s.source_kind == "sim":
                self._rebuild_at = time.monotonic() + SIM_REBUILD_DELAY
        if "board_yaw_deg" in changed:
            p.set_yaw(s.board_yaw_deg)
        if {"board_offset_x", "board_offset_y", "tracking_flip_horizontal",
                "tracking_flip_vertical"} & changed:
            p.set_alignment(BoardAlignment(
                s.board_offset_x, s.board_offset_y,
                s.tracking_flip_horizontal, s.tracking_flip_vertical))
        return changed
