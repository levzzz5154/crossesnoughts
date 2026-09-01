"""Minimal live preview for the board-only tracker."""
from __future__ import annotations

import pygame

from core.settings import GameSettings
from game.pipeline import Pipeline


BG = (14, 18, 23)
BOARD = (38, 48, 58)
GRID = (104, 122, 138)
POINT = (55, 232, 192)
TRACK = (255, 80, 82)
TEXT = (214, 226, 238)
DIM = (132, 150, 168)
GOOD = (110, 220, 140)
WARN = (240, 190, 110)
LIDAR = (255, 160, 60)


class TrackingPreview:
    """Draw the current scan and tracked position in board coordinates.

    The preview intentionally has no second coordinate transform: the
    pipeline has already applied yaw and reports board coordinates.  In sim
    mode moving the mouse over the board moves the simulated object, which
    makes the simple tracker easy to exercise without hardware.
    """

    def __init__(self, window: pygame.Surface, pipeline: Pipeline,
                 settings: GameSettings):
        self.window = window
        self.pipeline = pipeline
        self.settings = settings
        self.font = pygame.font.SysFont("dejavusansmono, monospace", 28)
        self.small = pygame.font.SysFont("dejavusansmono, monospace", 20)
        self.tiny = pygame.font.SysFont("dejavusansmono, monospace", 16)
        self.clock = pygame.time.Clock()
        self.board_rect = pygame.Rect(0, 0, 10, 10)

    def _layout(self, board_size: float) -> None:
        w, h = self.window.get_size()
        side = max(180, min(w - 80, h - 190))
        self.board_rect = pygame.Rect(
            (w - side) // 2, 82, side, side,
        )

    def _board_point(self, pos, board_size: float):
        if not self.board_rect.collidepoint(pos):
            return None
        r = self.board_rect
        x = (pos[0] - r.left) / r.width * board_size
        # Board y grows away from the lidar, so it grows upward on screen.
        y = (r.bottom - pos[1]) / r.height * board_size
        return max(0.0, min(board_size, x)), max(0.0, min(board_size, y))

    def run(self) -> str:
        while True:
            snap = self.pipeline.snapshot()
            size = float(snap.get("board_size", self.settings.board_size))
            self._layout(size)
            for ev in pygame.event.get():
                if ev.type == pygame.QUIT:
                    self.pipeline.set_pointer(None, None)
                    return "quit"
                if ev.type == pygame.KEYDOWN:
                    if ev.key == pygame.K_ESCAPE:
                        self.pipeline.set_pointer(None, None)
                        return "settings"
                    if ev.key == pygame.K_f:
                        pygame.display.toggle_fullscreen()
                        self.settings.fullscreen = not self.settings.fullscreen
                        self.settings.save()
                elif ev.type == pygame.MOUSEMOTION:
                    p = self._board_point(ev.pos, size)
                    if self.pipeline.source_spec.get("kind") == "sim" and p is not None:
                        self.pipeline.set_pointer(*p)
                    else:
                        self.pipeline.set_pointer(None, None)

            self._draw(snap, size)
            pygame.display.flip()
            self.clock.tick(60)

    def _to_screen(self, x: float, y: float):
        r = self.board_rect
        return (
            int(r.left + x / max(self._size, 1e-9) * r.width),
            int(r.bottom - y / max(self._size, 1e-9) * r.height),
        )

    def _draw(self, snap: dict, size: float) -> None:
        self._size = size
        s = self.window
        s.fill(BG)
        title = self.font.render("Lidar tracking preview", True, TEXT)
        s.blit(title, (24, 20))

        r = self.board_rect
        pygame.draw.rect(s, BOARD, r)
        pygame.draw.rect(s, GRID, r, 2)
        for i in (1, 2):
            x = r.left + r.width * i // 3
            y = r.top + r.height * i // 3
            pygame.draw.line(s, GRID, (x, r.top), (x, r.bottom), 1)
            pygame.draw.line(s, GRID, (r.left, y), (r.right, y), 1)

        for x, y in snap.get("points") or []:
            if 0.0 <= x <= size and 0.0 <= y <= size:
                pygame.draw.circle(s, POINT, self._to_screen(x, y), 2)

        # Board origin / lidar position.
        pygame.draw.circle(s, LIDAR, self._to_screen(0.0, 0.0), 6)
        pygame.draw.circle(s, LIDAR, self._to_screen(0.0, 0.0), 11, 1)

        track = snap.get("track")
        if track is not None:
            x, y, conf = track
            p = self._to_screen(x, y)
            radius = max(9, r.width // 45)
            pygame.draw.circle(s, TRACK, p, radius, 3)
            pygame.draw.line(s, TRACK, (p[0] - radius - 5, p[1]),
                             (p[0] + radius + 5, p[1]), 1)
            pygame.draw.line(s, TRACK, (p[0], p[1] - radius - 5),
                             (p[0], p[1] + radius + 5), 1)

        mode = snap.get("tracking_mode", self.settings.tracking_mode)
        ready = "ready" if snap.get("source_ok") else "starting"
        ready_col = GOOD if snap.get("source_ok") else WARN
        lines = [
            (f"mode: {mode}    board: {size:.2f} m    yaw: "
             f"{snap.get('yaw_deg', 0.0):.0f} deg", TEXT),
            (f"source: {self.pipeline.source_spec.get('kind') or '—'}  "
             f"{ready}  {snap.get('hz') or 0.0:.1f} Hz", ready_col),
        ]
        if track is None:
            lines.append(("track: —", DIM))
        else:
            x, y, conf = track
            lines.append((f"track: ({x:.3f}, {y:.3f}) m  conf {conf:.2f}",
                          TRACK))
        for i, (label, col) in enumerate(lines):
            s.blit(self.small.render(label, True, col), (24, 118 + i * 28))

        hint = "Move mouse over board in Sim to move the test point  |  Esc: settings  F: fullscreen"
        text = self.tiny.render(hint, True, DIM)
        s.blit(text, (24, s.get_height() - text.get_height() - 22))

