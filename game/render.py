"""pygame 2D renderer for the game — no browser, no canvas, no trig.

Grid + glyph images (vector fallback) + hover + tracked pointer + team
names + a win overlay (winner name, big logo, play-again hint). The canvas
is re-sized to the square inscribed in the window, so any monitor/resolution
renders crisp at any window size.
"""
from __future__ import annotations

from pathlib import Path

import pygame

from core.game import Turn

GRID = 3


class GameRenderer:
    def __init__(
        self,
        window: pygame.Surface,
        image_x: str | None = None,
        image_o: str | None = None,
    ):
        self.window = window
        self.font = pygame.font.SysFont("dejavusansmono, monospace", 30)
        self.small_font = pygame.font.SysFont("dejavusansmono, monospace", 22)
        self.big_font = pygame.font.SysFont("dejavusansmono, monospace", 54)
        self._image_x = image_x
        self._image_o = image_o
        self._glyph_cache: dict[tuple[str, int], pygame.Surface] = {}
        self.resize()

    # -- canvas --------------------------------------------------------------
    def _target_res(self) -> int:
        ww, wh = self.window.get_size()
        return max(240, min(ww, wh - 96))

    def resize(self) -> None:
        """Square canvas inscribed in the window (status bar at the bottom)."""
        self.resolution = self._target_res()
        self.canvas = pygame.Surface((self.resolution, self.resolution))
        self.cell = self.resolution // GRID

    # -- glyphs --------------------------------------------------------------
    def _glyph(self, kind: Turn, size: int) -> pygame.Surface:
        key = (kind.value, size)
        cached = self._glyph_cache.get(key)
        if cached is not None:
            return cached
        path = self._image_x if kind is Turn.X else self._image_o
        if path and Path(path).exists():
            try:
                img = pygame.image.load(path).convert_alpha()
                surf = pygame.transform.smoothscale(img, (size, size))
            except Exception:
                surf = self._vector_glyph(kind, size)
        else:
            surf = self._vector_glyph(kind, size)
        self._glyph_cache[key] = surf
        return surf

    def _vector_glyph(self, kind: Turn, size: int) -> pygame.Surface:
        color = (255, 120, 120) if kind is Turn.X else (120, 180, 255)
        surf = pygame.Surface((size, size), pygame.SRCALPHA)
        m = size // 5
        if kind is Turn.X:
            w = max(4, size // 24)
            pygame.draw.line(surf, color, (m, m), (size - m, size - m), w)
            pygame.draw.line(surf, color, (size - m, m), (m, size - m), w)
        else:
            pygame.draw.circle(surf, color, (size // 2, size // 2),
                               size // 2 - m, max(4, size // 24))
        return surf

    def set_images(self, image_x: str | None, image_o: str | None) -> None:
        if (image_x, image_o) != (self._image_x, self._image_o):
            self._image_x, self._image_o = image_x, image_o
            self._glyph_cache.clear()

    # -- frame --------------------------------------------------------------
    def draw(
        self,
        grid,
        turn: Turn,
        winner: Turn | None,
        win_line: list[tuple[int, int]] | None,
        hover: tuple[int, int] | None,
        status_line: str,
        marker: tuple[float, float, float] | None = None,
        team_x: str = "Player X",
        team_o: str = "Player O",
    ) -> None:
        if self.resolution != self._target_res():
            self.resize()
        c = self.canvas
        c.fill((18, 22, 28))
        # grid
        for i in range(1, GRID):
            x = i * self.cell
            pygame.draw.line(c, (90, 110, 130), (x, 0), (x, GRID * self.cell), 3)
            pygame.draw.line(c, (90, 110, 130), (0, x), (GRID * self.cell, x), 3)
        # glyphs
        for cy in range(GRID):
            for cx in range(GRID):
                t = grid[cy][cx]
                if t is None:
                    continue
                c.blit(self._glyph(t, self.cell), (cx * self.cell, cy * self.cell))
        # winning line highlight
        if winner is not None and win_line:
            (x1, y1), (x2, y2) = win_line[0], win_line[-1]
            a = ((x1 + 0.5) * self.cell, (y1 + 0.5) * self.cell)
            b = ((x2 + 0.5) * self.cell, (y2 + 0.5) * self.cell)
            pygame.draw.line(c, (110, 220, 140), a, b, max(6, self.cell // 20))
        # hover highlight
        if hover is not None and winner is None:
            hx, hy = hover
            s = pygame.Surface((self.cell, self.cell), pygame.SRCALPHA)
            s.fill((255, 255, 255, 24))
            c.blit(s, (hx * self.cell, hy * self.cell))
        # tracked pointer: a hollow ring at the position the lidar reports,
        # dimming as track confidence falls (conf = exp(-missed/3)).
        if marker is not None:
            mx, my, conf = marker
            k = max(0.0, min(1.0, float(conf)))
            col = (int(90 + 165 * k), int(80 + 130 * k), int(70 + 50 * k))
            r = max(6, self.cell // 12)
            pygame.draw.circle(c, col, (int(mx), int(my)), r, max(2, r // 5))
            pygame.draw.circle(c, col, (int(mx), int(my)), max(2, r // 6))
        # win overlay
        if winner is not None:
            self._draw_win_overlay(c, winner, team_x, team_o)
        # status line below the grid
        bar_y = GRID * self.cell + 8
        c.blit(self.font.render(status_line, True, (200, 215, 230)), (16, bar_y))
        c.blit(self.small_font.render(
            "New Game: N    Fullscreen: F    Settings: Esc", True,
            (120, 140, 160)), (16, bar_y + 42))
        # scale canvas to window
        ww, wh = self.window.get_size()
        scale = min(ww / self.resolution, wh / self.resolution)
        scaled = pygame.transform.smoothscale(
            c, (int(self.resolution * scale), int(self.resolution * scale)))
        self.window.fill((10, 12, 16))
        self.window.blit(scaled, ((ww - scaled.get_width()) // 2,
                                  (wh - scaled.get_height()) // 2))
        pygame.display.flip()

    def _draw_win_overlay(self, c: pygame.Surface, winner: Turn,
                          team_x: str, team_o: str) -> None:
        dim = pygame.Surface(c.get_size(), pygame.SRCALPHA)
        dim.fill((10, 12, 16, 170))
        c.blit(dim, (0, 0))
        name = team_x if winner is Turn.X else team_o
        res = self.resolution
        # big logo centered in the upper half
        logo = self._glyph(winner, int(res * 0.42))
        c.blit(logo, logo.get_rect(center=(res // 2, int(res * 0.36))))
        # winner name
        title = self.big_font.render(f"{name} wins!", True, (240, 245, 250))
        c.blit(title, title.get_rect(center=(res // 2, int(res * 0.70))))
        hint = self.font.render("N — play again", True, (130, 190, 150))
        c.blit(hint, hint.get_rect(center=(res // 2, int(res * 0.82))))
