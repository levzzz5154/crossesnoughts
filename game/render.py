"""pygame 2D renderer — the ONLY UI the game has. No browser, no canvas,
no trig. Grid + glyphs from image files (vector fallback) + hover + status +
the tracked pointer (drawn where the lidar actually sees the finger).
"""
from __future__ import annotations

from pathlib import Path

import pygame

from core.game import Turn

GRID = 3


class GameRenderer:
    def __init__(
        self,
        resolution: int = 1080,
        image_x: str | None = None,
        image_o: str | None = None,
    ):
        pygame.init()
        pygame.display.set_caption("Crosses & Noughts — YDLidar X3")
        self.resolution = resolution
        self.window = pygame.display.set_mode((resolution, resolution), pygame.RESIZABLE)
        self.canvas = pygame.Surface((resolution, resolution))
        self.font = pygame.font.SysFont("dejavusansmono, monospace", 28)
        self.small_font = pygame.font.SysFont("dejavusansmono, monospace", 20)
        self.cell = resolution // GRID
        self._load_glyphs(image_x, image_o)

    # -- glyphs -------------------------------------------------------------
    def _load_glyphs(self, image_x: str | None, image_o: str | None) -> None:
        self.glyph_x = self._load_glyph(image_x, (255, 120, 120))
        self.glyph_o = self._load_glyph(image_o, (120, 180, 255))

    def _load_glyph(self, path: str | None, color: tuple[int, int, int]) -> pygame.Surface:
        if path and Path(path).exists():
            img = pygame.image.load(path)
            return pygame.transform.smoothscale(img, (self.cell, self.cell))
        # vector fallback: X = two lines, O = circle
        surf = pygame.Surface((self.cell, self.cell), pygame.SRCALPHA)
        m = self.cell // 5
        if path == "x" or (path is None and color == (255, 120, 120)):
            pygame.draw.line(surf, color, (m, m), (self.cell - m, self.cell - m), max(4, self.cell // 24))
            pygame.draw.line(surf, color, (self.cell - m, m), (m, self.cell - m), max(4, self.cell // 24))
        else:
            pygame.draw.circle(surf, color, (self.cell // 2, self.cell // 2), self.cell // 2 - m, max(4, self.cell // 24))
        return surf

    # -- frame --------------------------------------------------------------
    def draw(
        self,
        grid,
        turn: Turn,
        winner: Turn | None,
        hover: tuple[int, int] | None,
        status_line: str,
        marker: tuple[float, float, float] | None = None,
    ) -> None:
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
                glyph = self.glyph_x if t is Turn.X else self.glyph_o
                c.blit(glyph, (cx * self.cell, cy * self.cell))
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
        # status line below the grid
        bar_y = GRID * self.cell + 8
        c.blit(self.font.render(status_line, True, (200, 215, 230)), (16, bar_y))
        c.blit(self.small_font.render("New Game: N", True, (120, 140, 160)), (16, bar_y + 34))
        # scale canvas to window
        ww, wh = self.window.get_size()
        scale = min(ww / self.resolution, wh / self.resolution)
        scaled = pygame.transform.smoothscale(c, (int(self.resolution * scale), int(self.resolution * scale)))
        self.window.fill((10, 12, 16))
        self.window.blit(scaled, ((ww - scaled.get_width()) // 2, (wh - scaled.get_height()) // 2))
        pygame.display.flip()