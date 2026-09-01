"""GameState + TapDetector: pure, headless-testable game logic (web-free).

Rules: 3x3 grid; turns alternate X/O after each tap; a tap may overwrite the
OPPONENT'S cell (that's the point of this variant) and the turn flips as
usual; re-tapping a cell that already holds your OWN mark is a NO-OP (no
move, no turn flip — you don't waste your turn on your own glyph); win =
3-in-a-row checked after every move (overwrites can break lines, so no
per-cell lock); no draw state; New Game clears the grid and resets turn to X.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

GRID = 3  # 3x3


class Turn(Enum):
    X = "X"
    O = "O"


@dataclass(frozen=True)
class GameEvent:
    kind: str  # "tap" | "win" | "new_game" | "overwrite"
    x: float = 0.0
    y: float = 0.0
    cell: tuple[int, int] | None = None
    turn: Turn | None = None
    line: list[tuple[int, int]] | None = None


WIN_LINES = [
    [(r, c) for c in range(GRID)] for r in range(GRID)
] + [
    [(r, c) for r in range(GRID)] for c in range(GRID)
] + [
    [(i, i) for i in range(GRID)],
    [(i, GRID - 1 - i) for i in range(GRID)],
]


class GameState:
    """3x3 board with turn alternation, overwrite, win detection."""

    def __init__(self) -> None:
        self.grid: list[list[Optional[Turn]]] = [
            [None] * GRID for _ in range(GRID)
        ]
        self.turn: Turn = Turn.X
        self.winner: Optional[Turn] = None
        self.win_line: list[tuple[int, int]] | None = None
        self.move_count = 0
        self.events: list[GameEvent] = []

    def _emit(self, ev: GameEvent) -> None:
        self.events.append(ev)

    def tap(self, x: float, y: float, board_size: float) -> bool:
        """Map pointer (board coords) to a cell and commit the move.

        Returns True if a move was committed. cell = floor(x*3/S),
        floor(y*3/S) clamped to [0,2]; pointer outside [0,S]^2 is not a tap.
        Tapping a cell that already holds the CURRENT player's own mark is a
        no-op (returns False, turn unchanged) — overwriting the opponent's
        mark remains allowed and commits normally.
        """
        if self.winner is not None:
            return False
        if not (0.0 <= x <= board_size and 0.0 <= y <= board_size):
            return False
        cell_x = min(GRID - 1, max(0, int(x * GRID / board_size)))
        cell_y = min(GRID - 1, max(0, int(y * GRID / board_size)))
        cell = (cell_x, cell_y)
        prev = self.grid[cell_y][cell_x]
        if prev is self.turn:
            return False  # own-cell tap: no-op, turn does NOT flip
        self.grid[cell_y][cell_x] = self.turn
        self.move_count += 1
        if prev is not None:
            self._emit(GameEvent("overwrite", x, y, cell, self.turn))
        self._emit(GameEvent("tap", x, y, cell, self.turn))
        line = self._check_win()
        if line is not None:
            self.winner = self.turn
            self.win_line = line
            self._emit(GameEvent("win", x, y, cell, self.turn, line))
        else:
            self.turn = Turn.O if self.turn is Turn.X else Turn.X
        return True

    def _check_win(self) -> list[tuple[int, int]] | None:
        for line in WIN_LINES:
            vals = [self.grid[r][c] for (r, c) in line]
            if vals[0] is not None and vals[0] == vals[1] == vals[2]:
                return line
        return None

    def new_game(self) -> None:
        self.grid = [[None] * GRID for _ in range(GRID)]
        self.turn = Turn.X
        self.winner = None
        self.win_line = None
        self.move_count = 0
        self._emit(GameEvent("new_game"))

    @property
    def active(self) -> bool:
        return self.winner is None


class TapDetector:
    """Dwell-based tap detection.

    A tap is a dwell: the pointer must stay within one cell for 0.3 s, then
    the move commits. Releasing and re-entering a cell re-fires (no
    auto-repeat while dwelling). Taps ignored while the game is inactive.
    """

    DWELL = 0.3  # s

    def __init__(self, dwell: float = DWELL):
        self.dwell = dwell
        self._cell: tuple[int, int] | None = None
        self._entered: float | None = None
        self._fired_cell: tuple[int, int] | None = None

    def update(self, x: float, y: float, board_size: float, t: float) -> bool:
        """Feed the latest pointer position (board coords).

        Returns True when a tap fires (dwell completed in the same cell).
        """
        if not (0.0 <= x <= board_size and 0.0 <= y <= board_size):
            self._cell = None
            self._entered = None
            return False
        cell_x = min(GRID - 1, max(0, int(x * GRID / board_size)))
        cell_y = min(GRID - 1, max(0, int(y * GRID / board_size)))
        cell = (cell_x, cell_y)
        if cell != self._cell:
            self._cell = cell
            self._entered = t
            self._fired_cell = None
            return False
        if self._fired_cell == cell:
            return False  # no auto-repeat while dwelling
        if t - self._entered >= self.dwell:
            self._fired_cell = cell
            return True
        return False

    def release(self) -> None:
        """Pointer left the board or the game went inactive: reset dwell."""
        self._cell = None
        self._entered = None
        self._fired_cell = None


class AppearanceDetector:
    """Fire once when a player first appears, with no dwell.

    A continuous track is one presence.  Movement while present is ignored;
    losing the track only arms the next appearance and never emits a tap.
    ``t`` is accepted for drop-in use by the scan loop, but is intentionally
    unused.
    """

    def __init__(self):
        self._present = False

    def update(self, x: float, y: float, board_size: float,
               t: float | None = None) -> bool:
        if not (0.0 <= x <= board_size and 0.0 <= y <= board_size):
            self._present = False
            return False
        if self._present:
            return False
        self._present = True
        return True

    def release(self) -> None:
        """Arm the detector for the next appearance."""
        self._present = False
