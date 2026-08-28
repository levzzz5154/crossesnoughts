"""Game state machine tests (plan §14 step 10, §17.11/12)."""
import pytest

from core.game import GameState, TapDetector, Turn

S = 2.0


def center(cx, cy):
    return ((cx + 0.5) / 3 * S, (cy + 0.5) / 3 * S)


def test_dwell_tap_and_rifire():
    d = TapDetector(dwell=0.3)
    assert not d.update(*center(0, 0), S, 0.0)
    assert not d.update(*center(0, 0), S, 0.2)
    assert d.update(*center(0, 0), S, 0.31)  # dwell completes
    assert not d.update(*center(0, 0), S, 0.4)  # no auto-repeat
    d.release()
    assert not d.update(*center(0, 0), S, 0.5)  # re-entry resets
    assert d.update(*center(0, 0), S, 0.81)  # re-fire after dwell


def test_dwell_needs_an_advancing_clock():
    """Regression guard for the frozen-dwell bug.

    TapDetector measures dwell against whatever clock the caller passes. Feed it
    a pointer SAMPLE timestamp (which only changes when the pointer moves) and a
    perfectly still pointer can never complete a dwell. The scan loop must pass
    scan time instead.
    """
    frozen = 10.0
    stuck = TapDetector(dwell=0.3)
    for _ in range(50):  # pointer parked on a cell, no new samples
        assert not stuck.update(*center(1, 1), S, frozen)
    # same parked pointer, clock advancing as it does across scans
    ok = TapDetector(dwell=0.3)
    assert not ok.update(*center(1, 1), S, frozen)
    assert not ok.update(*center(1, 1), S, frozen + 0.20)
    assert ok.update(*center(1, 1), S, frozen + 0.35)


def test_dwell_still_fires_without_pointer_motion():
    """A still pointer that is fed scan time commits exactly one move."""
    g = GameState()
    d = TapDetector(dwell=0.3)
    t = 0.0
    fired = 0
    for _ in range(8):  # 8 scans at 10 Hz, pointer never moves
        t += 0.1
        if d.update(*center(2, 2), S, t):
            g.tap(*center(2, 2), S)
            fired += 1
    assert fired == 1, "dwell should fire once, not zero times and not repeatedly"
    assert g.grid[2][2] == Turn.X


def test_release_cancels_dwell():
    """Losing the track (or lifting the hand) abandons a partial dwell."""
    d = TapDetector(dwell=0.3)
    assert not d.update(*center(0, 1), S, 0.0)
    assert not d.update(*center(0, 1), S, 0.2)
    d.release()  # track lost
    assert not d.update(*center(0, 1), S, 0.9)  # dwell restarted, not completed


def test_tap_maps_to_cells():
    g = GameState()
    assert g.tap(*center(0, 0), S)
    assert g.grid[0][0] == Turn.X
    assert g.tap(*center(1, 0), S)
    assert g.grid[0][1] == Turn.O
    assert g.tap(*center(2, 2), S)
    assert g.grid[2][2] == Turn.X


def test_edge_clamp():
    g = GameState()
    assert g.tap(S - 0.01, 0.01, S)  # near right/top corner
    assert g.grid[0][2] == Turn.X


def test_no_tap_outside_board():
    g = GameState()
    assert not g.tap(-0.1, 0.5, S)
    assert not g.tap(2.1, 0.5, S)
    assert not g.tap(0.5, -0.1, S)
    assert not g.tap(0.5, 2.1, S)
    assert g.move_count == 0


def test_turn_toggles():
    g = GameState()
    g.tap(*center(0, 0), S)
    assert g.turn == Turn.O
    g.tap(*center(1, 0), S)
    assert g.turn == Turn.X

def test_overwrite_any_cell():
    g = GameState()
    g.tap(*center(0, 0), S)  # X
    g.tap(*center(1, 1), S)  # O
    g.tap(*center(1, 1), S)  # X overwrites O's cell — allowed
    assert g.grid[1][1] == Turn.X
    assert g.winner is None


def test_overwrite_breaks_win_line():
    g = GameState()
    for c in [center(0, 0), center(1, 0), center(0, 1), center(1, 1), center(0, 2)]:
        g.tap(*c, S)
    assert g.winner == Turn.X
    assert g.win_line == [(0, 0), (1, 0), (2, 0)]
    # win locks the game: further taps ignored
    assert not g.tap(*center(2, 2), S)


def test_no_draw_state():
    g = GameState()
    # X: (0,0),(0,1),(2,0),(2,1),(1,2); O: (1,0),(1,1),(0,2),(2,2)
    # -> full grid with no 3-in-a-row: no draw state, game continues
    moves = [(0, 0), (1, 0), (0, 1), (1, 1), (2, 0), (0, 2), (2, 1), (2, 2), (1, 2)]
    for cx, cy in moves:
        g.tap(*center(cx, cy), S)
    assert g.winner is None
    assert g.move_count == 9
    # continues: next tap overwrites
    g.tap(*center(0, 0), S)
    assert g.grid[0][0] == Turn.O  # turn after 9 moves is O
def test_new_game_resets():
    g = GameState()
    for c in [center(0, 0), center(1, 0), center(0, 1), center(1, 1), center(0, 2)]:
        g.tap(*c, S)
    g.new_game()
    assert g.winner is None
    assert g.turn == Turn.X
    assert all(cell is None for row in g.grid for cell in row)
    assert g.move_count == 0


def test_events_emitted():
    g = GameState()
    g.tap(*center(0, 0), S)
    kinds = [e.kind for e in g.events]
    assert "tap" in kinds
    assert g.events[0].turn == Turn.X
    assert g.events[0].cell == (0, 0)
    for c in [center(1, 0), center(0, 1), center(1, 1), center(0, 2)]:
        g.tap(*c, S)
    assert any(e.kind == "win" for e in g.events)
    g.new_game()
    assert any(e.kind == "new_game" for e in g.events)


def test_inactive_ignores_taps():
    g = GameState()
    for c in [center(0, 0), center(1, 0), center(0, 1), center(1, 1), center(0, 2)]:
        g.tap(*c, S)
    assert g.winner is not None
    assert not g.tap(*center(2, 2), S)
    assert g.move_count == 5