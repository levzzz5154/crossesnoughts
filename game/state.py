"""GameState + TapDetector — pure, headless-testable (re-export of core/game).

The game imports ONLY core/ (lidar + tracker + game state): no FastAPI, no
uvicorn, no WS, no browser.
"""
from core.game import GameEvent, GameState, TapDetector, Turn  # noqa: F401