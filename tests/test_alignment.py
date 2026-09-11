"""Calibration geometry and tracker integration without hardware."""
import numpy as np
import pytest

from core.config import SceneConfig, NoiseConfig
from core.scan import LaserPoint, LaserScan
from core.settings import GameSettings
from core.tracking import SimpleTracker, Tracker
from core.transforms import BoardAlignment
from game.pipeline import Pipeline


@pytest.mark.parametrize('horizontal,vertical', [(False, False), (True, False), (False, True), (True, True)])
def test_offset_then_center_flip_and_inverse(horizontal, vertical):
    alignment = BoardAlignment(1.0, -0.5, horizontal, vertical)
    actual = alignment.to_board(1.2, 0.2, 2.0)
    assert actual == pytest.approx((1.8 if horizontal else 0.2, 1.3 if vertical else 0.7))
    assert alignment.from_board(*actual, 2.0) == pytest.approx((1.2, 0.2))
    assert alignment.to_board(2.0, 0.5, 2.0) == (1.0, 1.0)


def test_simple_tracker_filters_shifted_board_before_tracking():
    tracker = SimpleTracker(SceneConfig(board_size=1.0), NoiseConfig())
    tracker.alignment = BoardAlignment(2.0, -0.5, True, True)
    # First point lies in the shifted board, second only in the old footprint.
    points = [LaserPoint(angle=float(np.arctan2(x, y)), range=float(np.hypot(x, y)))
              for x, y in [(2.2, 0.2), (0.5, 0.5)]]
    result = tracker.process(LaserScan(points=points, size=2))
    assert result is not None
    assert (result.x, result.y) == pytest.approx((0.8, 0.3))


def test_advanced_centroid_uses_alignment_without_changing_measured_range():
    tracker = Tracker(SceneConfig(), NoiseConfig())
    angles, ranges = np.array([0.4, 0.41, 0.42]), np.array([1.0, 1.0, 1.0])
    x, y, r = tracker._centroid(angles, ranges)
    tracker.alignment = BoardAlignment(-0.3, 0.2, True, True)
    actual = tracker._centroid(angles, ranges)
    assert actual == pytest.approx((2.0 - (x + 0.3), 2.0 - (y - 0.2), r))


def test_alignment_survives_tracker_rebuild_and_resets_track():
    p = Pipeline(tracking_mode='simple')
    alignment = BoardAlignment(0.4, -0.2, True, False)
    p._handle_control(None, ('set_alignment', alignment))
    p._handle_control(None, ('set_board', 1.0, False))
    assert p.tracker.alignment == alignment
    p._handle_control(None, ('set_tracking_mode', 'advanced'))
    assert p.tracker.alignment == alignment
    assert p.snapshot()['track'] is None


def test_settings_alignment_roundtrip_and_validation(tmp_path):
    settings = GameSettings(board_offset_x=0.2, board_offset_y=0.7,
                            tracking_flip_horizontal=True, tracking_flip_vertical=True)
    assert GameSettings.load(settings.save(tmp_path / 'settings.json')) == settings
    settings = GameSettings(board_offset_x=float('nan'), board_offset_y='bad',
                            tracking_flip_horizontal='false')
    assert settings.board_offset_x == settings.board_offset_y == 0.0
    assert settings.tracking_flip_horizontal is False


def test_settings_offsets_clamped_to_one_metre():
    settings = GameSettings(board_offset_x=-0.1, board_offset_y=1.2)
    assert settings.board_offset_x == 0.0
    assert settings.board_offset_y == 1.0
