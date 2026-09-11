import pygame

from game.preview import TrackingPreview


class _Pipeline:
    source_spec = {"kind": "sim"}


class _Settings:
    pass


def test_preview_uses_same_centered_full_window_square_as_game():
    pygame.font.init()
    surface = pygame.Surface((1920, 1080))
    preview = TrackingPreview(surface, _Pipeline(), _Settings())
    preview._layout(1.0)
    assert preview.board_rect == pygame.Rect(420, 0, 1080, 1080)


def test_preview_board_y_matches_game_top_to_bottom_direction():
    pygame.font.init()
    surface = pygame.Surface((800, 600))
    preview = TrackingPreview(surface, _Pipeline(), _Settings())
    preview._layout(2.0)
    assert preview._board_point(preview.board_rect.topleft, 2.0) == (0.0, 0.0)
    assert preview._board_point(
        (preview.board_rect.right - 1, preview.board_rect.bottom - 1), 2.0
    )[1] > 1.99
