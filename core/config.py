"""Scene and noise configuration dataclasses.

Every tunable lives here. board_size is a single square scalar validated to
[1.0, 3.0] m; object_radius defaults to 0.10 m.
"""
from __future__ import annotations

from dataclasses import dataclass, field

BOARD_SIZE_MIN = 1.0
BOARD_SIZE_MAX = 2.0
@dataclass(frozen=True)
class SceneConfig:
    """Geometry of the scene: square board + surrounding box + object."""

    board_size: float = 2.0  # S, square side in metres, [1.0, 3.0]
    object_radius: float = 0.10  # tracked pointer radial signature, m
    box_wall_gap: float = 1.0  # box walls sit at S/2 + gap from board centre
    box_height: float = 2.4  # wall height, m
    box_wall_thickness: float = 0.01  # raycast wall thickness, m

    def __post_init__(self) -> None:
        if not (BOARD_SIZE_MIN <= self.board_size <= BOARD_SIZE_MAX):
            raise ValueError(
                f"board_size must be in [{BOARD_SIZE_MIN}, {BOARD_SIZE_MAX}] m, "
                f"got {self.board_size}"
            )
        if self.object_radius <= 0:
            raise ValueError(f"object_radius must be > 0, got {self.object_radius}")

    @property
    def box_half(self) -> float:
        """Half-side of the outer box in board frame (S/2 + gap)."""
        return self.board_size / 2 + self.box_wall_gap

    @property
    def box_w(self) -> float:
        """Outer box side length (S + 2*gap), matches hello schema box_w/box_d."""
        return self.board_size + 2 * self.box_wall_gap

    @property
    def box_wall_inner(self) -> float:
        """Inner face of a box wall in board coords: -gap or S + gap."""
        return self.box_wall_gap

    @property
    def box_wall_outer(self) -> float:
        return self.box_wall_gap + self.box_wall_thickness


@dataclass(frozen=True)
class NoiseConfig:
    """Lidar noise parameters (Alhashimi ICINCO 2016 heteroscedastic model)."""

    range_std_a: float = 0.004  # m, constant term
    range_std_b: float = 0.002  # m / m^2, quadratic term
    angular_jitter_std: float = 0.0017  # rad (~0.1 deg)
    dropout_rate: float = 0.02
    ghost_rate: float = 0.001
    ghost_std: float = 0.15  # m
    mixed_pixel_rate: float = 0.15
    min_range: float = 0.10  # m, blind spot; below -> invalid (0.0)
    max_range: float = 8.0  # m, no-hit returns > this -> invalid
    enabled: bool = True

    def range_std(self, d: float) -> float:
        """Range noise std at distance d (m): a + b*d^2."""
        return self.range_std_a + self.range_std_b * d * d