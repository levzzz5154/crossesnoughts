"""Lidar noise model: angular jitter, heteroscedastic range noise, dropout,
ghosts, blind spot, and mixed pixels. Applied inside SimLidarSource.
"""
from __future__ import annotations

import numpy as np

from core.config import NoiseConfig


def apply_noise(
    angles: np.ndarray,
    ranges: np.ndarray,
    noise: NoiseConfig,
    rng: np.random.Generator,
    second_closest: np.ndarray | None = None,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply noise to a clean raycast.

    Returns (ranges_out, angles_out). angles_out is the jittered angle grid
    (jitter applied BEFORE raycast by the caller; here only the measured
    angles are returned). ranges_out: 0.0 marks invalid (dropout, ghost
    beyond max, blind spot, or no hit).

    Order (plan §6): angular jitter (caller) -> raycast -> range noise ->
    dropout -> ghost -> blind spot -> mixed pixels.
    """
    if not noise.enabled:
        return ranges.copy(), angles.copy()

    r = ranges.copy()
    a = angles.copy()

    # (3) Range noise: heteroscedastic, sigma = a + b*d^2.
    valid = r > 0.0
    sigma = np.where(valid, noise.range_std(r), 0.0)
    r = np.where(valid, r + rng.normal(0.0, sigma), r)

    # (4) Dropout -> invalid.
    r = np.where(rng.random(r.shape) < noise.dropout_rate, 0.0, r)

    # (5) Ghost: rare large positive excursion.
    ghost = rng.random(r.shape) < noise.ghost_rate
    r = np.where(ghost & (r > 0.0), np.abs(r + rng.normal(0.0, noise.ghost_std, r.shape)), r)

    # (6) Blind spot / max range -> invalid.
    r = np.where((r > 0.0) & (r < noise.min_range), 0.0, r)
    r = np.where(r > noise.max_range, 0.0, r)

    # (7) Mixed pixels: silhouette bins blend toward the second-closest hit.
    if second_closest is not None and noise.mixed_pixel_rate > 0.0:
        mix = rng.random(r.shape) < noise.mixed_pixel_rate
        pull = np.where(mix & (r > 0.0), r + 0.5 * (second_closest - r), r)
        r = np.where(mix & (r > 0.0), pull, r)

    return r, a