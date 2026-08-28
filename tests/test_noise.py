"""Noise model tests (plan §14 step 4): determinism, bias, dropout, sigma."""
import numpy as np
import pytest

from core.config import NoiseConfig
from core.noise import apply_noise


def _run(noise, seed, n=20000):
    rng = np.random.default_rng(seed)
    angles = np.zeros(n)
    ranges = np.full(n, 2.0)  # flat wall at 2 m
    return apply_noise(angles, ranges, noise, rng)


def test_fixed_seed_determinism():
    a = _run(NoiseConfig(), 7)
    b = _run(NoiseConfig(), 7)
    assert np.array_equal(a[0], b[0])


def test_mean_bias_under_1mm():
    r, _ = _run(NoiseConfig(), 3)
    valid = r > 0
    assert abs(np.mean(r[valid]) - 2.0) < 0.001


def test_dropout_within_half_pp():
    r, _ = _run(NoiseConfig(), 5)
    drop = np.mean(r == 0)
    assert abs(drop - 0.02) < 0.005


def test_sigma_grows_with_distance():
    rng = np.random.default_rng(1)
    noise = NoiseConfig()
    r_far = apply_noise(np.zeros(20000), np.full(20000, 2.0), noise, rng)[0]
    r_near = apply_noise(np.zeros(20000), np.full(20000, 0.5), noise, rng)[0]
    std_far = np.std(r_far[r_far > 0])
    std_near = np.std(r_near[r_near > 0])
    assert std_far > std_near
    # quadratic model: sigma(2) = 0.004 + 0.002*4 = 0.012
    assert abs(std_far - 0.012) < 0.002


def test_disabled_noise_is_identity():
    r, a = _run(NoiseConfig(enabled=False), 9)
    assert np.allclose(r, 2.0)


def test_blind_spot_flags_short():
    rng = np.random.default_rng(2)
    noise = NoiseConfig(ghost_rate=0.0)  # ghosts would rescue short reads
    r, _ = apply_noise(np.zeros(1000), np.full(1000, 0.05), noise, rng)
    assert np.all(r == 0.0)  # below min_range -> invalid


def test_ghost_rate():
    rng = np.random.default_rng(4)
    noise = NoiseConfig(ghost_rate=0.01, ghost_std=0.5)
    r, _ = apply_noise(np.zeros(50000), np.full(50000, 2.0), noise, rng)
    # ghosted values are 2.0 +- 0.5, far beyond the 3-sigma noise band (0.036)
    ghosts = np.mean(np.abs(r - 2.0) > 0.15)
    assert 0.005 < ghosts < 0.03