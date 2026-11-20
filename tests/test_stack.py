import numpy as np

from src.fusion.stack import ground_plane, icp, _se3, take_rows, fills
from src.fusion.jbu import nearest_column


def _scene(rng):
    """A ground plane 1.5 m below the sensor, two walls and a box: enough structure to pin all six degrees of freedom."""
    g = np.c_[rng.uniform(-8, 8, (6000, 2)), np.full(6000, -1.5)]
    w1 = np.c_[rng.uniform(-8, 8, 2000), np.full(2000, 6.0), rng.uniform(-1.5, 2, 2000)]
    w2 = np.c_[np.full(2000, -5.0), rng.uniform(-6, 6, 2000), rng.uniform(-1.5, 2, 2000)]
    b = np.c_[rng.uniform(1, 2, 1500), rng.uniform(1, 2, 1500), rng.uniform(-1.5, -0.5, 1500)]
    return np.concatenate([g, w1, w2, b]) + rng.normal(0, 0.005, (11500, 3))


def test_ground_plane_height():
    n, c = ground_plane(_scene(np.random.default_rng(0)))
    assert abs(c - 1.5) < 0.02 and n[2] > 0.999


def test_icp_recovers_a_lift_and_a_turn():
    rng = np.random.default_rng(1); dst = _scene(rng)
    T = _se3(np.array([0.01, -0.02, 0.05, 0.05, -0.08, 0.25]))           # ~3 deg of yaw, 0.25 m up
    src = (dst - T[:3, 3]) @ T[:3, :3]                                   # dst = T src
    M, rms, inl = icp(src, dst)
    assert np.abs(M[:3, 3] - T[:3, 3]).max() < 0.02 and np.abs(M[:3, :3] - T[:3, :3]).max() < 0.005 and rms < 0.02


def test_one_take_reduces_to_one_row():
    rng = np.random.default_rng(2); H, W = 40, 60
    uv = np.c_[rng.uniform(0, W, 6), 20 + rng.uniform(-2, 2, 6)]; d = rng.uniform(2, 8, 6)
    F = fills(rng.uniform(0, 1, (H, W, 3)), uv, d, np.ones(6), np.zeros(6, int), 5, 15)
    assert np.allclose(F["nearest column"], nearest_column(uv, d, (H, W)))


def test_two_takes_split_by_row():
    W = 10; uv = np.array([[2.0, 5.0], [8.0, 5.0], [2.0, 30.0], [8.0, 30.0]]); d = np.array([1.0, 1.0, 5.0, 5.0])
    V, D, _ = take_rows(uv, d, np.ones(4), np.array([0, 0, 1, 1]), W)
    assert np.allclose(V, [[5.0] * W, [30.0] * W]) and np.allclose(D, [[1.0] * W, [5.0] * W])
    F = fills(np.zeros((40, W, 3)), uv, d, np.ones(4), np.array([0, 0, 1, 1]), 2, 5)
    assert F["nearest column"][0, 0] == 1.0 and F["nearest column"][39, 0] == 5.0
