"""Joint bilateral upsampling on a synthetic guide: depth must stop at a colour edge."""
import numpy as np

from src.fusion.jbu import upsample, nearest_column


def _scene():
    guide = np.zeros((40, 80, 3)); guide[:, 40:] = 1.0                   # dark left half, bright right half
    uv = np.array([[30.0, 20.0], [50.0, 20.0]]); depth = np.array([3.5, 6.0])
    return guide, uv, depth


def test_colour_term_keeps_depth_on_its_side():
    guide, uv, depth = _scene()
    D, conf = upsample(guide, uv, depth, sigma_x=20, sigma_y=40, sigma_c=0.1)
    assert abs(D[20, 39] - 3.5) < 1e-3 and abs(D[20, 40] - 6.0) < 1e-3   # one pixel either side of the edge
    assert abs(D[2, 39] - 3.5) < 1e-3                                    # and far from the seed row
    assert conf.min() > 0


def test_without_colour_the_edge_is_blurred():
    guide, uv, depth = _scene()
    D, _ = upsample(guide, uv, depth, sigma_x=20, sigma_y=40, sigma_c=None)
    assert 4.5 < D[20, 39] < 5.0 and 4.5 < D[20, 40] < 5.0               # both near the mean of 3.5 and 6.0


def test_worked_example_from_the_handout():
    # weights 0.90 and 0.05 on seeds at 3.5 m and 6.0 m give 3.63 m; equal weights give 4.75 m
    assert abs((0.90 * 3.5 + 0.05 * 6.0) / 0.95 - 3.63) < 0.005
    assert abs((3.5 + 6.0) / 2 - 4.75) < 1e-9


def test_nearest_column_uses_no_image():
    _, uv, depth = _scene()
    D = nearest_column(uv, depth, (40, 80))
    assert (D[:, :40] == 3.5).all() and (D[:, 41:] == 6.0).all()


def test_least_squares_respects_connectivity_where_bilateral_matches_colour():
    # dark | bright | dark: the right dark block has no seed and touches only the bright block.
    # The bilateral fill gives it the left block's depth (same colour); the least-squares fill the bright block's (connected).
    from src.fusion.wls import fill
    guide = np.zeros((30, 90, 3)); guide[:, 30:60] = 1.0
    uv = np.array([[15.0, 15.0], [45.0, 15.0]]); depth = np.array([3.5, 6.0])
    Db, _ = upsample(guide, uv, depth, sigma_x=40, sigma_y=40, sigma_c=0.1)
    Dw, _ = fill(guide, uv, depth, sigma=0.1, lam=1.0)
    assert abs(Db[15, 80] - 3.5) < 0.1
    assert abs(Dw[15, 80] - 6.0) < 0.1 and abs(Dw[15, 5] - 3.5) < 0.1 and abs(Dw[15, 45] - 6.0) < 0.1
