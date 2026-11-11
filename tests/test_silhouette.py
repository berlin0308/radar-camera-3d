"""Outline scoring on a synthetic object: a box at 3 m in front of a wall at 7 m."""
import numpy as np

from src.eval.silhouette import silhouette, outline, classes, edge_accuracy


def _scene():
    H, W = 60, 80; sil = np.zeros((H, W), bool); sil[20:50, 30:50] = True
    lab = np.full((H, W, 3), 0.5); lab[sil] = 0.5; lab[20:50, 30:40] = 0.1        # left half of the box is dark: distinct; right half matches the wall
    ref = np.where(sil, 3.0, 7.0); ref[::3] = np.nan                               # sparse rows, like the LiDAR
    return sil, lab, ref


def test_silhouette_closes_sparse_rows():
    sil, _, ref = _scene(); obj = sil & np.isfinite(ref)
    assert (silhouette(obj, close=(5, 3)) == sil).mean() > 0.98


def test_outline_classes_follow_colour():
    sil, lab, ref = _scene(); ol = outline(sil, lab, ref); cl = classes(ol)
    ys, xs = np.nonzero(cl["distinct"]); assert len(xs) and xs.mean() < 40
    ys, xs = np.nonzero(cl["similar"]); assert len(xs) and xs.mean() > 40


def test_edge_accuracy_perfect_and_constant():
    sil, lab, ref = _scene(); ol = outline(sil, lab, ref); cl = classes(ol)
    perfect = edge_accuracy(np.where(sil, 3.0, 7.0), sil, ol, cl)
    flat = edge_accuracy(np.full(sil.shape, 7.0), sil, ol, cl)
    assert all(v["accuracy"] == 1.0 for v in perfect.values())
    assert all(0.3 < v["accuracy"] < 0.7 for v in flat.values())                 # a constant gets only the outside half right
