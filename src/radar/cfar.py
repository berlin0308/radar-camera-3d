"""Stage 6: cell-averaging CFAR along range, then a local-maximum test in azimuth."""
import numpy as np


def ca_cfar_range(power, train=8, guard=2, pfa=1e-3):
    """Threshold each cell by the mean power of its range neighbours.

    power: (..., range), square-law (|x|^2). For exponential noise with N = 2*train training
    cells the multiplier that gives false-alarm probability pfa is N * (pfa^(-1/N) - 1).
    Returns (threshold, hit) with the same shape as power.
    """
    n = 2 * train
    alpha = n * (pfa ** (-1 / n) - 1)
    ker = np.zeros(2 * (train + guard) + 1); ker[:train] = 1; ker[-train:] = 1
    pad = np.pad(power, [(0, 0)] * (power.ndim - 1) + [(train + guard, train + guard)], mode="edge")
    noise = np.apply_along_axis(lambda v: np.convolve(v, ker, "valid"), -1, pad) / n
    thr = alpha * noise
    return thr, power > thr


def azimuth_peaks(power):
    """True where a cell is >= both azimuth neighbours (axis -2).

    Range-only CFAR keeps every cell of a target's 14-degree main lobe; this leaves one per lobe.
    """
    return (power >= np.roll(power, 1, axis=-2)) & (power >= np.roll(power, -1, axis=-2))
