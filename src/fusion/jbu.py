"""Joint bilateral upsampling of sparse depth seeds, guided by a camera image (Kopf et al. 2007).

Every pixel p takes a weighted mean of the seeds q:

    D(p) = sum_q w_s(p, q) w_c(p, q) d_q / sum_q w_s(p, q) w_c(p, q)

w_s falls off with the distance in the image (anisotropic: the seeds lie on one row, so the
vertical reach has to be long), w_c with the colour difference between p and the seed's own pixel
in the guide. The denominator is the confidence: how much support a pixel found.

Setting sigma_c to None drops w_c and gives the plain Gaussian fill; `nearest_column` uses no
image at all. Those two are the baselines the bilateral result is read against.
"""
import numpy as np


def seed_colours(guide, uv, half=2):
    """Median colour of the guide in a (2 half + 1)^2 patch around each seed."""
    H, W = guide.shape[:2]; out = np.empty((len(uv), guide.shape[2]))
    for i, (u, v) in enumerate(np.round(uv).astype(int)):
        patch = guide[max(v - half, 0):min(v + half + 1, H), max(u - half, 0):min(u + half + 1, W)]
        out[i] = np.median(patch.reshape(-1, guide.shape[2]), axis=0)
    return out


def upsample(guide, uv, depth, sigma_x, sigma_y, sigma_c=None, weight=None):
    """guide (H, W, C) float; uv (S, 2) seed pixels in guide coordinates; depth (S,). Returns depth (H, W), confidence (H, W)."""
    H, W = guide.shape[:2]; yy, xx = np.mgrid[:H, :W].astype(np.float32)
    weight = np.ones(len(uv)) if weight is None else weight
    cols = seed_colours(guide, uv) if sigma_c is not None else None
    num = np.zeros((H, W)); den = np.zeros((H, W))
    for i, ((u, v), d) in enumerate(zip(uv, depth)):
        w = weight[i] * np.exp(-0.5 * (((xx - u) / sigma_x) ** 2 + ((yy - v) / sigma_y) ** 2))
        if sigma_c is not None:
            w = w * np.exp(-0.5 * ((guide - cols[i]) ** 2).sum(axis=-1) / sigma_c ** 2)
        num += w * d; den += w
    return num / np.maximum(den, 1e-12), den


def nearest_column(uv, depth, shape):
    """No image: every pixel takes the seed nearest to its own column."""
    H, W = shape; j = np.abs(np.arange(W)[:, None] - uv[None, :, 0]).argmin(axis=1)
    return np.broadcast_to(depth[j][None, :], (H, W)).copy()
