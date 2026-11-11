"""Edge-aware weighted least squares fill: depth spreads along the image, not across its edges.

    minimise  sum_seeds s_i (D_i - d_i)^2  +  lam * sum_{p~q} w_pq (D_p - D_q)^2,
    w_pq = exp(-|I_p - I_q|^2 / (2 sigma^2))   on the 4-neighbour grid of the guide I.

The joint bilateral fill asks whether a pixel *looks like* a seed; this one asks whether a pixel
is *connected* to a seed without crossing an edge. That matters when two objects share a colour:
a bronze statue and a dark window behind it look alike, but an edge separates them.

Confidence is the same system with a small mass term, solved for an all-ones seed value: it decays
with the edge-weighted distance to the nearest seed.
"""
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla


def _laplacian(guide, sigma, floor=1e-4):
    H, W = guide.shape[:2]; idx = np.arange(H * W).reshape(H, W); rows, cols, vals = [], [], []
    for a, b in [(idx[:, :-1], idx[:, 1:]), (idx[:-1, :], idx[1:, :])]:
        ga, gb = guide.reshape(H * W, -1)[a.ravel()], guide.reshape(H * W, -1)[b.ravel()]
        w = np.maximum(np.exp(-((ga - gb) ** 2).sum(axis=1) / (2 * sigma ** 2)), floor)   # never fully cut: a seedless region still takes its neighbour's depth
        rows += [a.ravel(), b.ravel()]; cols += [b.ravel(), a.ravel()]; vals += [-w, -w]
    A = sp.csr_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))), shape=(H * W, H * W))
    return A - sp.diags(np.asarray(A.sum(axis=1)).ravel())


def fill(guide, uv, depth, sigma=0.05, lam=1.0, radius=1, weight=None, mass=1e-6):
    """guide (H, W, C); uv (S, 2) seed pixels; depth (S,). Returns depth (H, W), confidence (H, W) in [0, 1]."""
    H, W = guide.shape[:2]; n = H * W
    weight = np.ones(len(uv)) if weight is None else np.asarray(weight, float)
    s = np.zeros(n); sd = np.zeros(n)
    for (u, v), d, w in zip(np.round(uv).astype(int), depth, weight):
        for dv in range(-radius, radius + 1):
            for du in range(-radius, radius + 1):
                y, x = v + dv, u + du
                if 0 <= y < H and 0 <= x < W:
                    k = y * W + x; s[k] += w; sd[k] += w * d
    L = _laplacian(guide, sigma); S = sp.diags(s)
    D = spla.spsolve((S + lam * L).tocsc(), sd)
    C = spla.spsolve((S + lam * L + mass * sp.identity(n)).tocsc(), s)
    C = C / max(C.max(), 1e-12)
    return D.reshape(H, W), C.reshape(H, W)
