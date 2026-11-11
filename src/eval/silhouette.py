"""How well a dense depth map keeps an object's outline, split by how well the outline shows in colour.

The LiDAR gives the object's pixels (sparse rows); closing them gives its silhouette. Each pixel of
the silhouette's outline gets two numbers from what lies just inside and just outside it:

    contrast  = |mean Lab inside - mean Lab outside|   (Lab units, 0-100)
    step      = mean LiDAR range outside - inside      (m)

Only outline pixels with a real depth step count (the base of a statue meets the ground at the same
range, so there is no edge there to keep). An outline pixel is "distinct" when its contrast is high
and "similar" (to the background) when it is low.

A fill is scored near the outline by edge accuracy: of the pixels within `band` px of the outline,
the share that the fill puts on the right side, i.e. nearer the object's range when inside the
silhouette and nearer the background's range when outside. Silhouette IoU does the same over a box
around the whole object.
"""
import numpy as np
from scipy import ndimage as ndi


def silhouette(obj, close=(9, 5)):
    """Solid mask of the object from its scattered LiDAR pixels: closed, holes filled, largest piece."""
    m = ndi.binary_fill_holes(ndi.binary_closing(obj, structure=np.ones(close)))
    lbl, n = ndi.label(m)
    return lbl == (np.bincount(lbl.ravel())[1:].argmax() + 1) if n else m


def _side_mean(x, valid, side, r):
    k = np.ones((2 * r + 1, 2 * r + 1)); w = (valid & side).astype(float)
    return ndi.convolve(np.where(w > 0, x, 0.0), k) / np.maximum(ndi.convolve(w, k), 1e-9), ndi.convolve(w, k) > 0


def outline(sil, lab, ref, r_colour=4, r_depth=8, min_step=1.0):
    """Outline pixels with their contrast (Lab units), depth step (m) and the ranges just inside / outside."""
    edge = sil & ~ndi.binary_erosion(sil)
    ones = np.ones(sil.shape, bool)
    cin = np.stack([_side_mean(lab[..., c], ones, sil, r_colour)[0] for c in range(3)], -1)
    cout = np.stack([_side_mean(lab[..., c], ones, ~sil, r_colour)[0] for c in range(3)], -1)
    fin = np.isfinite(ref); din, okin = _side_mean(ref, fin, sil, r_depth); dout, okout = _side_mean(ref, fin, ~sil, r_depth)
    contrast = np.linalg.norm(cin - cout, axis=-1) * 100
    step = np.where(okin & okout, dout - din, np.nan)
    keep = edge & (step > min_step)
    return dict(edge=keep, contrast=contrast, step=step, d_in=din, d_out=dout)


def classes(ol, distinct=15.0, similar=7.0):
    """Outline pixels split into distinct (contrast >= distinct) and similar (contrast < similar)."""
    e, c = ol["edge"], ol["contrast"]
    return {"distinct": e & (c >= distinct), "similar": e & (c < similar)}


def edge_accuracy(D, sil, ol, cls, band=4):
    """Per outline class: share of pixels within `band` px of it that the fill puts on the right side."""
    out = {}
    for name, e in cls.items():
        if not e.any():
            out[name] = dict(n_outline=0, n=0, accuracy=float("nan")); continue
        dist, (iy, ix) = ndi.distance_transform_edt(~e, return_indices=True)
        near = dist <= band
        din, dout = ol["d_in"][iy, ix], ol["d_out"][iy, ix]               # ranges of the nearest outline pixel
        says_in = np.abs(D - din) < np.abs(D - dout)
        right = np.where(sil, says_in, ~says_in)
        # only pixels whose nearest outline pixel (of any class) is this one, so classes do not overlap
        alle = ol["edge"]; _, (jy, jx) = ndi.distance_transform_edt(~alle, return_indices=True)
        mine = e[jy, jx] & near
        out[name] = dict(n_outline=int(e.sum()), n=int(mine.sum()), accuracy=float(right[mine].mean()) if mine.any() else float("nan"))
    return out


def iou(D, sil, ol, margin=20):
    """Silhouette IoU in a box around the object: a pixel is 'object' if the fill is nearer the object's range than the background's."""
    sl = ndi.find_objects(sil.astype(int))[0]; H, W = sil.shape
    y0, y1 = max(sl[0].start - margin, 0), min(sl[0].stop + margin, H); x0, x1 = max(sl[1].start - margin, 0), min(sl[1].stop + margin, W)
    d_obj = np.nanmedian(ol["d_in"][ol["edge"]]); d_bg = np.nanmedian(ol["d_out"][ol["edge"]])
    pred = np.abs(D - d_obj) < np.abs(D - d_bg)
    p, t = pred[y0:y1, x0:x1], sil[y0:y1, x0:x1]
    return float((p & t).sum() / max((p | t).sum(), 1)), (y0, y1, x0, x1), float(d_obj), float(d_bg)
