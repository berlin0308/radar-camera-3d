"""The LiDAR as reference depth in the camera image, and the error of a depth map against it.

Depth here is the range from the camera centre (not z), since the lens is a fisheye. The LiDAR is
used for nothing but this: it never enters the method being scored.
"""
import json, os

import numpy as np

from ..calib.lidar import LidarTrace
from ..calib.board_image import project_fisheye
from ..calib.transform import joint_path
from ..recon.accumulate import height_above_ground

BINS = [(0, 5), (5, 10), (10, 20)]


def lidar_depth_image(trace, t_unix, scale=0.25, scans=3, max_angle=np.radians(80), return_xy=False):
    """Reference depth (H, W) and height above ground (H, W) at `scale`; NaN where no LiDAR point falls.

    With return_xy, also the LiDAR-frame x, y of the point behind each pixel (H, W, 2), to cut out an object.

    A few consecutive scans are merged (the rig is static) and the nearest point wins per pixel.
    """
    lt = LidarTrace(trace); k = lt.index_at(t_unix)
    path = joint_path(trace); j = json.load(open(path)); lc = j["lidar_to_camera"]; R, t = np.array(lc["R"]), np.array(lc["t_m"])
    cam = json.load(open(os.path.join(os.path.dirname(path), os.path.basename(j["radar_to_camera"]["intrinsics"]))))
    xyz, hgt = [], []
    for kk in range(max(k - scans // 2, 0), min(k + scans // 2 + 1, len(lt))):
        p = lt.xyz(kk); xyz.append(p); hgt.append(height_above_ground(p))
    xyz = np.concatenate(xyz); hgt = np.concatenate(hgt)
    xc = xyz @ R.T + t; d = np.linalg.norm(xc, axis=1)
    ok = (xc[:, 2] > 0.1) & (np.arccos(np.clip(xc[:, 2] / d, -1, 1)) < max_angle)
    uv = np.floor(project_fisheye(xc[ok], cam) * scale).astype(int); d, hgt, xy = d[ok], hgt[ok], xyz[ok, :2]
    H, W = int(cam["h"] * scale), int(cam["w"] * scale)
    inb = (uv[:, 0] >= 0) & (uv[:, 0] < W) & (uv[:, 1] >= 0) & (uv[:, 1] < H); uv, d, hgt, xy = uv[inb], d[inb], hgt[inb], xy[inb]
    order = np.argsort(-d)                                               # far first, so the nearest is written last
    depth = np.full((H, W), np.nan); height = np.full((H, W), np.nan); xyimg = np.full((H, W, 2), np.nan)
    depth[uv[order, 1], uv[order, 0]] = d[order]; height[uv[order, 1], uv[order, 0]] = hgt[order]
    xyimg[uv[order, 1], uv[order, 0]] = xy[order]
    return (depth, height, xyimg) if return_xy else (depth, height)


def edge_band(ref, win=7, jump=1.0, grow=3):
    """Pixels near a depth edge of the reference: within `grow` px of a window whose LiDAR depths span more than `jump` m.

    This is where an edge-aware fill should differ from a plain one; away from edges both are the
    same smooth interpolation.
    """
    from scipy.ndimage import maximum_filter, minimum_filter, binary_dilation
    hi = maximum_filter(np.where(np.isfinite(ref), ref, -np.inf), size=win)
    lo = minimum_filter(np.where(np.isfinite(ref), ref, np.inf), size=win)
    band = np.isfinite(hi) & np.isfinite(lo) & (hi - lo > jump)
    return binary_dilation(band, iterations=grow)


def errors(pred, ref, mask=None, bins=BINS):
    """Mean and median absolute error per distance bin of the reference, over pixels where both exist."""
    ok = np.isfinite(ref) & np.isfinite(pred)
    if mask is not None:
        ok &= mask
    out = {}
    for lo, hi in bins + [(bins[0][0], bins[-1][1])]:
        m = ok & (ref >= lo) & (ref < hi); e = np.abs(pred[m] - ref[m])
        out[f"{lo}-{hi} m"] = dict(n=int(m.sum()), mae=float(e.mean()) if m.any() else float("nan"),
                                   median=float(np.median(e)) if m.any() else float("nan"))
    return out
