"""Radar depth seeds for a static rig: the range-azimuth cells that keep detecting, placed in the image.

One frame gives about a dozen CFAR points, and some of them are noise. With the rig still, a real
surface is detected in the same cell frame after frame, so the share of frames in which a cell
fires separates surfaces from false alarms without any pose.

A seed has a range and an azimuth but no height (the radar's elevation is unusable), so it is
placed at the radar's own height, z = 0 in the radar frame. In the image the seeds therefore lie
on one row.
"""
import numpy as np

from ..radar import RadarTrace, detect
from ..radar.angle import N_AZ, SIN_AZ
from ..calib.transform import RadarToCamera

SEED_DTYPE = [("r", "f4"), ("az", "f4"), ("snr", "f4"), ("share", "f4")]


def static_cells(trace, t0, seconds, stride=2, snr_min=10.0, min_share=0.3):
    """Cells detected in at least min_share of the frames from t0 (Unix) on. Returns a SEED_DTYPE array."""
    tr = RadarTrace(trace); k0 = tr.index_at(t0); k1 = min(tr.index_at(t0 + seconds), len(tr.ts) - 1)
    dr = float(tr.params.ranges[1] - tr.params.ranges[0]); nr = len(tr.params.ranges)
    hits = np.zeros((N_AZ, nr)); snr = np.zeros((N_AZ, nr)); rsum = np.zeros((N_AZ, nr)); n = 0
    for k in range(k0, k1, stride):
        if not tr.valid[k]:
            continue
        p, _ = detect(tr.frame(k), tr.params); p = p[p["snr"] > snr_min]; n += 1
        ia = np.clip(np.round((np.sin(p["az"]) + 1) * (N_AZ / 2)).astype(int), 0, N_AZ - 1)
        ir = np.clip(np.round(p["r"] / dr).astype(int), 0, nr - 1)
        np.add.at(hits, (ia, ir), 1); np.add.at(snr, (ia, ir), p["snr"]); np.add.at(rsum, (ia, ir), p["r"])
    ia, ir = np.nonzero(hits >= min_share * n)
    out = np.empty(len(ia), SEED_DTYPE)
    out["r"] = rsum[ia, ir] / hits[ia, ir]; out["az"] = np.arcsin(SIN_AZ[ia])
    out["snr"] = snr[ia, ir] / hits[ia, ir]; out["share"] = hits[ia, ir] / n
    return out, n


def drop_sidelobes(seeds, dr=0.0874, margin_db=12.0):
    """Drop a seed when a much stronger one sits in the same range bin at another azimuth.

    Eight elements leave azimuth sidelobes: a strong target (a reflector, a car's flank) shows up
    again at its own range tens of degrees away. Persistence cannot remove these, they are there in
    every frame. Returns (kept, dropped).
    """
    keep = np.ones(len(seeds), bool)
    for i, s in enumerate(seeds):
        same = (np.abs(seeds["r"] - s["r"]) < 0.5 * dr) & (seeds["az"] != s["az"])
        keep[i] = not (same.any() and seeds["snr"][same].max() > s["snr"] + margin_db)
    return seeds[keep], seeds[~keep]


def first_returns(seeds, az_tol=np.radians(3.0)):
    """Keep, per azimuth, only the nearest seed.

    The radar sees past a car to the wall behind it; the camera does not. A far seed drawn at the
    radar's height lands on the near object in the image and would sample the wrong colour.
    """
    keep = np.ones(len(seeds), bool)
    for i, s in enumerate(seeds):
        same = np.abs(seeds["az"] - s["az"]) < az_tol
        keep[i] = s["r"] <= seeds["r"][same].min() + 1e-6
    return seeds[keep]


def to_image(seeds, trace):
    """Seeds -> pixels (N, 2) in the 1920x1080 frame and range from the camera (N,), plus the kept mask."""
    C = RadarToCamera.load_joint(trace=trace)
    xyz = np.stack([-seeds["r"] * np.sin(seeds["az"]), seeds["r"] * np.cos(seeds["az"]), np.zeros(len(seeds))], axis=-1)
    xc = C(xyz); uv = C.pixels(xyz)
    ok = (xc[:, 2] > 0.1) & (uv[:, 0] >= 0) & (uv[:, 0] < C.cam["w"]) & (uv[:, 1] >= 0) & (uv[:, 1] < C.cam["h"])
    return uv, np.linalg.norm(xc, axis=1), ok
