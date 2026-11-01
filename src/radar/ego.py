"""Ego-velocity from Doppler, and the static-world consistency filter.

While the rig moves with speed s and heading phi (in the radar's xy plane, + az = left), every
static target at azimuth az has radial velocity v_r = sign * s * cos(az - phi), where sign is -1 because
our Doppler axis (params.velocities, fft-shifted bin index minus n_chirps/2) counts approaching targets as
negative: walking straight at the Jeep at 1 m/s (15-49-40, 735-780 s) puts the static-world ridge at
-1.0 m/s in front. Points off that curve are multipath ghosts, angle sidelobes, or things that move.

The estimate uses the whole range-Doppler-azimuth spectrum, not just the CFAR points: for each
candidate (s, phi) the power along the curve is summed over azimuth and range, and the maximum
wins. With 10-20 points per frame a point-only fit is under-determined at walking speeds.
"""
import numpy as np

SIGN = -1.0          # v < 0 = approaching (checked on the straight walk to the Jeep)


def ego_curve(az, speed, heading):
    return SIGN * speed * np.cos(az - heading)


def estimate_ego_dense(spec_az, velocities, azimuths, speeds=None, headings_deg=None, r_lo=17, r_hi=235, tol_bins=1.5):
    """(speed, heading_rad, support) from the (doppler, az, range) complex spectrum of one frame.

    Each azimuth bin votes once with the Doppler of its strongest return beyond r_lo (1.5 m, past
    the leakage and the rig's own near clutter); the (speed, heading) curve that passes within
    tol_bins of the most votes wins, votes weighted by that return's power. support = weighted
    fraction of azimuth bins on the winning curve (0..1).
    """
    P = (np.abs(spec_az[..., r_lo:r_hi]) ** 2).sum(axis=-1)               # (doppler, az)
    D = len(velocities); v_res = velocities[1] - velocities[0]
    k_peak = P.argmax(axis=0); w = P.max(axis=0); w = w / (w.sum() + 1e-12)   # one vote per azimuth
    v_peak = velocities[k_peak]
    if speeds is None: speeds = np.arange(0.0, 1.80, v_res / 2)
    if headings_deg is None: headings_deg = np.arange(-180, 180, 2.0)       # full circle: orbiting an object means moving sideways
    S = speeds[:, None, None]; H = np.radians(headings_deg)[None, :, None]
    resid = np.abs(v_peak[None, None, :] - ego_curve(azimuths[None, None, :], S, H))
    support = ((resid < tol_bins * v_res) * w[None, None, :]).sum(axis=-1)
    i, j = np.unravel_index(np.argmax(support), support.shape)
    return float(speeds[i]), float(np.radians(headings_deg[j])), float(support[i, j])


def static_mask(pts, speed, heading, v_res=0.0559, tol_bins=1.5):
    """True for detections consistent with a static world under the given ego-motion."""
    return np.abs(pts["v"] - ego_curve(pts["az"], speed, heading)) < tol_bins * v_res


def static_mask_from_velocity(pts, v_xy, v_res=0.0559, tol_bins=1.5):
    """Same test with the sensor velocity known (e.g. from LiDAR odometry): v_xy = (vx, vy) in the
    radar frame, x right, y forward. A static target at azimuth az (+ left) lies along
    u = (-sin az, cos az) and is approached at v_xy . u."""
    u = np.stack([-np.sin(pts["az"]), np.cos(pts["az"])], axis=-1)
    v_static = SIGN * (u @ np.asarray(v_xy))
    return np.abs(pts["v"] - v_static) < tol_bins * v_res


def filter_static(pts, info, params, min_support=0.4, tol_bins=1.5):
    """Keep the detections of one frame that belong to the static world.

    pts, info: output of pointcloud.detect (info['spec_az'] is the angle spectrum). Returns
    (static points, (speed, heading, support)). If fewer than min_support of the azimuth votes sit
    on any curve (no usable Doppler structure), keeps points with |v| < tol_bins bins instead.
    """
    s, phi, sup = estimate_ego_dense(info["spec_az"], params.velocities, _azimuths(), tol_bins=tol_bins)
    v_res = params.velocity_resolution
    if sup < min_support:
        return pts[np.abs(pts["v"]) < tol_bins * v_res], (0.0, 0.0, sup)
    return pts[static_mask(pts, s, phi, v_res=v_res, tol_bins=tol_bins)], (s, phi, sup)


def _azimuths():
    from .angle import AZIMUTHS
    return AZIMUTHS
