"""Stage 7: one frame -> list of detections (r, azimuth, elevation, velocity, snr) and xyz."""
import numpy as np

from .range_doppler import range_fft, doppler_fft, tdm_compensate
from .angle import virtual_array, azimuth_fft, elevation, AZIMUTHS
from .cfar import ca_cfar_range, azimuth_peaks

POINT_DTYPE = np.dtype([("r", "f4"), ("az", "f4"), ("el", "f4"), ("v", "f4"), ("snr", "f4")])


def spectra(cube):
    """Complex cube -> (rd, spec_az, spec_el): range-Doppler and the two angle spectra."""
    rd = tdm_compensate(doppler_fft(range_fft(cube)))
    va = virtual_array(rd)
    return rd, azimuth_fft(va[:, 1]), azimuth_fft(va[:, 0])


RANGE_OFFSET = 0.183     # m; the radar reads long by this much (M1 fit against a tape measure, 10/21)


def detect(cube, params, train=8, guard=2, pfa=1e-3, r_min=0.35, r_max=20.5, range_offset=RANGE_OFFSET):
    """Detections of one frame.

    Power is summed over Doppler before CFAR (the rig moves, so a static scene is spread over
    velocity). Bins below r_min (TX->RX leakage) and above r_max (I/Q mirror of near echoes) are
    masked. Velocity and elevation are read at the strongest Doppler bin of each detected cell.
    The constant range bias measured in M1 is subtracted from r (range_offset=0 to get raw bins).
    Returns (points, info) where info holds the intermediate maps for plotting.
    """
    rd, spec_az, spec_el = spectra(cube)
    power = (np.abs(spec_az) ** 2).sum(axis=0)                       # (azimuth, range)
    thr, hit = ca_cfar_range(power, train, guard, pfa)
    ranges = params.ranges
    hit &= (ranges >= r_min) & (ranges <= r_max)
    hit &= azimuth_peaks(power)

    a_idx, r_idx = np.nonzero(hit)
    kd = np.argmax(np.abs(spec_az[:, a_idx, r_idx]), axis=0)
    pts = np.empty(len(a_idx), dtype=POINT_DTYPE)
    pts["r"] = ranges[r_idx] - range_offset
    pts["az"] = AZIMUTHS[a_idx]
    pts["el"] = np.arcsin(elevation(spec_az[kd, a_idx, r_idx], spec_el[kd, a_idx, r_idx]))
    pts["v"] = params.velocities[kd]
    pts["snr"] = 10 * np.log10(power[a_idx, r_idx] / (thr[a_idx, r_idx] / _alpha(train, pfa)))
    return pts, dict(rd=rd, spec_az=spec_az, power=power, thr=thr)


def _alpha(train, pfa):
    n = 2 * train
    return n * (pfa ** (-1 / n) - 1)


def to_xyz(pts):
    """Spherical -> Cartesian: x right, y forward, z up (metres)."""
    r, az, el = pts["r"], pts["az"], pts["el"]
    return np.stack([-r * np.sin(az) * np.cos(el), r * np.cos(az) * np.cos(el), r * np.sin(el)], axis=-1)
