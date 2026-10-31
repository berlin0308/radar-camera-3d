"""Stages 4-5: MIMO virtual array and angle FFT (AWR1843Boost)."""
import numpy as np

from .range_doppler import hann

N_AZ = 64                                   # zero-padded azimuth FFT size
SIN_AZ = (np.arange(N_AZ) - N_AZ // 2) / (N_AZ / 2)     # d = lambda/2 -> sin(theta) per bin
AZIMUTHS = np.arcsin(SIN_AZ)                # rad, + = left (after the flip in azimuth_fft)


def virtual_array(rd):
    """(doppler, tx, rx, range) -> (doppler, 2 rows, 8 elements, range).

    Row 1 is the azimuth row: TX1·RX1-4 then TX3·RX1-4, half-wavelength spacing, no gap
    (TX1 and TX3 are 2 wavelengths apart). Row 0 is TX2·RX1-4, raised by half a wavelength,
    sitting over azimuth positions 2..5.
    """
    D, _, _, R = rd.shape
    va = np.zeros((D, 2, 8, R), dtype=np.complex64)
    va[:, 1, 0:4] = rd[:, 0]
    va[:, 1, 4:8] = rd[:, 2]
    va[:, 0, 2:6] = rd[:, 1]
    return va


def azimuth_fft(row):
    """(..., 8 elements, range) -> (..., N_AZ, range) spectrum; bin m is at AZIMUTHS[m], + = left.

    The raw FFT puts +sin(theta) on the right (confirmed on 10/21), so the axis is flipped here.
    """
    w = hann(row.shape[-2])[:, None]
    spec = np.fft.fftshift(np.fft.fft(row * w, n=N_AZ, axis=-2), axes=-2)
    return spec[..., ::-1, :]


def elevation(spec_az, spec_el):
    """sin(elevation) from the phase difference between the raised row and the azimuth row.

    Both inputs are azimuth spectra at the same (doppler, azimuth bin, range); the raised row
    is half a wavelength up, so the phase difference is pi * sin(el).
    """
    return np.clip(np.angle(spec_el * np.conj(spec_az)) / np.pi, -1, 1)
