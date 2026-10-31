"""Stages 2-3: range FFT, Doppler FFT, TDM-MIMO phase compensation."""
import numpy as np


def hann(n):
    """Hann window without the zero end points, scaled to unit mean so dB levels stay comparable."""
    w = np.hanning(n + 2).astype(np.float32)[1:-1]
    return w / w.mean()


def _hann(n, axis, ndim):
    shape = [1] * ndim; shape[axis] = n
    return hann(n).reshape(shape)


def range_fft(cube):
    """(chirp, tx, rx, sample) -> (chirp, tx, rx, range). Bin k is at k * range_resolution."""
    return np.fft.fft(cube * _hann(cube.shape[-1], -1, cube.ndim), axis=-1)


def doppler_fft(rc):
    """(chirp, tx, rx, range) -> (doppler, tx, rx, range), fft-shifted so index n_chirps//2 is v = 0."""
    return np.fft.fftshift(np.fft.fft(rc * _hann(rc.shape[0], 0, rc.ndim), axis=0), axes=0)


def tdm_compensate(rd):
    """Remove the Doppler phase that TX i picks up by firing i/n_tx of a loop later.

    Doppler bin k (shifted) means a phase advance of 2*pi*(k - D/2)/D per loop, so TX i has
    an extra 2*pi*(k - D/2)/D * i/n_tx on every element. Apply before building the virtual array.
    """
    D, n_tx = rd.shape[0], rd.shape[1]
    k = (np.arange(D) - D // 2)[:, None, None, None]
    i = np.arange(n_tx)[None, :, None, None]
    return rd * np.exp(-2j * np.pi * k / D * i / n_tx).astype(np.complex64)
