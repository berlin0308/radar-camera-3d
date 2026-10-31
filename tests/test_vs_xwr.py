"""Each stage of src/radar against xwr.rsp on the same frame. xwr is the oracle only; src/ never imports it.

    .venv/bin/python -m pytest tests/ -q
Set RADAR_TRACE to another trace folder to test on it.
"""
import os

import numpy as np
import pytest

from src.radar import RadarTrace, iiqq_to_complex, range_fft, doppler_fft, tdm_compensate, virtual_array, azimuth_fft

xwr_rsp = pytest.importorskip("xwr.rsp")
from xwr.rsp.numpy import AWR1843Boost

TRACE = os.environ.get("RADAR_TRACE", "data/raw/2026-10-29.15-18-16")
FRAME = int(os.environ.get("RADAR_FRAME", "300"))
RTOL = 1e-5


@pytest.fixture(scope="module")
def frame():
    tr = RadarTrace(TRACE)
    return np.array(tr.iq[FRAME])            # int16 (chirp, tx, rx, 512)


def rel_err(a, b):
    return np.abs(a - b).max() / np.abs(b).max()


def test_params_match_radar_json():
    RadarTrace(TRACE)                          # ChirpParams.check asserts range/doppler resolution


def test_adc_unpacking(frame):
    ours = iiqq_to_complex(frame)
    ref = xwr_rsp.iq_from_iiqq(frame, sample_swap=False)
    assert ours.shape == ref.shape == (64, 3, 4, 256)
    assert np.array_equal(ours, ref)


def test_range_doppler(frame):
    cube = iiqq_to_complex(frame)
    ours = doppler_fft(range_fft(cube))
    ref = AWR1843Boost(window=True).doppler_range(cube[None])[0]
    assert ours.shape == ref.shape
    assert rel_err(ours, ref) < RTOL


def test_tdm_compensation(frame):
    rd = doppler_fft(range_fft(iiqq_to_complex(frame)))
    ours = tdm_compensate(rd)
    ref = AWR1843Boost(window=True).doppler_correction(rd[None])[0]
    assert rel_err(ours, ref) < RTOL


def test_virtual_array(frame):
    rd = tdm_compensate(doppler_fft(range_fft(iiqq_to_complex(frame))))
    ours = virtual_array(rd)
    ref = AWR1843Boost(window=True).mimo_virtual_array(rd[None])[0]
    assert np.array_equal(ours, ref)


def test_azimuth_spectrum(frame):
    """xwr does an un-padded 8-point azimuth FFT with + on the right; ours is 64-point, flipped."""
    rd = tdm_compensate(doppler_fft(range_fft(iiqq_to_complex(frame))))
    va = virtual_array(rd)
    ref = AWR1843Boost(window={"azimuth": True, "elevation": False}).elevation_azimuth(rd[None])[0]
    ref_az = ref[:, 1]                     # shifted 2-point elevation FFT: index 1 = row0 + row1 (sum of both rows)
    ours = azimuth_fft(va[:, 1] + va[:, 0])                   # same row sum, 64 bins, flipped
    ours8 = ours[:, ::-1][:, ::8]                              # every 8th bin of the un-flipped 64 = the 8-point FFT
    assert rel_err(ours8, ref_az) < RTOL
