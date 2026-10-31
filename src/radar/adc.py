"""Stage 1: raw ADC file -> complex cube [chirp, tx, rx, sample]."""
import json, os

import numpy as np

from .params import ChirpParams


def iiqq_to_complex(iiqq):
    """(..., 2n) int16 as written by the DCA1000 -> (..., n) complex64.

    Two LVDS lanes are interleaved as Q0 Q1 I0 I1 Q2 Q3 I2 I3 ...  Getting the order
    wrong mirrors the range axis (near echoes show up at max_range - d).
    """
    x = np.asarray(iiqq, dtype=np.float32)
    out = np.empty(x.shape[:-1] + (x.shape[-1] // 2,), dtype=np.complex64)
    out[..., 0::2] = x[..., 2::4] + 1j * x[..., 0::4]
    out[..., 1::2] = x[..., 3::4] + 1j * x[..., 1::4]
    return out


class RadarTrace:
    """One roverd radar recording, memory-mapped. iq[k] is frame k as stored (int16)."""

    def __init__(self, trace_dir):
        self.dir = trace_dir
        r = os.path.join(trace_dir, "radar")
        meta = json.load(open(os.path.join(r, "meta.json")))
        self.shape = tuple(meta["iq"]["shape"])                       # [chirp, tx, rx, 2*samples]
        self.iq = np.memmap(os.path.join(r, "iq"), dtype="<i2", mode="r").reshape(-1, *self.shape)
        self.ts = np.fromfile(os.path.join(r, "ts"), dtype="<f8")
        self.valid = np.fromfile(os.path.join(r, "valid"), dtype="u1").astype(bool)
        self.params = ChirpParams.from_trace(trace_dir).check(trace_dir)
        assert self.shape == (self.params.n_chirps, self.params.n_tx, self.params.n_rx, 2 * self.params.n_samples)

    def __len__(self):
        return len(self.ts)

    def frame(self, k):
        """Complex cube of frame k: (chirp, tx, rx, sample)."""
        return iiqq_to_complex(self.iq[k])

    def index_at(self, t):
        """Frame index whose timestamp is closest to Unix time t (sensors share one clock)."""
        k = int(np.searchsorted(self.ts, t))
        if k > 0 and (k == len(self.ts) or t - self.ts[k - 1] < self.ts[k] - t):
            k -= 1
        return k
