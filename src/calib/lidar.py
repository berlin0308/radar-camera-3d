"""Read one Ouster scan from a roverd lidar folder into x, y, z (sensor frame)."""
import json, lzma, os

import numpy as np


class LidarTrace:
    def __init__(self, trace_dir):
        d = os.path.join(trace_dir, "lidar")
        self.dir = d
        self.ts = np.fromfile(os.path.join(d, "ts"), dtype="<f8")
        self.idx = np.fromfile(os.path.join(d, "rng_i"), dtype="<u8")
        meta = json.load(open(os.path.join(d, "meta.json")))
        self.shape = tuple(meta["rng"]["shape"])                     # [beams, 2048]
        bi = json.load(open(os.path.join(d, "lidar.json")))["beam_intrinsics"]
        self.alt = np.deg2rad(bi["beam_altitude_angles"])[:, None]
        self.az = np.deg2rad(bi["beam_azimuth_angles"])[:, None]
        self.enc = 2 * np.pi * (1 - np.arange(self.shape[1]) / self.shape[1])[None, :]

    def __len__(self):
        return len(self.ts)

    def range_image(self, k):
        """Range in metres, 0 where there is no return."""
        with open(os.path.join(self.dir, "rng"), "rb") as f:
            f.seek(self.idx[k]); buf = f.read(self.idx[k + 1] - self.idx[k])
        return np.frombuffer(lzma.decompress(buf), "<u2").reshape(self.shape).astype(np.float32) / 1000

    def xyz(self, k, r_min=0.3):
        """(N, 3) points of scan k in the lidar frame (x, y in the spin plane, z up)."""
        r = self.range_image(k)
        ok = r > r_min
        th = self.enc + self.az
        x = r * np.cos(th) * np.cos(self.alt); y = r * np.sin(th) * np.cos(self.alt); z = r * np.sin(self.alt)
        return np.stack([x[ok], y[ok], z[ok]], axis=-1)

    def index_at(self, t):
        k = int(np.searchsorted(self.ts, t))
        if k > 0 and (k == len(self.ts) or t - self.ts[k - 1] < self.ts[k] - t):
            k -= 1
        return k
