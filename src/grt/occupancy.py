"""Geometry of the GRT occupancy grid: cell (el, az, rng) -> xyz in the LiDAR sensor frame.

The grid is the Ouster range image after destagger, middle half of the columns (front 180 deg),
decimated (2 beams, 8 columns, 4 radar range bins). So cell (i, j, k) covers beams 2i..2i+1,
columns 512+8j..519+8j of the 2048-column image, and ranges (k, k+1) x 4 x range_resolution.
Angles come from lidar.json, the same tables roverd uses, not the linspace approximation.
"""
import json, os

import numpy as np

CROP_AZ, D_EL, D_AZ, D_RNG = 0.25, 2, 8, 4


class GridGeometry:
    def __init__(self, trace_dir):
        bi = json.load(open(os.path.join(trace_dir, "lidar", "lidar.json")))["beam_intrinsics"]
        self.alt = np.deg2rad(bi["beam_altitude_angles"])            # (beams,)
        self.az_off = np.deg2rad(bi["beam_azimuth_angles"])
        rj = json.load(open(os.path.join(trace_dir, "radar", "radar.json")))
        self.d_rng = rj["range_resolution"] * D_RNG
        n_beams, n_cols = len(self.alt), 2048
        c0 = int(CROP_AZ * n_cols)
        self.n_el, self.n_az, self.n_rng = n_beams // D_EL, (n_cols - 2 * c0) // D_AZ, rj["shape"][-1] // D_RNG
        # destaggered image: column c is encoder angle 2*pi*(1 - c/n_cols); beam azimuth offsets are removed by destagger
        cols = c0 + np.arange(self.n_az) * D_AZ + (D_AZ - 1) / 2
        self.az = 2 * np.pi * (1 - cols / n_cols)                     # (n_az,) rad, lidar frame, CCW from +x
        self.el = self.alt.reshape(self.n_el, D_EL).mean(axis=1)      # (n_el,) rad
        self.rng = (np.arange(self.n_rng) + 0.5) * self.d_rng          # (n_rng,) m, cell centres

    def xyz(self, cells):
        """(N, 3) int cell indices (el, az, rng) -> (N, 3) xyz metres in the lidar frame."""
        i, j, k = cells.T
        r, az, el = self.rng[k], self.az[j], self.el[i]
        return np.stack([r * np.cos(el) * np.cos(az), r * np.cos(el) * np.sin(az), r * np.sin(el)], axis=-1)

    def points(self, grid, threshold=0.5, logits=True):
        """Occupied cells of one (el, az, rng) grid -> (xyz, probability)."""
        p = 1 / (1 + np.exp(-grid.astype(np.float32))) if logits else grid.astype(np.float32)
        cells = np.argwhere(p > threshold)
        return self.xyz(cells), p[tuple(cells.T)]

    def bev(self, grid, logits=True, mode="max"):
        """(el, az, rng) -> (az, rng) top view by max over elevation (probabilities)."""
        p = 1 / (1 + np.exp(-grid.astype(np.float32))) if logits else grid.astype(np.float32)
        return p.max(axis=0) if mode == "max" else p.mean(axis=0)
