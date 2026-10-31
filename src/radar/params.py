"""Chirp parameters and the axes they imply (range, velocity, angle)."""
from dataclasses import dataclass
import json, os

import numpy as np
import yaml

C = 2.99792458e8


@dataclass(frozen=True)
class ChirpParams:
    n_samples: int          # ADC samples per chirp (complex)
    n_chirps: int           # chirp loops per frame
    n_tx: int
    n_rx: int
    sample_rate: float      # Hz
    slope: float            # Hz / s
    idle_time: float        # s
    ramp_end_time: float    # s
    adc_start_time: float   # s
    f_start: float          # Hz

    @classmethod
    def from_trace(cls, trace_dir):
        r = yaml.safe_load(open(os.path.join(trace_dir, "config.yaml")))["radar"]["args"]["radar"]
        return cls(n_samples=r["adc_samples"], n_chirps=r["frame_length"], n_tx=r["num_tx"], n_rx=4,
                   sample_rate=r["sample_rate"] * 1e3, slope=r["freq_slope"] * 1e12,
                   idle_time=r["idle_time"] * 1e-6, ramp_end_time=r["ramp_end_time"] * 1e-6,
                   adc_start_time=r["adc_start_time"] * 1e-6, f_start=r["frequency"] * 1e9)

    # --- derived
    @property
    def t_chirp(self):            # one TX slot
        return self.idle_time + self.ramp_end_time

    @property
    def t_loop(self):             # one loop over all TX = Doppler sampling period
        return self.n_tx * self.t_chirp

    @property
    def bandwidth(self):          # swept while sampling
        return self.slope * self.n_samples / self.sample_rate

    @property
    def f_centre(self):
        return self.f_start + self.slope * self.adc_start_time + self.bandwidth / 2

    @property
    def wavelength(self):
        return C / self.f_centre

    @property
    def range_resolution(self):
        return C / (2 * self.bandwidth)

    @property
    def max_range(self):
        return self.sample_rate * C / (2 * self.slope)

    @property
    def velocity_resolution(self):
        return self.wavelength / (2 * self.n_chirps * self.t_loop)

    @property
    def max_velocity(self):
        return self.wavelength / (4 * self.t_loop)

    @property
    def ranges(self):
        return np.arange(self.n_samples) * self.range_resolution

    @property
    def velocities(self):         # fft-shifted Doppler axis, index n_chirps // 2 is zero
        return (np.arange(self.n_chirps) - self.n_chirps // 2) * self.velocity_resolution

    def check(self, trace_dir, rtol=1e-3):
        """Resolutions must match what the recorder wrote in radar.json."""
        j = json.load(open(os.path.join(trace_dir, "radar", "radar.json")))
        assert abs(self.range_resolution / j["range_resolution"] - 1) < rtol, (self.range_resolution, j)
        assert abs(self.velocity_resolution / j["doppler_resolution"] - 1) < rtol, (self.velocity_resolution, j)
        return self
