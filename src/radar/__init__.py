"""Classical FMCW radar chain: raw ADC -> range -> Doppler -> angle -> CFAR -> 3D points.

Own implementation (numpy only). `xwr.rsp` is used in tests/ as the numerical oracle, never here.
"""
from .params import ChirpParams
from .adc import RadarTrace, iiqq_to_complex
from .range_doppler import range_fft, doppler_fft, tdm_compensate
from .angle import virtual_array, azimuth_fft, elevation, N_AZ, AZIMUTHS
from .cfar import ca_cfar_range, azimuth_peaks
from .pointcloud import detect, to_xyz, spectra

__all__ = ["ChirpParams", "RadarTrace", "iiqq_to_complex", "range_fft", "doppler_fft",
           "tdm_compensate", "virtual_array", "azimuth_fft", "elevation", "N_AZ", "AZIMUTHS",
           "ca_cfar_range", "azimuth_peaks", "detect", "to_xyz", "spectra"]
