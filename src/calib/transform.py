"""Apply the solved radar -> LiDAR extrinsics.

    from src.calib.transform import RadarToLidar
    T = RadarToLidar.load()                 # calib/extrinsics_radar_lidar.json
    xyz_l = T(xyz_r)                        # (N, 3) radar frame -> lidar frame
    xyz_r = T.inverse(xyz_l)

The solve is 2D (yaw, tx, ty). z is passed through with an optional fixed offset, so points keep the
radar's (noisy) elevation unless you flatten them first.
"""
import json, os

import numpy as np

DEFAULT = os.path.join(os.path.dirname(__file__), "..", "..", "calib", "extrinsics_radar_lidar.json")


class RadarToLidar:
    def __init__(self, yaw_deg, t_xy, t_z=0.0):
        c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
        self.R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        self.t = np.array([t_xy[0], t_xy[1], t_z])
        self.yaw_deg = yaw_deg

    @classmethod
    def load(cls, path=DEFAULT):
        j = json.load(open(path))
        return cls(j["yaw_deg"], j["t_xy_m"], j["t_z_m"] or 0.0)

    def __call__(self, xyz):
        return np.asarray(xyz) @ self.R.T + self.t

    def inverse(self, xyz):
        return (np.asarray(xyz) - self.t) @ self.R


DEFAULT_CAM = os.path.join(os.path.dirname(__file__), "..", "..", "calib", "extrinsics_radar_camera.json")


class RadarToCamera:
    """Radar frame -> camera frame (x right, y down, z forward) and on to fisheye pixels.

        C = RadarToCamera.load()
        xyz_c = C(xyz_r)                 # camera-frame metres
        uv = C.pixels(xyz_r)             # (N, 2) pixels in the 1920x1080 frame
    """

    def __init__(self, R, t, cam):
        self.R, self.t, self.cam = np.asarray(R, float), np.asarray(t, float), cam

    @classmethod
    def load(cls, path=DEFAULT_CAM):
        j = json.load(open(path))
        cam = json.load(open(os.path.join(os.path.dirname(path), os.path.basename(j["intrinsics"]))))
        return cls(j["R"], j["t_m"], cam)

    def __call__(self, xyz):
        return np.asarray(xyz) @ self.R.T + self.t

    def pixels(self, xyz):
        from .board_image import project_fisheye
        return project_fisheye(self(xyz), self.cam)
