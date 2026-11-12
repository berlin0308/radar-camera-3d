"""A dense depth map seen in 3D: each pixel's ray times its depth, coloured by the camera, drawn from another viewpoint.

This is the 3D output of the dense-depth step: the same camera frame lands on a different surface
for each fill, so a wrong depth shows as a picture pasted on the wrong wall.
"""
import json, os

import numpy as np

from ..calib.transform import joint_path


def intrinsics(trace):
    path = joint_path(trace); j = json.load(open(path))
    return json.load(open(os.path.join(os.path.dirname(path), os.path.basename(j["radar_to_camera"]["intrinsics"]))))


def rays(cam, H, W):
    """Unit rays (H, W, 3) of the OpenCV fisheye model (x right, y down, z forward) for an image of H x W pixels."""
    sc = W / cam["w"]
    u, v = np.meshgrid((np.arange(W) + .5) / sc, (np.arange(H) + .5) / sc)
    a, b = (u - cam["cx"]) / cam["fl_x"], (v - cam["cy"]) / cam["fl_y"]; thd = np.hypot(a, b); th = thd.copy()
    k = [cam["k1"], cam["k2"], cam["k3"], cam["k4"]]
    for _ in range(20):                                                  # invert thd = th (1 + k1 th^2 + ...) by Newton
        f = th * (1 + k[0] * th**2 + k[1] * th**4 + k[2] * th**6 + k[3] * th**8) - thd
        th = th - f / (1 + 3 * k[0] * th**2 + 5 * k[1] * th**4 + 7 * k[2] * th**6 + 9 * k[3] * th**8)
    s = np.where(thd > 0, np.sin(th) / np.maximum(thd, 1e-12), 1.0)
    return np.stack([a * s, b * s, np.cos(th)], -1)


def render(R, D, img, yaw=35.0, pitch=25.0, pivot=5.0, back=4.0, size=(270, 480), f=300.0, dmax=12.0):
    """Coloured points R * D seen from a camera turned `yaw` deg to the side and `pitch` deg down about a point `pivot` m ahead."""
    m = np.isfinite(D) & (D < dmax) & (R[..., 2] > 0.2)
    P, C = R[m] * D[m][:, None], img[m]
    cy, sy = np.cos(np.radians(yaw)), np.sin(np.radians(yaw)); cp, sp = np.cos(np.radians(pitch)), np.sin(np.radians(pitch))
    Ry = np.array([[cy, 0, -sy], [0, 1, 0], [sy, 0, cy]]); Rx = np.array([[1, 0, 0], [0, cp, -sp], [0, sp, cp]])
    Q = (P - [0, 0, pivot]) @ (Rx @ Ry).T + [0, 0, pivot + back]
    ok = Q[:, 2] > 0.5; Q, C = Q[ok], C[ok]
    x = (f * Q[:, 0] / Q[:, 2] + size[1] / 2).astype(int); y = (f * Q[:, 1] / Q[:, 2] + size[0] / 2).astype(int)
    canvas = np.ones(size + (3,)); o = np.argsort(-Q[:, 2])             # far first, near drawn last
    for dy in (0, 1):
        for dx in (0, 1):
            xx, yy = x[o] + dx, y[o] + dy; k = (xx >= 0) & (xx < size[1]) & (yy >= 0) & (yy < size[0])
            canvas[yy[k], xx[k]] = C[o][k]
    return canvas
