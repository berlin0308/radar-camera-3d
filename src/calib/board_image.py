"""Find the white calibration board in each camera frame of the 10/21 calibration shots.

The board is the largest bright, low-texture blob near where the radar detection projects under
the co-located guess (camera at the radar, axes aligned). Writes calib/board_pixels.json and a
check figure; look at the figure before trusting the centres.

    python -m src.calib.board_image
"""
import json, os, subprocess

import numpy as np
from scipy import ndimage

from ..radar import RadarTrace
from .radar_lidar import SHOTS, R_MAX, radar_board

CAM = "calib/camera_fisheye_iq1m.json"


def frame_at(trace_dir, t_unix, path):
    """Full-resolution camera frame nearest Unix time t_unix, saved to path; returns the array."""
    ts = np.fromfile(os.path.join(trace_dir, "camera", "ts"), "<f8"); k = int(np.searchsorted(ts, t_unix))
    subprocess.run(["ffmpeg", "-v", "error", "-y", "-ss", f"{k / 30:.3f}", "-i", os.path.join(trace_dir, "camera", "video.avi"),
                    "-frames:v", "1", path], check=True)
    import matplotlib.image as mpimg
    return mpimg.imread(path)


def project_fisheye(xyz_cam, cam):
    """OpenCV fisheye model: (N, 3) camera-frame points (x right, y down, z forward) -> (N, 2) pixels."""
    x, y, z = xyz_cam.T
    a, b = x / z, y / z
    r = np.hypot(a, b); th = np.arctan(r)
    thd = th * (1 + cam["k1"] * th**2 + cam["k2"] * th**4 + cam["k3"] * th**6 + cam["k4"] * th**8)
    s = np.where(r > 0, thd / r, 1.0)
    return np.stack([cam["fl_x"] * a * s + cam["cx"], cam["fl_y"] * b * s + cam["cy"]], axis=-1)


def radar_to_cam_axes(xyz_r):
    """Radar (x right, y forward, z up) -> camera axes (x right, y down, z forward), no offset."""
    return np.stack([xyz_r[..., 0], -xyz_r[..., 2], xyz_r[..., 1]], axis=-1)


def find_board(img, guess, lo=0.40, hi=0.85, min_area=15000, max_area=1500000):
    """Centroid, bbox and mask of the board: a large, roughly square, mid-grey, smooth blob nearest the guessed pixel.

    The board is lit grey-white (0.4 to 0.85 of full scale); ceiling lights are brighter and are
    excluded by the upper bound, shelves and screens by the texture test.
    """
    g = img[..., :3].mean(axis=-1)
    g = g / 255 if g.max() > 1.5 else g
    smooth = ndimage.uniform_filter(g, 9)
    local_std = np.sqrt(np.maximum(ndimage.uniform_filter(g**2, 9) - smooth**2, 0))
    mask = (smooth > lo) & (smooth < hi) & (local_std < 0.03)
    mask = ndimage.binary_opening(mask, iterations=4)
    lab, n = ndimage.label(mask)
    best, best_d = None, 1e9
    for i in range(1, n + 1):
        m = lab == i; area = int(m.sum())
        if not (min_area <= area <= max_area): continue
        ys, xs = np.nonzero(m); c = np.array([xs.mean(), ys.mean()])
        w, h = np.ptp(xs) + 1, np.ptp(ys) + 1
        if not (0.5 < w / h < 2.0) or area < 0.5 * w * h: continue   # square-ish and filled, not a wall strip
        std = g[m].std()
        d = np.hypot(*(c - guess))
        if d < best_d: best, best_d = dict(centre=c, bbox=[int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())], area=area, std=float(std)), d
    return best


# Hand seeds (pixel on the board, read off the check figure) for shots where the automatic search
# picks the wall or nothing; the blob is then grown from the seed inside a window sized by the range.
SEEDS = {"calib_01": (1069, 684), "calib_02": (637, 645), "calib_03b": (939, 712), "calib_04": (979, 489), "calib_05b": (1069, 712)}


def grow_board(img, seed, r, cam, board_m=0.6, tol=0.08):
    """Blob of similar brightness and low texture around the seed, limited to the board's expected image size."""
    g = img[..., :3].mean(axis=-1); g = g / 255 if g.max() > 1.5 else g
    smooth = ndimage.uniform_filter(g, 9)
    local_std = np.sqrt(np.maximum(ndimage.uniform_filter(g**2, 9) - smooth**2, 0))
    half = int(0.75 * cam["fl_x"] * board_m / r)
    x0, y0 = max(0, seed[0] - half), max(0, seed[1] - half); x1, y1 = min(g.shape[1], seed[0] + half), min(g.shape[0], seed[1] + half)
    s0 = smooth[seed[1], seed[0]]
    win = np.zeros_like(g, bool); win[y0:y1, x0:x1] = True
    mask = win & (np.abs(smooth - s0) < tol) & (local_std < 0.03)
    lab, n = ndimage.label(mask); k = lab[seed[1], seed[0]]
    if k == 0: return None
    m = lab == k; ys, xs = np.nonzero(m)
    return dict(centre=np.array([xs.mean(), ys.mean()]), bbox=[int(xs.min()), int(ys.min()), int(xs.max()), int(ys.max())], area=int(m.sum()), std=float(g[m].std()))


def run(out_json="calib/board_pixels.json", fig_path="figures/calib_board_in_images.png", frame_dir="data/sessions/calib_frames"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    cam = json.load(open(CAM)); os.makedirs(frame_dir, exist_ok=True)
    fig, axes = plt.subplots(2, 4, figsize=(16.64, 5.6), constrained_layout=True)
    out = {}
    for ax, (name, trace) in zip(axes.flat, SHOTS.items()):
        d = os.path.join("data/raw", trace); rb = radar_board(d, r_max=R_MAX.get(name, 5.0))
        img = frame_at(d, rb["t"], os.path.join(frame_dir, f"{name}.png"))
        xyz_r = np.array([[-rb["r"] * np.sin(rb["az"]), rb["r"] * np.cos(rb["az"]), 0.0]])   # z = 0: radar elevation is unusable
        guess = project_fisheye(radar_to_cam_axes(xyz_r), cam)[0]
        b = grow_board(img, SEEDS[name], rb["r"], cam) if name in SEEDS else find_board(img, guess)
        ax.imshow(img); ax.scatter(*guess, marker="x", s=80, color="#2C43B0", lw=2)
        if b:
            x0, y0, x1, y1 = b["bbox"]; ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, color="#E07B39", lw=2))
            ax.scatter(*b["centre"], marker="+", s=120, color="#E07B39", lw=2)
            out[name] = dict(centre=[float(b["centre"][0]), float(b["centre"][1])], bbox=b["bbox"], area=b["area"],
                             radar=dict(r=rb["r"], az=rb["az"], el=rb["el"]), t=rb["t"])
            print(f"{name}: board centre ({b['centre'][0]:.0f}, {b['centre'][1]:.0f}) px, guess ({guess[0]:.0f}, {guess[1]:.0f}), area {b['area']}")
        else:
            print(f"{name}: board not found")
        ax.set_title(f"{name}: r {rb['r']:.2f} m, az {np.degrees(rb['az']):+.0f}°", fontsize=11); ax.axis("off")
    fig.suptitle("Board in the camera (orange box and +) and the radar detection projected with no extrinsics (blue x)", fontsize=12)
    os.makedirs(os.path.dirname(fig_path), exist_ok=True); fig.savefig(fig_path, dpi=120)
    json.dump(out, open(out_json, "w"), indent=1); print("saved", out_json, fig_path)


if __name__ == "__main__":
    run()
