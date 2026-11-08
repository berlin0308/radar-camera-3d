"""Keep only the Gaussians that are on the object, and put them at the origin, for the splat viewer.

    ~/miniconda3/envs/kplanes/bin/python src/ext/obj_rgb/gs_filter_ckpt.py \
        /data/christineh/gs_scotty/runs/scotty/splatfacto/posed96 \
        --poses out/da3_scotty/poses96.npz --data /data/christineh/gs_scotty/data_obj \
        --out /data/christineh/gs_scotty/runs/scotty/splatfacto/objonly

Runs in the `kplanes` env. Edits a trained run rather than retraining it: the Gaussians are filtered
and translated in the checkpoint, so nothing is optimised again.

Two problems with pointing `ns-viewer` at the raw run, both fixed here:

The **courtyard is noise for this purpose.** The splat models everything the cameras saw, and the walls
5–15 m out are seen from a narrow range of angles, so the optimiser fits them with very long thin
Gaussians. Those are the white needles that fill the viewport. Keeping only the Gaussians whose centre
is on the object, and dropping any that are still longer than --max-scale (a needle whose centre
happens to fall inside the cut-out would otherwise spear straight through it), leaves the statue.

The **viewer orbits the world origin and zooms toward it**, and that origin is the first LiDAR scan,
16 m away in the middle of the courtyard, so the object can never be reached however far you scroll.
Every surviving Gaussian is therefore translated by the same offset that `export_nerfstudio --origin
object` applies to the cameras, which puts the statue at the origin. Point the new run at that dataset
(--data) and the train-camera frustums line up with it.

Scales are stored as log scale and opacities as logits, as splatfacto keeps them.
"""
import argparse, os, shutil

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run", help="the trained run dir (holds config.yml and nerfstudio_models/)")
    ap.add_argument("--poses", required=True, help="npz from src.ext.obj_rgb.export_poses")
    ap.add_argument("--data", required=True, help="dataset dir the new run should point at")
    ap.add_argument("--out", required=True)
    ap.add_argument("--radius", type=float, default=1.2, help="cut-out radius about the object (m)")
    ap.add_argument("--z", type=float, nargs=2, default=(0.02, 2.6), help="height band above the ground (m)")
    ap.add_argument("--opacity", type=float, default=0.1, help="drop Gaussians fainter than this")
    ap.add_argument("--max-scale", type=float, default=0.35,
                    help="drop Gaussians whose longest axis exceeds this (m): the needles")
    ap.add_argument("--keep-world", action="store_true", help="do not translate to the object")
    a = ap.parse_args()

    import torch
    ck_dir = os.path.join(a.run, "nerfstudio_models")
    ck_name = sorted(os.listdir(ck_dir))[-1]
    ck = torch.load(os.path.join(ck_dir, ck_name), map_location="cpu", weights_only=False)
    pl = ck["pipeline"]
    pre = "_model.gauss_params."
    keys = [k for k in pl if k.startswith(pre)]
    means = pl[pre + "means"].numpy()
    n0 = len(means)

    P = np.load(a.poses, allow_pickle=True)
    c_ref, ground = P["c_ref"], float(np.median(P["ground_frame"]))
    shift = np.r_[c_ref, ground]
    rel = means - shift
    rho = np.hypot(rel[:, 0], rel[:, 1])
    scale = np.exp(pl[pre + "scales"].numpy()).max(axis=1)
    alpha = 1.0 / (1.0 + np.exp(-pl[pre + "opacities"].numpy().squeeze(-1)))

    inside = (rho < a.radius) & (rel[:, 2] > a.z[0]) & (rel[:, 2] < a.z[1])
    bright = alpha > a.opacity
    compact = scale < a.max_scale
    keep = inside & bright & compact
    print(f"{n0:,} Gaussians")
    print(f"  on the object (r < {a.radius} m, h {a.z[0]}–{a.z[1]} m): {inside.sum():,}")
    print(f"  of those, opacity > {a.opacity}: {(inside & bright).sum():,}")
    print(f"  of those, longest axis < {a.max_scale} m: {keep.sum():,}"
          f"  (dropped {int((inside & bright & ~compact).sum()):,} needles, longest {scale[inside].max():.2f} m)")
    if not keep.any():
        raise SystemExit("nothing survived; loosen --radius / --opacity / --max-scale")

    idx = torch.from_numpy(np.flatnonzero(keep))
    for k in keys:
        pl[k] = pl[k][idx].contiguous()
    if not a.keep_world:
        pl[pre + "means"] = pl[pre + "means"] - torch.tensor(shift, dtype=pl[pre + "means"].dtype)
        print(f"  translated by {-shift.round(3)} m: the object is now at the origin")
    ck["optimizers"] = {}          # the Adam state refers to the old count; the viewer does not train

    out_ck = os.path.join(a.out, "nerfstudio_models")
    os.makedirs(out_ck, exist_ok=True)
    torch.save(ck, os.path.join(out_ck, ck_name))

    cfg = open(os.path.join(a.run, "config.yml")).read()
    old_data = os.path.realpath(_data_of(cfg))
    cfg = cfg.replace(_block(old_data), _block(os.path.realpath(a.data)))
    cfg = cfg.replace(f"timestamp: {os.path.basename(a.run)}", f"timestamp: {os.path.basename(a.out)}")
    open(os.path.join(a.out, "config.yml"), "w").write(cfg)
    for f in ("dataparser_transforms.json",):
        s = os.path.join(a.run, f)
        if os.path.isfile(s):
            shutil.copy(s, os.path.join(a.out, f))
    print(f"\nwrote {a.out}  ({os.path.getsize(os.path.join(out_ck, ck_name)) / 1e6:.0f} MB)")
    print("view it with:\n  ~/miniconda3/envs/kplanes/bin/ns-viewer --load-config "
          f"{os.path.join(a.out, 'config.yml')} --viewer.websocket-port 7007 --viewer.websocket-host 127.0.0.1")


def _block(path):
    """A PosixPath as pyyaml writes it, so the data dir can be swapped by text."""
    return "\n".join("- " + p for p in ["/"] + [c for c in path.strip("/").split("/")])


def _data_of(cfg):
    lines = cfg.splitlines()
    i = next(j for j, l in enumerate(lines) if l.startswith("data: "))
    parts = []
    for l in lines[i + 1:]:
        if not l.startswith("- "):
            break
        parts.append(l[2:])
    return "/".join(p for p in parts if p != "/")


if __name__ == "__main__":
    main()
