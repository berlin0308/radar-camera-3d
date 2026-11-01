"""Run a released GRT checkpoint on one of our traces and save the 3D occupancy per frame.

Writes <out>/<trace>/occ3d_logit.npy  float16 [N, 64 el, 128 az, 64 rng]   model output (logits)
       <out>/<trace>/occ3d_label.npy  bool    [N, 64 el, 128 az, 64 rng]   nrdk's own LiDAR label for the same frames
       <out>/<trace>/ts.npy           float64 [N]                           radar timestamps

    ~/nrdk-venv/bin/python -m src.grt.infer <results_dir> data/raw/2026-10-29.16-22-36 --out data/grt --batch 16

<results_dir> is an nrdk Result directory (.hydra/config.yaml, checkpoints/, checkpoints.yaml), e.g.
iq1m-checkpoints/base/small as downloaded from OSN.
"""
import argparse, os, time

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("results"); ap.add_argument("trace")
    ap.add_argument("--out", default="data/grt"); ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--device", default="cuda:0"); ap.add_argument("--max-frames", type=int, default=None)
    ap.add_argument("--start", type=float, default=None, help="seconds into the trace to start")
    ap.add_argument("--seconds", type=float, default=None, help="length of the window to run")
    a = ap.parse_args()

    import hydra, optree, torch
    from abstract_dataloader.generic import DatasetMeta
    from nrdk.framework import Result

    result = Result(a.results); cfg = result.config()
    cfg["meta"]["dataset"] = os.path.dirname(os.path.abspath(a.trace.rstrip("/")))
    cfg["datamodule"]["batch_size"] = a.batch; cfg["datamodule"]["num_workers"] = 8; cfg["datamodule"]["prefetch_factor"] = 2
    transforms = hydra.utils.instantiate(cfg["transforms"])
    lm = hydra.utils.instantiate(cfg["lightningmodule"], transforms=transforms).to(a.device)
    lm.load_weights(result.best); lm.eval()

    datamodule = hydra.utils.instantiate(cfg["datamodule"], transforms=transforms)
    dataset_constructor = hydra.utils.instantiate(cfg["datamodule"]["dataset"])
    dataset = DatasetMeta(dataset_constructor(paths=[os.path.abspath(a.trace)]), meta={"train": False, "split": "test"})
    dl = datamodule.dataloader(dataset, mode="test")

    name = os.path.basename(a.trace.rstrip("/")); out = os.path.join(a.out, name); os.makedirs(out, exist_ok=True)
    t_first = float(np.fromfile(os.path.join(a.trace, "radar", "ts"), "<f8")[0])
    lo = t_first + (a.start or 0.0); hi = lo + a.seconds if a.seconds is not None else np.inf
    logits, labels, ts = [], [], []
    t0 = time.perf_counter(); n = 0
    with torch.no_grad():
        for batch in dl:
            b_ts = batch["occ3d"].timestamps if isinstance(batch, dict) else batch.occ3d.timestamps
            b_ts = np.asarray(b_ts).reshape(b_ts.shape[0], -1)[:, -1]
            if b_ts.max() < lo: continue                            # before the window: skip the model
            if b_ts.min() >= hi: break                               # past the window: done
            y_true = lm.transform(batch, device=a.device)        # nrdk 0.2.x: transform, then forward(y_true) -> y_pred
            y_pred = lm(y_true)
            pred = y_pred["occ3d"] if isinstance(y_pred, dict) else getattr(y_pred, "occ3d")
            lab = y_true["occ3d"] if isinstance(y_true, dict) else getattr(y_true, "occ3d")
            lab_occ = lab.occupancy if hasattr(lab, "occupancy") else lab["occupancy"]
            lab_ts = lab.timestamps if hasattr(lab, "timestamps") else lab["timestamps"]
            pred = pred.reshape(pred.shape[0], -1, *pred.shape[-3:])[:, -1]      # (batch, t, el, az, rng) -> last t
            lab_occ = lab_occ.reshape(lab_occ.shape[0], -1, *lab_occ.shape[-3:])[:, -1]
            lab_ts = lab_ts.reshape(lab_ts.shape[0], -1)[:, -1]
            logits.append(pred.to(torch.float16).cpu().numpy()); labels.append(lab_occ.bool().cpu().numpy()); ts.append(lab_ts.double().cpu().numpy())
            n += pred.shape[0]
            if a.max_frames and n >= a.max_frames: break
    logits = np.concatenate(logits); labels = np.concatenate(labels); ts = np.concatenate(ts)
    m = (ts >= lo) & (ts < hi)                                        # trim the partial first/last batches to the window
    logits, labels, ts = logits[m], labels[m], ts[m]
    np.save(os.path.join(out, "occ3d_logit.npy"), logits); np.save(os.path.join(out, "occ3d_label.npy"), labels); np.save(os.path.join(out, "ts.npy"), ts)
    print(f"{name}: {len(ts)} frames in {time.perf_counter() - t0:.0f} s, logits {logits.shape} {logits.dtype}, "
          f"label occupancy {labels.mean()*100:.2f}% of cells, pred>0 {(logits > 0).mean()*100:.2f}% -> {out}")


if __name__ == "__main__":
    main()
