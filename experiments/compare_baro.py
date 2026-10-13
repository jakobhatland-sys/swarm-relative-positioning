import sys; sys.path.insert(0, ".")
import numpy as np
from experiments.anchor_sweep import run_trial

M, anchors = 5, 8
print(f"{'k':>3} {'barometer':>10} {'konv.':>6} {'rel RMS [m]':>12} {'ANEES':>7}")
for k in (4, 6):
    for baro in (None, 0.3):
        res = np.array([run_trial(anchors, k, s, init_sigma=1.0, iters=5, baro_sigma=baro) for s in range(M)])
        ok = res[:, 1] < 0.3
        r, nees = np.median(res[ok][:, 1:], axis=0) if ok.any() else (np.nan, np.nan)
        label = "nei" if baro is None else f"σ={baro} m"
        print(f"{k:>3} {label:>10} {ok.sum():>3}/{M} {r:>12.3f} {nees:>7.2f}", flush=True)