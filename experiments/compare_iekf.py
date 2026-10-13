import sys; sys.path.insert(0, ".")
import numpy as np
from experiments.anchor_sweep import run_trial

M, anchors = 5, 8
print(f"{'init σ':>6} {'k':>3} {'metode':>7} {'konv.':>6} {'rel RMS [m]':>12} {'ANEES':>7}")
for init_sigma in (1.0, 2.0):
    for k in (4, 6, 8):
        for name, iters in (("EKF", 1), ("IEKF", 5)):
            res = np.array([run_trial(anchors, k, s, init_sigma=init_sigma, iters=iters) for s in range(M)])
            ok = res[:, 1] < 0.3
            r, nees = np.median(res[ok][:, 1:], axis=0) if ok.any() else (np.nan, np.nan)
            print(f"{init_sigma:>6} {k:>3} {name:>7} {ok.sum():>3}/{M} {r:>12.3f} {nees:>7.2f}", flush=True)