import sys; sys.path.insert(0, ".")
import numpy as np
from scipy.stats import chi2
from src.swarm.simulation import init_swarm, simulate
from src.swarm.sensors import knn_pairs, uwb_ranges
from src.swarm.estimator import SwarmEKF
from src.swarm.metrics import rms, relative_rms, nees_position

def pick_anchors(pos, m):
    """Farthest point sampling: spre ankrene for best mulig geometri."""
    ids = [int(np.argmin(np.linalg.norm(pos - pos.mean(0), axis=1)))]
    while len(ids) < m:
        d = np.min(np.linalg.norm(pos[:, None] - pos[ids][None], axis=2), axis=1)
        ids.append(int(np.argmax(d)))
    return ids

def run_trial(n_anchors, k, seed, n=64, steps=300, dt=0.1):
    rng = np.random.default_rng(seed)
    truth = simulate(init_swarm(n, seed=seed), steps=steps, dt=dt, seed=seed)
    anchors = pick_anchors(truth[0], n_anchors)
    ekf = SwarmEKF(truth[0] + rng.normal(0, 2.0, (n, 3)))
    for t in range(1, steps):
        p = truth[t]
        ekf.predict(dt)
        pairs = knn_pairs(p, k=k)
        ekf.update_ranges(pairs, uwb_ranges(p, pairs, 0.1, rng), 0.1)
        ekf.update_anchors(anchors, p[anchors] + rng.normal(0, 0.02, (n_anchors, 3)), 0.02)
    p = truth[-1]
    return rms(ekf.pos - p), relative_rms(ekf.pos, p), nees_position(ekf, p)

M = 5                                               # Monte Carlo-kjøringer per kombinasjon
lo, hi = chi2.ppf([0.025, 0.975], M * 192) / (M * 192)
print(f"95 % konsistensintervall for ANEES: [{lo:.2f}, {hi:.2f}]")
print(f"{'ankre':>5} {'k':>3} {'abs RMS [m]':>12} {'rel RMS [m]':>12} {'ANEES':>7}")
for k in (4, 6, 8):
    for m in (1, 2, 4, 8, 16):
        a, r, nees = np.mean([run_trial(m, k, s) for s in range(M)], axis=0)
        print(f"{m:>5} {k:>3} {a:>12.3f} {r:>12.3f} {nees:>7.2f}", flush=True)