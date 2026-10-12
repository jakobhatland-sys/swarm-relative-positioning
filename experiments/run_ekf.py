import sys; sys.path.insert(0, ".")
import numpy as np, matplotlib.pyplot as plt
from src.swarm.simulation import init_swarm, simulate
from src.swarm.sensors import knn_pairs, uwb_ranges
from src.swarm.estimator import SwarmEKF

dt, n = 0.1, 64
anchors = [0, 7, 56, 63, 27]                       # 4 hjørner + 1 sentralt
rng = np.random.default_rng(2)
truth = simulate(init_swarm(n), dt=dt)
ekf = SwarmEKF(truth[0] + rng.normal(0, 2.0, (n, 3)))  # init: 2 m feil

rms = []
for k in range(1, len(truth)):
    p = truth[k]
    ekf.predict(dt)
    pairs = knn_pairs(p, k=6)
    ekf.update_ranges(pairs, uwb_ranges(p, pairs, 0.1, rng), 0.1)
    ekf.update_anchors(anchors, p[anchors] + rng.normal(0, 0.02, (len(anchors), 3)), 0.02)
    rms.append(np.sqrt(np.mean(np.sum((ekf.pos - p)**2, axis=1))))

e = ekf.pos - truth[-1]
print(f"RMS horisontal: {np.sqrt(np.mean(e[:, 0]**2 + e[:, 1]**2)):.3f} m")
print(f"RMS vertikal:   {np.sqrt(np.mean(e[:, 2]**2)):.3f} m")
plt.plot(np.arange(1, len(truth)) * dt, rms)
plt.xlabel("Tid [s]"); plt.ylabel("3D RMS-feil [m]"); plt.yscale("log"); plt.grid(); plt.show()