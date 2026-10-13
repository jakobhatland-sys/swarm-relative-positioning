import sys; sys.path.insert(0, ".")
import numpy as np, matplotlib.pyplot as plt
from src.swarm.simulation import init_swarm
from src.swarm.sensors import knn_pairs
from experiments.anchor_sweep import pick_anchors

pos = init_swarm(64)
anchors = pick_anchors(pos, 8)
views = ((0, 1, "Øst [m]", "Nord [m]", "ovenfra"), (0, 2, "Øst [m]", "Opp [m]", "fra siden"))

fig, axes = plt.subplots(2, 2, figsize=(13, 10))
for col, k in enumerate((4, 8)):
    pairs = knn_pairs(pos, k=k)
    deg = np.bincount(np.array(pairs).ravel(), minlength=len(pos))
    for row, (a, b, xl, yl, view) in enumerate(views):
        ax = axes[row, col]
        for i, j in pairs:
            ax.plot(pos[[i, j], a], pos[[i, j], b], color="0.75", lw=0.8, zorder=1)
        sc = ax.scatter(pos[:, a], pos[:, b], c=deg, cmap="viridis", s=45, zorder=2)
        ax.scatter(pos[anchors, a], pos[anchors, b], marker="^", s=160, facecolors="none",
                   edgecolors="red", lw=2, zorder=3, label="Anker (RTK)")
        ax.set_xlabel(xl); ax.set_ylabel(yl)
        ax.set_title(f"k = {k}: {len(pairs)} UWB-linker ({view})")
        if row == 0:
            ax.set_aspect("equal")
        fig.colorbar(sc, ax=ax, label="Antall naboer (grad)")
axes[0, 0].legend(loc="upper left")
plt.tight_layout()
plt.savefig("figures/uwb_network.png", dpi=150)
plt.show()