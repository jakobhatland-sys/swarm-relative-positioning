import numpy as np

def knn_pairs(pos, k=6, max_range=50.0):
    """Unike nabopar (i, j) innen rekkevidde, k nærmeste per drone."""
    d = np.linalg.norm(pos[:, None] - pos[None], axis=2)
    np.fill_diagonal(d, np.inf)
    pairs = set()
    for i, row in enumerate(d):
        for j in np.argsort(row)[:k]:
            if row[j] <= max_range:
                pairs.add((min(i, int(j)), max(i, int(j))))
    return sorted(pairs)

def uwb_ranges(pos, pairs, sigma=0.1, rng=None):
    """Simulerte UWB-avstander [m] med gaussisk støy (typisk TWR: ~0,1 m)."""
    rng = rng or np.random.default_rng()
    i, j = np.array(pairs).T
    true = np.linalg.norm(pos[j] - pos[i], axis=1)
    return true + rng.normal(0, sigma, true.shape)
