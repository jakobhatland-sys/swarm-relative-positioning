import numpy as np
from src.swarm.simulation import init_swarm
from src.swarm.sensors import knn_pairs, uwb_ranges

def test_ranges_without_noise_equal_true_distance():
    pos = init_swarm(16)
    pairs = knn_pairs(pos, k=4)
    i, j = np.array(pairs).T
    assert np.allclose(uwb_ranges(pos, pairs, sigma=0.0), np.linalg.norm(pos[j] - pos[i], axis=1))

def test_each_drone_has_at_least_four_neighbors():
    """Nødvendig (ikke tilstrekkelig) krav for unik 3D-lokalisering: d+1 = 4."""
    pairs = knn_pairs(init_swarm(64), k=6)
    degree = np.bincount(np.array(pairs).ravel(), minlength=64)
    assert degree.min() >= 4
    