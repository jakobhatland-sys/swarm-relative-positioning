import numpy as np
from src.swarm.simulation import init_swarm
from src.swarm.sensors import knn_pairs
from src.swarm.estimator import SwarmEKF

def test_range_jacobian_matches_numerical():
    ekf = SwarmEKF(init_swarm(9))
    pairs = knn_pairs(ekf.pos, k=3)
    rho0, H = ekf.range_model(pairs)
    eps, Hn = 1e-6, np.zeros_like(H)
    for s in range(len(ekf.x)):
        ekf.x[s] += eps; Hn[:, s] = (ekf.range_model(pairs)[0] - rho0) / eps; ekf.x[s] -= eps
    assert np.allclose(H, Hn, atol=1e-5)
from src.swarm.sensors import uwb_ranges

def test_iekf_with_one_iteration_equals_ekf():
    p = init_swarm(9); pairs = knn_pairs(p, k=4)
    r = uwb_ranges(p, pairs, sigma=0.0)
    p0 = p + np.random.default_rng(0).normal(0, 0.5, p.shape)
    a, b = SwarmEKF(p0), SwarmEKF(p0)
    a.update_ranges(pairs, r, 0.1, iters=1)
    rho, H = b.range_model(pairs); b._update(r, rho, H, 0.01 * np.eye(len(r)))
    assert np.allclose(a.x, b.x) and np.allclose(a.P, b.P)