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
    