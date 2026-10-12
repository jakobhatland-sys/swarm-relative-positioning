import numpy as np
from scipy.spatial.transform import Rotation
from src.swarm.simulation import init_swarm
from src.swarm.metrics import relative_rms

def test_relative_rms_ignores_rotation_and_translation():
    p = init_swarm(16)
    q = p @ Rotation.from_euler("xyz", [10, -20, 35], degrees=True).as_matrix().T + [100, -50, 7]
    assert relative_rms(q, p) < 1e-9
    