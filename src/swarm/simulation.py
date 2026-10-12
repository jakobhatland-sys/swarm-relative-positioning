import numpy as np

def init_swarm(n=64, spacing=10.0, seed=0):
    """Plasserer n droner i et rutenett med litt tilfeldig støy (meter, ENU)."""
    rng = np.random.default_rng(seed)
    side = int(np.ceil(np.sqrt(n)))
    grid = np.array([(i % side, i // side, 0) for i in range(n)]) * spacing
    return grid + rng.normal(0, 1.0, grid.shape) + [0, 0, 50]  # 50 m høyde