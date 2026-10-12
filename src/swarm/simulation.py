import numpy as np

def init_swarm(n=64, spacing=10.0, seed=0):
    """Plasserer n droner i et rutenett med litt tilfeldig støy (meter, ENU)."""
    rng = np.random.default_rng(seed)
    side = int(np.ceil(np.sqrt(n)))
    grid = np.array([(i % side, i // side, 0) for i in range(n)]) * spacing
    return grid + rng.normal(0, 1.0, grid.shape) + [0, 0, 50]  # 50 m høyde

#En Gauss-Markov-prosess trekker hver drone tilbake mot felles hastighet. 
#Svermen holder dermed formasjonen, men med realistisk individuell variasjon. 
#Standardverdiene gir 600 steg × 0,1 s = 60 s flyging.

def simulate(pos0, steps=600, dt=0.1, v=(2.0, 0.0, 0.0), tau=5.0, sigma_v=0.3, seed=1):
    """Felles hastighet v + individuell Gauss-Markov-avvik. Returnerer (steps, n, 3)."""
    rng = np.random.default_rng(seed)
    v = np.asarray(v, float)
    vel = np.tile(v, (len(pos0), 1))
    traj = np.empty((steps, len(pos0), 3)); traj[0] = pos0
    for k in range(1, steps):
        vel += -(vel - v) / tau * dt + rng.normal(0, sigma_v * np.sqrt(dt), vel.shape)
        traj[k] = traj[k-1] + vel * dt
    return traj