import sys; sys.path.insert(0, ".")
import matplotlib.pyplot as plt
from src.swarm.simulation import init_swarm

pos = init_swarm(n=64)
ax = plt.figure().add_subplot(projection="3d")
ax.scatter(pos[:, 0], pos[:, 1], pos[:, 2])
ax.set_xlabel("Øst [m]"); ax.set_ylabel("Nord [m]"); ax.set_zlabel("Opp [m]")
plt.title("Sverm: 64 droner (ENU)"); plt.show()