import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

data = np.genfromtxt('/home/vladimir/Documents/programs/EnvironmentMap.txt', delimiter=' ', comments='#')
data = data[~np.isnan(data).any(axis=1)]

fig, ax = plt.subplots(figsize=(10, 10))

wall_points = 4 * 251
points_per_pillar = 18
pillar_count = 4
charger_points = 18
start = wall_points

walls = data[0:wall_points]
ax.scatter(walls[:,0], walls[:,1], s=2, c='black', label='Walls')

# Рисуем каждую колонну отдельно
for p in range(pillar_count):
    pillar_start = start + p * points_per_pillar
    pillar = data[pillar_start:pillar_start + points_per_pillar]
    cx, cy = np.mean(pillar[:,0]), np.mean(pillar[:,1])
    angles = np.arctan2(pillar[:,1] - cy, pillar[:,0] - cx)
    sorted_idx = np.argsort(angles)
    pillar_sorted = pillar[sorted_idx]
    ax.add_patch(Polygon(pillar_sorted, closed=True, fill=True, facecolor='blue', alpha=0.3, edgecolor='blue'))
    ax.scatter(pillar[:,0], pillar[:,1], s=2, c='blue')

start = wall_points + pillar_count * points_per_pillar

charger = data[start:start+charger_points]
cx, cy = np.mean(charger[:,0]), np.mean(charger[:,1])
angles = np.arctan2(charger[:,1] - cy, charger[:,0] - cx)
sorted_idx = np.argsort(angles)
charger_sorted = charger[sorted_idx]
ax.add_patch(Polygon(charger_sorted, closed=True, fill=True, facecolor='yellow', alpha=0.5, edgecolor='orange'))
ax.scatter(charger[:,0], charger[:,1], s=2, c='orange', label='Charger')

start += charger_points

robot = data[start:]
ax.scatter(robot[:,0], robot[:,1], s=5, c='red', label='Robot')

ax.set_title('Task B: Environment Map')
ax.set_xlabel('X (m)')
ax.set_ylabel('Y (m)')
ax.grid(True)
ax.set_aspect('equal')
ax.set_xlim(-0.1, 2.6)
ax.set_ylim(-0.1, 2.6)
ax.legend(loc='upper right', markerscale=3)

fig.savefig('/home/vladimir/Documents/programs/EnvironmentMap_plot.png', dpi=150)
plt.close()
print("Done")