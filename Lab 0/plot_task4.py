import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon

# Читаем данные построчно
with open('/home/vladimir/Documents/programs/Trajectories-Map', 'r') as f:
    lines = f.readlines()

# Разбиваем на секции
sections = {}
current_section = None
for line in lines:
    line = line.strip()
    if not line or line.startswith('#'):
        current_section = line.replace('#', '').strip()
        sections[current_section] = []
    elif current_section:
        parts = line.split(',')
        if len(parts) == 2:
            try:
                sections[current_section].append([float(parts[0]), float(parts[1])])
            except:
                pass

fig, ax = plt.subplots(figsize=(12, 12))

# Стены
if 'Walls' in sections:
    walls = np.array(sections['Walls'])
    ax.scatter(walls[:,0], walls[:,1], s=2, c='gray', label='Walls')

# Колонны
if 'Pillars' in sections:
    pillars = np.array(sections['Pillars'])
    points_per_pillar = 18
    for p in range(4):
        start = p * points_per_pillar
        pillar = pillars[start:start + points_per_pillar]
        cx, cy = np.mean(pillar[:,0]), np.mean(pillar[:,1])
        angles = np.arctan2(pillar[:,1] - cy, pillar[:,0] - cx)
        sorted_idx = np.argsort(angles)
        pillar_sorted = pillar[sorted_idx]
        ax.add_patch(Polygon(pillar_sorted, closed=True, fill=True, facecolor='blue', alpha=0.3, edgecolor='blue'))
        ax.scatter(pillar[:,0], pillar[:,1], s=5, c='blue')

# Зарядка
if 'Charger' in sections:
    charger = np.array(sections['Charger'])
    cx, cy = np.mean(charger[:,0]), np.mean(charger[:,1])
    angles = np.arctan2(charger[:,1] - cy, charger[:,0] - cx)
    sorted_idx = np.argsort(angles)
    charger_sorted = charger[sorted_idx]
    ax.add_patch(Polygon(charger_sorted, closed=True, fill=True, facecolor='yellow', alpha=0.5, edgecolor='orange'))
    ax.scatter(charger[:,0], charger[:,1], s=5, c='orange', label='Charger')

# Робот (рисуем последним, поверх всего)
if 'Robot' in sections:
    robot = np.array(sections['Robot'])
    ax.scatter(robot[:,0], robot[:,1], s=50, marker='s', c='red', label='Robot', zorder=10)

# Траектории
if 'Trajectory 1: Vl=10, Vr=8' in sections:
    traj1 = np.array(sections['Trajectory 1: Vl=10, Vr=8'])
    ax.plot(traj1[:,0], traj1[:,1], 'b-', linewidth=2, label='Traj 1: Vl=10, Vr=8')

if 'Trajectory 2: Vl=5, Vr=7' in sections:
    traj2 = np.array(sections['Trajectory 2: Vl=5, Vr=7'])
    ax.plot(traj2[:,0], traj2[:,1], 'r-', linewidth=2, label='Traj 2: Vl=5, Vr=7')

ax.set_title('Task A+B: Trajectories and Environment Map')
ax.set_xlabel('X (cm)')
ax.set_ylabel('Y (cm)')
ax.grid(True)
ax.set_aspect('equal')
ax.legend(loc='upper right', markerscale=2)

fig.savefig('/home/vladimir/Documents/programs/Trajectories-Map_plot.png', dpi=150)
plt.close()
print("Done")