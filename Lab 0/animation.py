import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.animation import FuncAnimation
import matplotlib.animation as animation

# Читаем данные
with open('/home/vladimir/Documents/programs/Trajectories-Map', 'r') as f:
    lines = f.readlines()

sections = {}
current_section = None
for line in lines:
    line = line.strip()
    if not line or line.startswith('#'):
        current_section = line.replace('#', '').strip()
        sections[current_section] = []
    elif current_section and current_section != '':
        parts = line.split(',')
        if len(parts) == 2:
            try:
                sections[current_section].append([float(parts[0]), float(parts[1])])
            except:
                pass

# Данные траекторий
traj1 = np.array(sections['Trajectory 1: Vl=10, Vr=8'])
traj2 = np.array(sections['Trajectory 2: Vl=5, Vr=7'])

# Стены
walls = np.array(sections['Walls'])

# Колонны
pillars = np.array(sections['Pillars'])
points_per_pillar = 18

# Зарядка
charger = np.array(sections['Charger'])

# Робот
robot = np.array(sections['Robot'])

# Начальная позиция робота
robot_start = traj1[0]

# Создаем фигуру
fig, ax = plt.subplots(figsize=(12, 12))

# Функция обновления кадра
def update(frame):
    ax.clear()
    
    # Стены
    ax.scatter(walls[:,0], walls[:,1], s=2, c='blue', label='Walls')
    
    # Колонны
    for p in range(4):
        pillar = pillars[p * points_per_pillar:(p + 1) * points_per_pillar]
        cx, cy = np.mean(pillar[:,0]), np.mean(pillar[:,1])
        angles = np.arctan2(pillar[:,1] - cy, pillar[:,0] - cx)
        sorted_idx = np.argsort(angles)
        pillar_sorted = pillar[sorted_idx]
        ax.add_patch(Polygon(pillar_sorted, closed=True, fill=True, facecolor='blue', alpha=0.3, edgecolor='blue'))
        ax.scatter(pillar[:,0], pillar[:,1], s=5, c='blue')
    
    # Зарядка
    cx, cy = np.mean(charger[:,0]), np.mean(charger[:,1])
    angles = np.arctan2(charger[:,1] - cy, charger[:,0] - cx)
    sorted_idx = np.argsort(angles)
    charger_sorted = charger[sorted_idx]
    ax.add_patch(Polygon(charger_sorted, closed=True, fill=True, facecolor='yellow', alpha=0.5, edgecolor='orange'))
    ax.scatter(charger[:,0], charger[:,1], s=5, c='yellow', label='Charger')
    
    # Траектории (полностью - всегда видимы)
    ax.plot(traj1[:,0], traj1[:,1], 'b-', linewidth=1, alpha=0.5, label='Traj 1')
    ax.plot(traj2[:,0], traj2[:,1], 'r-', linewidth=1, alpha=0.5, label='Traj 2')
    
    # Анимация: frame < 101 - красная траектория, frame >= 101 - синяя
    if frame < len(traj1):
        # Красная траектория - показываем пройденный путь ярче
        current_traj = traj2[:frame+1]
        ax.plot(current_traj[:,0], current_traj[:,1], 'r-', linewidth=3, alpha=1.0, label='Traj 2: Vl=5, Vr=7')
        # Робот
        ax.scatter(traj2[frame,0], traj2[frame,1], s=150, marker='s', c='red', edgecolor='black', linewidth=2, zorder=10, label='Robot')
        plt.title('Robot Animation - Red Trajectory (Vl=5, Vr=7)')
    else:
        # Синяя траектория - показываем пройденный путь ярче
        frame_blue = frame - len(traj1)
        current_traj = traj1[:frame_blue+1]
        ax.plot(current_traj[:,0], current_traj[:,1], 'b-', linewidth=3, alpha=1.0, label='Traj 1: Vl=10, Vr=8')
        ax.scatter(traj1[frame_blue,0], traj1[frame_blue,1], s=150, marker='s', c='red', edgecolor='black', linewidth=2, zorder=10, label='Robot')
        plt.title('Robot Animation - Blue Trajectory (Vl=10, Vr=8)')
    
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.grid(True)
    ax.set_aspect('equal')
    ax.legend(loc='upper right', markerscale=1)
    
    # Установка границ
    ax.set_xlim(-10, 270)
    ax.set_ylim(-10, 270)
    
    return []

# Создаем анимацию
total_frames = len(traj1) * 2
ani = FuncAnimation(fig, update, frames=total_frames, interval=50, blit=True)

# Сохраняем как GIF
ani.save('/home/vladimir/Documents/programs/robot_animation.gif', writer='pillow', fps=20, dpi=100)
plt.close()
print("GIF saved!")