import matplotlib.pyplot as plt

x, y = [], []
with open('data/robotTrajData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split()
            x.append(float(parts[0]))
            y.append(float(parts[1]))

plt.figure(figsize=(10, 8))
plt.plot(x, y, 'b-', linewidth=2)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.axis('equal')
plt.savefig('data/robot_trajectory.png', dpi=150)
print('Trajectory plot saved')
