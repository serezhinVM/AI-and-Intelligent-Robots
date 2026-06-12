import matplotlib.pyplot as plt

x, y = [], []
with open('data/robotTrajData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split()
            x.append(float(parts[0]))
            y.append(float(parts[1]))

# find plateau: truncate where last 100 points are within 0.001 of each other
last = len(x) - 1
while last > 100 and abs(x[last] - x[last-100]) < 0.001 and abs(y[last] - y[last-100]) < 0.001:
    last -= 1
last = min(last + 50, len(x))

plt.figure(figsize=(10, 8))
plt.plot(x[:last], y[:last], 'b-', linewidth=2)
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Robot Trajectory')
plt.grid(True)
plt.axis('equal')
plt.savefig('data/robot_trajectory.png', dpi=150)
print(f'Trajectory plot saved ({last} of {len(x)} points)')
