import matplotlib.pyplot as plt

x_map, y_map = [], []
with open('data/laserMapData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split(',')
            x_map.append(float(parts[0]))
            y_map.append(float(parts[1]))

x_rob, y_rob = [], []
with open('data/robotTrajData.csv') as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) >= 2:
            x_rob.append(float(parts[0]))
            y_rob.append(float(parts[1]))

# truncate robot trajectory at plateau
last = len(x_rob) - 1
while last > 100 and abs(x_rob[last] - x_rob[last-100]) < 0.001 and abs(y_rob[last] - y_rob[last-100]) < 0.001:
    last -= 1
last = min(last + 50, len(x_rob))

plt.figure(figsize=(10, 8))
plt.scatter(x_map, y_map, s=1, c='red', label='Laser points')
plt.plot(x_rob[:last], y_rob[:last], 'b-', linewidth=2, label='Robot trajectory')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Laser-Based Environment Map')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.savefig('data/laser_map.png', dpi=150)
print(f'Map saved ({last} of {len(x_rob)} trajectory points)')
