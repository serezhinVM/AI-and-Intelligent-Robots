import matplotlib.pyplot as plt

x_map, y_map = [], []
with open('/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/laserMapData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split(',')
            x_map.append(float(parts[0]))
            y_map.append(float(parts[1]))

x_rob, y_rob = [], []
with open('/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotTrajData.csv') as f:
    for line in f:
        parts = line.strip().split()
        if len(parts) >= 2:
            x_rob.append(float(parts[0]))
            y_rob.append(float(parts[1]))

plt.figure(figsize=(10, 8))
plt.scatter(x_map, y_map, s=1, c='red', label='Laser points')
plt.plot(x_rob, y_rob, 'b-', linewidth=2, label='Robot trajectory')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.title('Laser-Based Environment Map')
plt.grid(True)
plt.axis('equal')
plt.legend()
plt.savefig('/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/laser_map.png', dpi=150)
print('Map saved to laser_map.png')
