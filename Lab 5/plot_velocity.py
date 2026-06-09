import matplotlib.pyplot as plt

timestep, linear_vel, angular_vel = [], [], []
with open('/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robotVelData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split()
            timestep.append(float(parts[0]))
            linear_vel.append(float(parts[1]))
            angular_vel.append(float(parts[2]))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(timestep, linear_vel, 'r-', linewidth=1.5)
ax1.set_ylabel('Linear velocity (m/s)')
ax1.set_title('Robot Velocities')
ax1.grid(True)

ax2.plot(timestep, angular_vel, 'b-', linewidth=1.5)
ax2.set_xlabel('Timestep')
ax2.set_ylabel('Angular velocity (rad/s)')
ax2.grid(True)

plt.tight_layout()
plt.savefig('/home/vladimir/M-Drive/ros2_ws/src/tutorial_pkg/robot_velocity.png', dpi=150)
print('Velocity plot saved to robot_velocity.png')
