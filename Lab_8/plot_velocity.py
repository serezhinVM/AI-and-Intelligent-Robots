import matplotlib.pyplot as plt

timestep, linear_vel, angular_vel = [], [], []
with open('data/robotVelData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split()
            timestep.append(float(parts[0]))
            linear_vel.append(float(parts[1]))
            angular_vel.append(float(parts[2]))

# truncate at last meaningful movement
last = len(linear_vel) - 1
while last > 10 and abs(linear_vel[last]) < 0.001:
    last -= 1
last = min(last + 50, len(linear_vel))

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))

ax1.plot(timestep[:last], linear_vel[:last], 'r-', linewidth=1.5)
ax1.set_ylabel('Linear velocity (m/s)')
ax1.set_title(f'Robot Velocities ({last} of {len(linear_vel)} cycles)')
ax1.grid(True)

ax2.plot(timestep[:last], angular_vel[:last], 'b-', linewidth=1.5)
ax2.set_xlabel('Timestep')
ax2.set_ylabel('Angular velocity (rad/s)')
ax2.grid(True)

plt.tight_layout()
plt.savefig('data/robot_velocity.png', dpi=150)
print(f'Velocity plot saved ({last} of {len(linear_vel)} points)')
