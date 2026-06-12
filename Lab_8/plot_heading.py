import matplotlib.pyplot as plt

timestep, heading = [], []
with open('data/robotHeadingData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split()
            timestep.append(float(parts[0]))
            heading.append(float(parts[1]))

plt.figure(figsize=(10, 6))
plt.plot(timestep, heading, 'g-', linewidth=1.5)
plt.xlabel('Timestep')
plt.ylabel('Heading (rad)')
plt.title('Robot Heading over Time')
plt.grid(True)
plt.savefig('data/robot_heading.png', dpi=150)
print('Heading plot saved')
