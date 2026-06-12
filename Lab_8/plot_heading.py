import matplotlib.pyplot as plt

timestep, heading = [], []
with open('data/robotHeadingData.csv') as f:
    for line in f:
        line = line.strip()
        if line:
            parts = line.split()
            timestep.append(float(parts[0]))
            heading.append(float(parts[1]))

# truncate at plateau (heading stable for 100 steps)
last = len(heading) - 1
while last > 100 and abs(heading[last] - heading[last-100]) < 0.001:
    last -= 1
last = min(last + 50, len(heading))

plt.figure(figsize=(10, 6))
plt.plot(timestep[:last], heading[:last], 'g-', linewidth=1.5)
plt.xlabel('Timestep')
plt.ylabel('Heading (rad)')
plt.title(f'Robot Heading over Time ({last} of {len(heading)} cycles)')
plt.grid(True)
plt.savefig('data/robot_heading.png', dpi=150)
print(f'Heading plot saved ({last} of {len(heading)} points)')
