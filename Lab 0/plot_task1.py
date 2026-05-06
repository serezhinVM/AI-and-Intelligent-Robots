import matplotlib.pyplot as plt
import numpy as np

x, y = np.loadtxt('/home/vladimir/Documents/programs/trajectory1', delimiter=',', unpack=True)

plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=5, c='blue')
plt.title('Task A-1: Robot Trajectory (Vl=10, Vr=8)')
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.grid(True)
plt.axis('equal')
plt.savefig('/home/vladimir/Documents/programs/trajectory1_plot.png', dpi=150)
plt.close()
print("Done")