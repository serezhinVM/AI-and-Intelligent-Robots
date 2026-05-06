import matplotlib.pyplot as plt
import numpy as np

traj1 = np.loadtxt('/home/vladimir/Documents/programs/trajectory_both', delimiter=',', max_rows=101)
traj2 = np.loadtxt('/home/vladimir/Documents/programs/trajectory_both', delimiter=',', skiprows=102)

plt.figure(figsize=(8, 8))
plt.scatter(traj1[:,0], traj1[:,1], s=5, c='blue', label='Traj 1: Vl=10, Vr=8')
plt.scatter(traj2[:,0], traj2[:,1], s=5, c='red', label='Traj 2: Vl=5, Vr=7')
plt.title('Task A-2: Two Trajectories')
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('/home/vladimir/Documents/programs/trajectory_both_plot.png', dpi=150)
plt.close()
print("Done")