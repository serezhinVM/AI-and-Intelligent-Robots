# Lab 5 - PID Controller for Robot Navigation

Replace open-loop control with PID controllers for each stage of robot navigation from home through two gaps to the charger.

## Package: tutorial_pkg

### Structure

```
Lab 5/
+-- Lab 5.pdf
+-- README.md
+-- plot_heading.py
+-- plot_trajectory.py
+-- plot_velocity.py
+-- plot_laser_map.py
+-- heading.png (generated)
+-- robot_trajectory.png (generated)
+-- robot_velocity.png (generated)
+-- laser_map.png (generated)
L-- tutorial_pkg/
    +-- CMakeLists.txt
    +-- package.xml
    +-- launch/
    |   L-- tutorial_turtlebot.launch.py
    +-- src/
    |   L-- tutorial_pkg_node.cpp
    +-- robotTrajData.csv
    +-- robotVelData.csv
    +-- robotHeadData.csv
    +-- laserData.csv
    L-- laserMapData.csv
```

### Build

```bash
cd ~/M-Drive/ros2_ws
colcon build
source install/setup.sh
export TURTLEBOT3_MODEL=burger
```

### Run

```bash
ros2 launch tutorial_pkg tutorial_turtlebot.launch.py
```

### PID Controllers

| Stage | Controller | Target | PID Parameters |
|-------|-----------|--------|----------------|
| 1 | `PID_wallFollowing` | leftRange → 0.3m | kp=0.5, ki=0.01, kd=0.01 |
| 2 | `PID_pass1stGap` | heading → 0 rad | kp=2.0, ki=0.05, kd=0.01 |
| 3 | `PID_moveto2ndGap` | heading → 0 rad | kp=2.0, ki=0.05, kd=0.01 |
| 4 | `PID_pass2ndGap` | heading → -1.57 rad | kp=2.0, ki=0.05, kd=0.01 |
| 5 | `PID_reachCharger` | heading → 3.14 rad | kp=2.0, ki=0.05, kd=0.01 |

### Output Files

- `robotTrajData.csv` - Robot position (X, Y) trajectory
- `robotVelData.csv` - Linear and angular velocity per timestep
- `robotHeadData.csv` - Robot heading angle per timestep
- `laserData.csv` - Laser range readings at 5 angles
- `laserMapData.csv` - Cartesian coordinates of laser points in global frame

### Plotting

```bash
python3 plot_trajectory.py
python3 plot_velocity.py
python3 plot_heading.py
python3 plot_laser_map.py
```
