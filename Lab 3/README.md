# Lab 3 - Odometry Based Navigation for TurtleBot3

ROS2 package for odometry-based navigation of a TurtleBot3 robot in Gazebo simulation. The robot navigates through a room with pillars to reach a charging station.

## Package: tutorial_pkg

### Structure

```
Lab 3/
+-- Lab 3.pdf
+-- README.md
+-- robot_trajectory.png
+-- robot_velocity.png
L-- tutorial_pkg/
    +-- CMakeLists.txt
    +-- package.xml
    +-- launch/
    В¦   L-- tutorial_turtlebot.launch.py
    +-- src/
    В¦   L-- tutorial_pkg_node.cpp
    +-- robotTrajData.csv
    L-- robotVelData.csv
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

### Node: Stopper

The robot navigates through 6 stages:
1. Move forward until Y < landmark (1.38)
2. Move forward-right until X < landmark (0.80)
3. Move forward (high speed) until X < landmark (1.30)
4. Move forward-right until X < landmark (1.72)
5. Move forward (low speed) until Y < landmark (0.30)
6. Stop

### Output Files

- `robotTrajData.csv` - Robot position (X, Y) at each odometry callback
- `robotVelData.csv` - Cycle number, linear velocity (m/s), angular velocity (rad/s)
- `robot_trajectory.png` - Plot of robot trajectory
- `robot_velocity.png` - Plot of linear and angular velocity over time
