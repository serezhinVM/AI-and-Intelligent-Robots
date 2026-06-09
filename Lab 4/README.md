# Lab 4 - Laser Scanner

ROS2 package for laser-based environment mapping. The robot uses LaserScan data to detect obstacles and build a map of the room with pillars.

## Package: tutorial_pkg

### Structure

```
Lab 4/
+-- Lab 4.pdf
+-- README.md
+-- plot_laser_map.py
+-- laser_map.png
L-- tutorial_pkg/
    +-- CMakeLists.txt
    +-- package.xml
    +-- launch/
    |   L-- tutorial_turtlebot.launch.py
    +-- src/
    |   L-- tutorial_pkg_node.cpp
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

### Tasks

#### Task 1 - Collect Laser Range Readings
Added LaserScan subscription to read range data at 5 angles:
- left (90°), mid-left (45°), front (0°), mid-right (315°), right (270°)
- Data saved to `laserData.csv`

#### Task 2 - Build Environment Map
Convert laser readings from polar coordinates (range, theta) in robot frame to cartesian coordinates (xg, yg) in global frame using robot head angle (Th) from odometry quaternion:
- `xg = Range * cos(Th + theta) + X`
- `yg = Range * sin(Th + theta) + Y`
- Data saved to `laserMapData.csv`
- Map visualization generated with `plot_laser_map.py`

### Output Files

- `laserData.csv` - Raw laser range readings (5 angles) with index
- `laserMapData.csv` - Cartesian coordinates of laser points in global frame
- `laser_map.png` - Environment map plot (laser points + robot trajectory)
