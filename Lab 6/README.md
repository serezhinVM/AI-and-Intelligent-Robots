# Lab 6 - PID + Improved Laser Map + Code Optimisation

Builds on **Lab 5**. Three parts:

- **6.1 - PID navigation (reused from Lab 5).** The PID controller passes both
  gaps without collision and reaches the charger robustly.
- **6.2 - Improved map.** The environment map is now built from **many laser
  readings** (every MAP_STEP-th ray of the full 360-deg scan, ~90 per scan)
  instead of just 5 -> a **dense** map (~30 000 points vs ~1 650).
- **6.3 - Optimised code.** Magic numbers replaced by named constants
  (LEFT_LASER_INDEX, MAP_STEP, heading_*, odom_landmark*);
  transformMapPoint helper introduced; unused code removed.

## Result

**Start** (0.30, 0.30) -> wall-follow -> Gap 1 -> Gap 2 -> charger.
Same PID trajectory as Lab 5. The improved map now shows the full room
outline and all four pillars clearly.

## Structure

`
Lab 6/
+-- README.md
+-- plot_trajectory.py
+-- plot_velocity.py
+-- plot_heading.py
+-- plot_laser_map.py
+-- data/
|   +-- robotTrajData.csv
|   +-- robotVelData.csv
|   +-- robotHeadingData.csv
|   +-- laserMapData.csv
|   +-- robot_trajectory.png
|   +-- robot_velocity.png
|   +-- robot_heading.png
|   +-- laser_map.png
L-- tutorial_pkg/
    +-- CMakeLists.txt
    +-- package.xml
    +-- launch/
    |   L-- tutorial_turtlebot.launch.py
    +-- worlds/
    |   L-- rosbot_sim_room_12.world
    +-- src/
        L-- tutorial_pkg_node.cpp
`

## Build & run

`ash
cd ~/M-Drive/ros2_ws
colcon build --packages-select tutorial_pkg
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tutorial_pkg tutorial_turtlebot.launch.py
`

## Outputs

| File | Graph |
|------|-------|
| data/robotTrajData.csv | trajectory (XY scatter) |
| data/robotVelData.csv | linear + angular velocity |
| data/robotHeadingData.csv | heading over time |
| data/laserMapData.csv | improved dense map |
| data/robot_trajectory.png | Trajectory plot |
| data/robot_velocity.png | Velocity plot |
| data/robot_heading.png | Heading plot |
| data/laser_map.png | Improved laser map |

### Regenerate graphs

`ash
python3 plot_trajectory.py
python3 plot_velocity.py
python3 plot_heading.py
python3 plot_laser_map.py
`

## Changes from Lab 5

- scanCallback now loops over all laser rays with step MAP_STEP=4
  -> dense map (every 4th ray, ~90 per scan).
- New transformMapPoint(laserRange, laserTh) converts a single laser
  reading to world coordinates using the robot's pose.
- Named constants: LEFT_LASER_INDEX=90, MAP_STEP=4, heading_gap1,
  heading_gap2, heading_charger, wall_target, odom_landmark*.
- Removed: 5-range log (laserData.csv), per-angle/per-range variables,
  unused includes, RCLCPP_INFO spam in odom callback.
- PID controller, landmarks, and headings unchanged from Lab 5.
