# Lab 1 — ROS2 Robot Operating System (Part A)

ROS2 Humble package for controlling a TurtleBot3 robot. Creates a publisher node that sends velocity commands to the robot.

## Package: tutorial_pkg

### Structure

```
tutorial_pkg/
+-- CMakeLists.txt
+-- package.xml
+-- launch/
¦   L-- tutorial_pkg_node.launch.py
L-- src/
    L-- tutorial_pkg/
        L-- tutorial_pkg_node.cpp
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
# Using ros2 run
ros2 run tutorial_pkg tutorial_pkg_node

# Or using launch file
ros2 launch tutorial_pkg tutorial_pkg_node.launch.py
```

### Node: Stopper

Publishes to /cmd_vel (geometry_msgs/Twist) to drive the robot forward at 0.1 m/s. Designed for use with TurtleBot3 in Gazebo simulation.
