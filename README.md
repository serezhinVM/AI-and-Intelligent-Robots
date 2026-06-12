# AI and Intelligent Robots

Laboratory works for the AI and Intelligent Robots course at Xi'an Jiaotong University.

**Technologies:** ROS 2 Humble · Gazebo · TurtleBot3 · C++ · Python (matplotlib) · CMake

---

## Repository Structure

```
Lab 0/   — Odometry and Environment Mapping    (standalone C++, no ROS)
Lab 1/   — ROS 2 Robot Operating System Part A
Lab 2/   — ROS 2 Robot Operating System Part B
Lab 3/   — Odometry Based Navigation for TurtleBot3
Lab 4/   — Laser Scanner
Lab 5/   — PID Controller for Robot Navigation
Lab 6/   — PID + Improved Laser Map + Code Optimisation
Lab 7/   — Fuzzy Controllers
Lab_8/   — Trapezoidal Centroid Fuzzy Controllers
```

---

## Lab 0 — Odometry and Environment Mapping

**Language:** Standalone C++ (no ROS), Python visualization

Implements wheel-encoder odometry (dead-reckoning) and builds an occupancy-style environment map from sensor data. Results are plotted with Python scripts (trajectory, environment map, animation).

**Key files:** `1_odometry.cpp`, `3_environment_map.cpp`, `plot_task*.py`, `animation.py`

---

## Lab 1 — ROS 2 Part A

**Language:** ROS 2 Humble, C++, TurtleBot3, Gazebo

First ROS 2 publisher node (`Stopper`) that publishes `geometry_msgs/Twist` to `/cmd_vel` to drive TurtleBot3 forward at 0.1 m/s in simulation.

**Key files:** `tutorial_pkg/src/tutorial_pkg_node.cpp`, `tutorial_pkg/launch/`

---

## Lab 2 — ROS 2 Part B

**Language:** ROS 2 Humble, C++, TurtleBot3, Gazebo

Drives the robot in a custom Gazebo world (`rosbot_sim_room_12.world`). Two tasks: vertical movement (yaw = 0.00) and horizontal movement (yaw = 1.57).

**Key files:** `tutorial_pkg/src/tutorial_pkg_node.cpp`, `worlds/rosbot_sim_room_12.world`

---

## Lab 3 — Odometry Based Navigation

**Language:** ROS 2 Humble, C++, Python plotting

First navigation task: robot moves from start (0.3, 0.3) to a charging station through a room with pillars. Uses **open-loop bang-bang control** in 6 stages (landmark thresholds). Generates trajectory and velocity plots.

**Key files:** `tutorial_pkg_node.cpp`, `data/robotTrajData.csv`, `plot_trajectory.py`

---

## Lab 4 — Laser Scanner

**Language:** ROS 2 Humble, C++, Python plotting

Collects LaserScan data at 5 angles (left, mid-left, front, mid-right, right) and builds a Cartesian environment map from polar coordinates using odometry pose.

**Key files:** `tutorial_pkg_node.cpp`, `plot_laser_map.py`, `data/laserMapData.csv`

---

## Lab 5 — PID Controller

**Language:** ROS 2 Humble, C++, Python plotting

Replaces Lab 3's bang-bang control with a **PID controller** (P-dominant). 5 navigation stages: wall-follow → heading to Gap 1 → heading to Gap 2 → through Gap 2 → to charger. Data logged: trajectory, velocity, heading, laser map.

**Key files:** `tutorial_pkg_node.cpp`, `data/`, `plot_*.py`

---

## Lab 6 — PID + Improved Map + Optimisation

**Language:** ROS 2 Humble, C++, Python plotting

Three parts: (1) PID navigation re-used from Lab 5, (2) dense environment map built from every 4th laser ray (~90 rays/scan, ~30 000 points), (3) code cleanup — named constants, `transformMapPoint()` helper, removed dead code.

**Key files:** `tutorial_pkg_node.cpp`, `data/laserMapData.csv`, `plot_laser_map.py`

---

## Lab 7 — Fuzzy Controllers

**Language:** ROS 2 Humble, C++, Python plotting

Replaces PID with **fuzzy IF-THEN controllers** using crisp-band membership functions and 9-rule tables. Stage 1 wall-following uses fuzzy; stages 3–5 use a P-controller for heading hold. Reliability ~80 % due to bang-bang behaviour at band boundaries.

**Key files:** `tutorial_pkg_node.cpp`, `data/`, `plot_*.py`

---

## Lab 8 — Trapezoidal Centroid Fuzzy Controllers

**Language:** ROS 2 Humble, C++, Python plotting

Upgrades Lab 7 with **centroid defuzzification**. Key changes:
- **Trapezoidal membership functions** (near plateau ≤ 0.15 m, medium full 0.35–0.60 m, far ≥ 0.80 m)
- **Min-max rule aggregation** → **weighted-average centroid** for both speed and turn
- Continuous control eliminates bang-bang jumps. Reliability ~100 %.

**Key files:** `tutorial_pkg_node.cpp`, `data/`, `plot_*.py`

---

## Common Scenario (Labs 3–8)

All navigation labs (3–8) share the same task:

```
Start (0.3, 0.3, yaw = 1.57) → wall-follow → Gap 1 → Gap 2 → Charger (≈2.25, 0.27)
```

World file: `rosbot_sim_room_12.world`.
ROS 2 package: `tutorial_pkg`.
Launch: `ros2 launch tutorial_pkg tutorial_turtlebot.launch.py`.

---

## Plots and Data

Each lab from 3 onward generates CSV data and PNG plots in a `data/` subdirectory:

| File | Content |
|------|---------|
| `robotTrajData.csv` | X, Y, heading over time |
| `robotVelData.csv` | Linear.x, angular.z over time |
| `laserMapData.csv` | Global-frame laser map points |
| `robot_trajectory.png` | Robot path through the room |
| `robot_velocity.png` | Speed profile |
| `robot_heading.png` | Heading vs time |
| `laser_map.png` | Environment map |

---

## Getting Started

```bash
git clone https://github.com/serezhinVM/AI-and-Intelligent-Robots.git
cd AI-and-Intelligent-Robots/Lab_8/tutorial_pkg
colcon build --packages-select tutorial_pkg
ros2 launch tutorial_pkg tutorial_turtlebot.launch.py
```

Requires ROS 2 Humble, Gazebo, and TurtleBot3 packages installed.

---

## Licence

Educational project — no licence specified.
