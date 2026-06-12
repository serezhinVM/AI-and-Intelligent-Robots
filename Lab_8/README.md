# Lab 8 — Trapezoidal centroid fuzzy controllers

Extends Lab 7 with **complete centroid defuzzification**: trapezoidal membership
functions (plateaus for near/far, wide medium overlap) replace crisp band
thresholds, and the min-max rule output is defuzzified via **weighted-average
centroid**. The continuous control eliminates Lab 7's bang-bang jumps —
~100 % reliability.

## What is different from Lab 7

| Aspect | Lab 7 (crisp fuzzy) | Lab 8 (centroid fuzzy) |
|--------|---------------------|------------------------|
| Membership functions | Crisp bands (0/1) | Trapezoidal, continuous [0,1] |
| Defuzzification | Predefined (speed,turn) per rule | Weighted-average centroid |
| Speed in stages 3-5 | frontRange < 0.5 ? 0.1 : 0.3 | centroidSpeed(frontRange) capped at MID |
| Velocity log | 3 cols (cycle, lin.x, ang.z) | Same (kept from Lab 7) |
| Heading bearings | -0.70 / -1.50 rad | -0.75 / -1.55 rad |
| Reliability | ~80 % | ~100 % |

## Stages

| Stage | Behaviour | Method |
|-------|-----------|--------|
| 1 | Wall following | 2-input centroid fuzzy (leftRange, mleftRange) |
| 2 | Turn into Gap 1 | 2-input centroid fuzzy (leftRange, mleftRange) |
| 3 | Descent to Gap 2 | Heading-hold P-controller + centroid speed |
| 4 | Through Gap 2 | Heading-hold P-controller + centroid speed |
| 5 | To charger | Heading-hold P-controller + centroid speed |
| 6 | Stop | — |

## Centroid fuzzy (stages 1-2)

**Membership functions** (trapezoidal):
- **near**: full <= 0.15 m -> linear -> 0 at 0.40 m
- **medium**: linear up 0.20->0.35, full 0.35-0.60, linear down 0.60->0.80
- **far**: linear up 0.50->0.80, full >= 0.80 m

**Rule application** — each of the 9 {speedIdx, turnIdx} rules fires with
weight = fmin(mu1, mu2), aggregated by fmax into speed and turn output sets.

**Defuzzification** — weighted average of singleton outputs:

crispSpeed = (w_low * 0.10 + w_mid * 0.25 + w_high * 0.45) / (w_low + w_mid + w_high)
crispTurn  = (w_left * 0.35 + w_none * 0.00 + w_right * -0.55) / (w_left + w_none + w_right)

## Structure

Lab_8/
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

## Build and run

cd ~/M-Drive/ros2_ws
colcon build --packages-select tutorial_pkg
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tutorial_pkg tutorial_turtlebot.launch.py

## Plots

| File | Content |
|------|---------|
| data/robot_trajectory.png | Robot XY path |
| data/robot_velocity.png | Linear + angular velocity |
| data/robot_heading.png | Heading over time |
| data/laser_map.png | Laser environment map + trajectory |

### Regenerate

python3 plot_trajectory.py
python3 plot_velocity.py
python3 plot_heading.py
python3 plot_laser_map.py

## Implementation notes

- The trapezoidal membership functions give a plateau of full activation for
  near (<= 0.15) and far (>= 0.80), making the robot more decisive in extreme
  situations while still smooth in transitions.
- centroidSpeed(front) replaces the crisp frontRange < 0.5 ? 0.1 : 0.3
  of Lab 7 — the robot now slows down gradually as obstacles approach.
- The heading-hold P-controller (stages 3-5) is kept for reliability through
  the tight diagonal Gap 2 (0.33 m).
- Angular velocity is logged alongside linear velocity in robotVelData.csv
  (3 columns: cycle, linear_x, angular_z).
