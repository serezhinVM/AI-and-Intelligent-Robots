# Lab 5 — PID Controller for Robot Navigation

Builds on **Lab 4** (laser + heading + map). The open-loop bang-bang moves of
Lab 3 are replaced by **PID controllers**, giving a smooth trajectory from home,
through both gaps, to the charger.

## Result

**Start** (0.30, 0.30) → **wall-follow** up the left wall → smooth curve through
**Gap 1** → **Gap 2** → **charger**. Stops at **(2.18, 0.27)**, heading **−1.46 rad**.
All pillars cleared. Trajectory is smooth compared to the Lab 3 bang-bang path.

The CSVs and PNGs in `data/` are from the final run.

## The PID controller

```
err = setValue - measuredValue
ei  = ei_pre + err ;  ed = err - ed_pre
output = kp*err + ki*ei + kd*ed     (clamped to ±Max_output)
```

`PID_para` is passed **by value** (no persistent I/D state between calls), so
the controller acts as a strong **P** regulator with effective gain
kp + ki + kd = 2.06.

The navigation switch runs in `odomCallback` (30 Hz) for crisp control; laser
data is collected in `scanCallback` (~5 Hz).

Five PID behaviours, one per stage:

| Stage | Behaviour | Setpoint | gains (kp, ki, kd, max) |
|-------|-----------|----------|--------------------------|
| 1 | wall following | `leftRange` → 0.30 m | 0.5, 0.01, 0.01, 0.6 |
| 2 | heading → Gap 1 | yaw → 0 rad (+X) | 2.0, 0.05, 0.01, 0.6 |
| 3 | heading → Gap 2 | yaw → −0.80 rad | 2.0, 0.05, 0.01, 0.6 |
| 4 | through Gap 2 | yaw → −0.80 rad | 2.0, 0.05, 0.01, 0.6 |
| 5 | → charger | yaw → −1.45 rad | 2.0, 0.05, 0.01, 0.6 |
| 6 | stop | — | — |

Forward speed is constant `MIDDLE` = 0.3 m/s; the PID sets the turn rate.
Wall-following output is negated so the robot turns *toward* the wall.

Odom landmarks (tuned for the Burger in the TurtleBot3 House world):

| Landmark | Value | Transition |
|----------|-------|------------|
| `odom_landmark1` (Y) | 1.38 | stage 1 → 2 |
| `odom_landmark2` (X) | 0.95 | stage 2 → 3 |
| `odom_landmark3` (X) | 1.55 | stage 3 → 4 |
| `odom_landmark4` (X) | 1.95 | stage 4 → 5 |
| `odom_landmark5` (Y) | 0.33 | stage 5 → 6 |

## Build & run

```bash
cd ~/M-Drive/ros2_ws
colcon build --packages-select tutorial_pkg
source install/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 launch tutorial_pkg tutorial_turtlebot.launch.py
```

## Outputs

| File | Graph |
|------|-------|
| `data/robotTrajData.csv` (`x y`) | **trajectory** (XY scatter) |
| `data/robotVelData.csv` (`cycle linear_x angular_z`) | **velocity** (linear + angular) |
| `data/robotHeadData.csv` (`cycle yaw`) | **heading** |
| `data/laserMapData.csv` (`x, y`) | **laser map** (overlay on trajectory) |
| `data/laserData.csv` | raw 5 ranges (optional) |
| `data/robot_trajectory.png` | Trajectory plot |
| `data/robot_velocity.png` | Velocity plot |
| `data/robot_heading.png` | Heading plot |
| `data/laser_map.png` | Laser map plot |

### Regenerate graphs

```bash
python3 plot_trajectory.py
python3 plot_velocity.py
python3 plot_heading.py
python3 plot_laser_map.py
```

## Implementation Notes

- `PID_control` uses **pass-by-value** (no persistent I/D state), acting as a strong P controller.
- Navigation switch moved to `odomCallback` (30 Hz) for crisp control.
- Wall-following uses negated PID output so robot steers toward the wall when too far.
- All headings and landmarks tuned in Gazebo simulation.
