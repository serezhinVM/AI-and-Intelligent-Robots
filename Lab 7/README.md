# Lab 7 — Fuzzy controllers

Same task as Labs 5/6 (home → Gap 1 → Gap 2 → charger), but the PID controllers are replaced by **fuzzy IF‑THEN controllers**: each laser input is sorted into a crisp band (near/medium/far) and a rule table picks a `(speed, turn)` action.

Stages 1–2 use the lab notes' **9‑rule crisp fuzzy tables** (fuzzify `leftRange` / `mleftRange` → speed + turn). Stages 3–5 descend along a **fixed bearing** (heading‑hold P‑controller, like the PID) with crisp speed, because a purely reactive fuzzy turn is **chaotic** at the tight 0.33 m diagonal Gap 2. The dense laser map of Lab 6 is kept; the robot stops on the charger (~0.03 m).

## Result

**Start** (0.30, 0.30) → wall-follow (fuzzy) → smooth turn through **Gap 1** (fuzzy) → heading‑hold through **Gap 2** → onto **charger**.

The CSVs and PNGs in `data/` are from a successful run.

## Structure

```
Lab 7/
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
    |   +-- tutorial_turtlebot.launch.py
    |   L-- tutorial_pkg_node.launch.py
    +-- worlds/
    |   L-- rosbot_sim_room_12.world
    +-- src/
        L-- tutorial_pkg_node.cpp
```

## Fuzzy controllers

### Stage 1 — Wall following

Fuzzify `leftRange` (laser at +90°) and `mleftRange` (+45°):

| leftRange | near (< 0.3) | medium (< 0.5) | far (≥ 0.5) |
|-----------|-------------|----------------|-------------|
| **near** (< 0.4) | 0.1 m/s, −0.3 rad/s | 0.1 m/s, 0.0 rad/s | 0.1 m/s, +0.3 rad/s |
| **medium** (< 0.6) | 0.3 m/s, −0.3 rad/s | 0.5 m/s, 0.0 rad/s | 0.3 m/s, +0.3 rad/s |
| **far** (≥ 0.6) | 0.3 m/s, −0.6 rad/s | 0.3 m/s, 0.0 rad/s | 0.5 m/s, +0.3 rad/s |

Exit when `PositionY < 1.30 && PositionX < 0.60`.

### Stage 2 — Turn into Gap 1

Fuzzify `leftRange` and `mleftRange` with wider thresholds:

| leftRange | near (< 0.4) | medium (< 0.6) | far (≥ 0.6) |
|-----------|-------------|----------------|-------------|
| **near** (< 0.5) | 0.1 m/s, −0.3 rad/s | 0.1 m/s, −0.3 rad/s | 0.1 m/s, −0.3 rad/s |
| **medium** (< 0.8) | 0.3 m/s, −0.6 rad/s | 0.1 m/s, 0.0 rad/s | 0.3 m/s, −0.6 rad/s |
| **far** (≥ 0.8) | 0.3 m/s, −0.6 rad/s | 0.1 m/s, −0.3 rad/s | 0.3 m/s, 0.0 rad/s |

Exit when `PositionX < 0.95`.

### Stages 3–5 — Heading hold (P‑controller)

```
error = targetHeading - robotHeadAngle    (normalised to [-π, π])
turn  = clamp(1.2 × error, ±0.8)
```

| Stage | Target heading | Exit condition |
|-------|---------------|----------------|
| 3 | −0.70 rad | `PositionX < 1.55` |
| 4 | −0.70 rad | `PositionX < 1.95` |
| 5 | −1.50 rad | `PositionY > 0.28` |

Forward speed in stages 3–5: **0.1 m/s** when `frontRange < 0.5`, otherwise **0.3 m/s**.

### Stage 6 — Stop

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
| `data/robotTrajData.csv` | trajectory (XY scatter) |
| `data/robotVelData.csv` | linear + angular velocity |
| `data/robotHeadingData.csv` | heading over time |
| `data/laserMapData.csv` | dense laser map |
| `data/robot_trajectory.png` | Trajectory plot |
| `data/robot_velocity.png` | Velocity plot |
| `data/robot_heading.png` | Heading plot |
| `data/laser_map.png` | Laser map |

### Regenerate graphs

```bash
python3 plot_trajectory.py
python3 plot_velocity.py
python3 plot_heading.py
python3 plot_laser_map.py
```

## Implementation notes

- Crisp thresholds make the output jump at each band boundary (bang‑bang).
  The crisp wall‑following occasionally diverges → **~80 % reliability**.
  **Lab 8's centroid defuzzification** smooths the output and reaches **100 %**.
- Navigation switch runs in `odomCallback` (30 Hz); laser data is collected
  in `scanCallback` (~5 Hz).
- Dense laser map (every 4th ray, `MAP_STEP=4`) — same as Lab 6.
- Launch file spawns the robot at (0.3, 0.3, Y=1.5708) and starts the node
  after a 5‑second delay so odometry is ready.
