# Hackathon demonstration

## Current row demonstration

From the repository root:

```bash
cd ros2_ws
./run.sh row_mission:=true row_count:=3
```

For the entire configured field, use `row_count:=23`. Stop with Ctrl-C or
`./stop.sh` from this workspace. All commands in this document use the new
`ros2_ws` layout; commands containing `cd ros2_ws` start at the repository root.

Full-field completion and no-contact tractor/crop avoidance remain unverified.
The row driver uses Gazebo ground truth. The fixed tractor stop is not a detour
planner. The current collision-monitor sensor inputs are disabled, and the
RGB-D stop does not cover headland turns. These are current prototype limits.

The frontier/return sequence below is historical evidence from earlier tuning.
Select `row_mission:=false` for its controls; `demo.sh start/pause/return` do not
control the row driver. Revalidate it before presenting it as a current result.

## 1. Start Gazebo and RViz

Open Terminal 1 and paste:

```bash
cd ros2_ws
./stop.sh
./run.sh row_mission:=false explore:=false
```

Wait about 30 seconds. Gazebo shows the supplied cotton farm and rover in its
open margin. RViz shows the robot, point cloud and live map. This command enables
all twelve implementation stages, with frontier selection paused for presenting.
If another run is open, stop it with Ctrl-C first.

## 2. Run the tested short demonstration

In Terminal 2:

```bash
cd ros2_ws
./demo.sh
```

Watch the curved outbound drive and return. The terminal prints:

```text
Driving a short curved route with obstacle checks enabled.
Drive succeeded. Requesting Dijkstra-guided return.
PASS: curved drive and return
```

Then the light test switches day → night → day and the suspension test cycles
terrain → level → hybrid. Both print `pass: True` in their result records.
This exact sequence passed in a fresh run on 2026-09-09. The final return error
was **0.216 m** and **0.141 rad**, within the documented 0.30 m / 0.30 rad tolerances.
Run this sequence once per fresh simulation; restarting resets the recorded start.

## 3. Interactive controls

Run any of these from Terminal 2:

```bash
./demo.sh night
./demo.sh day
./demo.sh auto-light
./demo.sh level
./demo.sh terrain
./demo.sh hybrid
./demo.sh start
./demo.sh pause
./demo.sh return
```

`start` enables frontier exploration; `pause` cancels the current exploration
motion; `return` asks for a Dijkstra-guided return. A blocked route is reported,
not silently counted as visited. After a completed return, `start` resumes frontier selection with the original
recorded start pose retained.

For the fully automatic frontier launch from a fresh terminal:

```bash
cd ros2_ws
./run.sh row_mission:=false
```

Whole-field completion remains an integration goal. Use the short tested sequence
for a time-limited presentation rather than promising a fixed completion time.

## 4. Show engineering evidence

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=agri_ugv
export ROS_LOG_DIR="$PWD/log/checks"
python3 tools/check_structure.py
python3 -m pytest -q tests
ros2 topic list
ros2 topic echo /encoders --once
ros2 topic echo /suspension/debug --once
ros2 topic echo /exploration/status --once --qos-durability transient_local
```

Six encoder names are expected: L1/L2/L3 and R1/R2/R3. All eight wheel joints have
velocity commands, but rear wheel joints have no encoder state interfaces.
The current structure check reports 286.11 kg and positive inertia matrices.

Optional live evidence capture:

```bash
python3 tools/check_live.py --seconds 10 --output reports/live_demo.json
python3 tools/check_tf.py --root map --seconds 10 --output reports/tf_demo.json
python3 tools/check_mapping.py
```

## What to say to the judges

> We integrated an eight-wheel agricultural rover with six wheel encoders,
> camera, 3D LiDAR and IMU in ROS 2 Jazzy and Gazebo Harmonic. We have demonstrated
> multi-sensor mapping, a curved autonomous drive, Dijkstra-guided return,
> automatic headlights and bounded active suspension control. Our longer trial
> reached two exploration frontiers. Full-field completion and validation against
> the original CAD joint/material definitions are the remaining work.

Avoid claiming exact return, production-ready autonomy, measured hydraulic
pressure, physical LDR photometry, or complete field coverage. The robot mechanism
and material inertias remain provisional; the farm assets are the supplied ones.

## Evidence files

- `reports/presentation_demo.json`: successful outbound/return sequence.
- `reports/stage10_lighting.json`: applied spotlight states and image brightness.
- `reports/stage11_suspension.json`: three modes, finite stroke/force, bounded effort.
- `reports/structure.json`: mass, inertia and encoder/drive separation.
- `reports/stage8_margin.json`: curved Nav2 arrival and no-spin command check.
- `reports/stage8_dynamic.json`: actual moving-worker poses.
- `reports/live_camera.png`, `reports/slam_grid.png`: captured simulation outputs.
- `reports/light_1_raw.png`, `reports/light_1_gain.png`: spotlight and gain images.
- `run/mission_*.jsonl`: timestamped mission events; `log/`: build/runtime logs.

The full per-step command table and implementation limitations are in `README.md`.
