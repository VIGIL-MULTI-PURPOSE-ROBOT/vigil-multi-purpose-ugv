# VIGIL Multi-Purpose UGV: engineering project briefing

This briefing describes the **VIGIL** robot, its three simulation environments, file locations,
operating instructions, implementation status and development conventions.

---

## 0. Owner's rules (always follow them)

1. **Three separate domains. Never mix them.** Agriculture, Rock Terrain and Military SAR each have
   their own workspace, world, robot model copy, controllers, launch files, parameters, dashboard
   and `ROS_DOMAIN_ID`. Do not create a combined workspace. Do not share or merge launch files,
   worlds, controllers or navigation code between domains. When a task names one domain, modify
   files **only** inside that domain's folder.
2. **Do not break existing functionality.** Extend the existing nodes. Do not rewrite them. Keep
   existing topics, parameters and behaviour unless the task says otherwise.
3. **No cheating in simulation.** Never teleport the robot, set its pose directly, or fake distance
   checks or collisions. Motion must come from ROS 2 commands → ros2_control / Gazebo physics.
4. **Back up before modifying.** Copy the original files into a dated backup folder in that
   workspace (for example `.autonomy_backup/2026-09-24/`, `.navfix_backup/2026-09-24/`).
5. **Test honestly.** Say clearly what was verified in Gazebo, what was only checked offline
   (Python tests without Gazebo), and what was not tested.
6. **Reports:** the owner usually asks for a fixed report format (for example "Root cause / Files
   changed / Fix / Test result / Remaining issues"). Use exactly the headings they give.
7. During long work the owner likes **progress updates as percentages**.

---

## 1. The robot: VIGIL

- An eight-wheel rover: twin rockers and independently sprung wheels. Wheel radii are 0.23463 m
  (front pair) and 0.17655 m (the others). Skid steering.
- Simulated mass is about 286 kg. **Note:** the mass, inertia and rocker pivots were inferred from
  an STL without joints, so the model is not an exact CAD twin.
- Sensors (they vary by domain): RGB camera (4K in SAR), depth camera, segmentation camera,
  thermal camera 320×240 (SAR), 3D LiDAR, IMU, wheel encoders.
- Each domain has **its own copy** of the robot description, tuned for that domain.

## 2. Platform

- Ubuntu 24.04, **ROS 2 Jazzy**, **Gazebo Harmonic** (DART physics), ros2_control
  (`gz_ros2_control`), Nav2, RTAB-Map, robot_localization. Python nodes use numpy and OpenCV.
- GitHub: `VIGIL-MULTI-PURPOSE-ROBOT/vigil-multi-purpose-ugv`. The repo README is
  `~/Documents/vigil/REPO_README.md`.

## 3. Folder map (the owner's computer, user `/home/user`)

```
~/Documents/vigil/                       launcher + publishing scripts (no robot code)
  vigil                                  the `vigil` menu command (installed by setup_vigil.sh)
  master_launcher/vigil_launcher.py      menu: 1 Military SAR, 2 Agriculture, 3 Rock Terrain, 4 Exit
  agriculture/ros2_ws  -> symlink to ~/Documents/robot/ros2_ws
  rock_terrain/vigil_rough_terrain_ws -> symlink to the rock-terrain workspace
  military_sar/        -> SAR (~/Documents/military_world and its sar_ws)
  publish_to_github.sh, REPO_README.md, README.md, setup_vigil.sh
```

The launcher sources only the chosen project's `install/setup.bash`. It stops leftovers from the
other projects and never builds.

| Domain | Workspace | Package | ROS_DOMAIN_ID | Dashboard |
|---|---|---|---|---|
| Military SAR | `~/Documents/military_world/sar_ws` | `vigil_sar` | 72 | http://localhost:8080 |
| Agriculture | `~/Documents/robot/ros2_ws` | `agri_ugv` (+ `agri_ugv_description`, `agri_ugv_setup`) | 91 (`GZ_PARTITION=agri_ugv`) | http://localhost:8080 |
| Rock terrain | `vigil_rough_terrain_ws` | `vigil_rough_terrain` | 71 | http://localhost:8080 |

Only one domain runs at a time (they share port 8080).

---

## 4. Domain 1: MILITARY SEARCH AND RESCUE (`~/Documents/military_world`)

**World**
- A 300 × 300 m disaster area built in Blender (`military_world.blend`), exported by
  `export_gazebo.py` to `gazebo_export/military_world.sdf`.
- It contains roads, buildings, rubble, vehicles, a river, a forest, about 20 walking people, and
  casualties inside and outside buildings.
- The walking people use the plugin `plugins/WaypointSystem.cc`, built with cmake into
  `military_world/build/`.
- Scale is 1 unit = 1 m. The operational zone is 170 × 170 m.
- `sar_*.json/.csv` hold the building footprints, people, routes and heat table.

**Rover workspace** `sar_ws/src/vigil_sar/`
- `config/sar_mission.yaml`: all mission, world, sensor and people parameters.
- `config/physics.yaml`: gravity, time step, acceleration and jerk limits, torque, mass.
- `launch/sar_mission.launch.py` is the all-in-one launch. There are also sim, navigation, sar and
  dashboard launches.
- The world is **rebuilt at every launch** by `scripts/build_sar_world.py` into `sar_ws/generated/`.
  It prunes collision shapes: about 3,100 → 955 kept, only those the rover can reach.
- Key scripts:
  - `cliff_navigator.py`: A→B navigation. It uses a depth-camera terrain and cliff map, a
    footprint-aware A*, replanning and hill climbing. It never stops for good: it turns, replans
    or reverses instead.
  - `drive.py`: jerk-limited drive with acceleration levels.
  - Thermal human detection. It filters by temperature band, contrast, size and shape, and needs
    4-frame confirmation. Each person gets a permanent ID (H1, H2, …) and a map marker.
  - The SAR mission: 10 search points around B, thermal scans of building faces, and a
    `SAR COMPLETE` report.
  - `obstacle_core.py` / `obstacle_tracker.py`: dynamic obstacle tracking and prediction.
  - `sim_speed.py`: 4× simulation-speed control.
  - `dynamic_check.py`: an observe-only Gazebo check.
  - `ugv_dashboard.py` + `dashboard/index.html`: the web control station.
  - `scenario_worlds.py`, `motion_test.py`.
- **Dashboard flow:**
  1. SET GOAL B (default (80, 40), Urban Street D).
  2. START A→B.
  3. On arrival the switch shows `SAR READY`.
  4. Press SAR to search. Events appear, such as `SEARCH POINT k COMPLETE` and `HUMAN H1 DETECTED`,
     then `SAR COMPLETE`.
- **Topics:** `/navigation/goal`, `/navigation/command` (START), `/sar/command` (START/STOP),
  `/sar/events`, `/sar/humans` (JSON), `/perception/obstacles`, `/navigation/status`,
  `/drive/status`.
- **Recent work (done and committed):**
  - **Dynamic obstacles:** tracker, prediction, collision geometry, ACCELERATION + button,
    4× simulation speed, and dashboard panels. 40 tests.
  - **Human/robot collision fix:** people are now primitive collision bodies (sphere head;
    cylinder trunk, hips, legs and arms). Walkers are force-driven (controller 2,
    human_contact_speed 1.3). Collision bitmasks: ground 0x01, static world 0x02, walkers 0x05,
    rover 0xffff. The rover has collision proxies (`urdf/collision_proxies.xacro`: hull, rear body,
    rear tower, mast, bracket) plus lidar, thermal-mast and camera collisions. People can no longer
    overlap the UGV.
- **Status:**
  - Verified in Gazebo: smooth driving (3.0 m/s top speed, about 1 m/s² acceleration, no wheelie;
    motion tests 16/16), plus the flat, slope, hill, obstacle and cliff scenarios.
  - Offline tests: `test_sar_offline.py` (61 pass), `test_dynamic_obstacles.py` (40).
  - Not done / open:
    - The measured real-time speed after the collision pruning (the full world ran at about
      6.5 % RTF before).
    - A full urban SAR run to the end (`test_motion.sh --urban`).
    - SLAM (the pose comes from simulator ground truth).
    - One missing person, `SAR_StaticPerson_001`. It needs a Blender re-export.
    - The **SAR autonomy supervisor** (from the "autonomy upgrade") has not started.
- **Tests:**
  - `python3 test/test_sar_offline.py` (about 8 min).
  - `bash test_motion.sh [--scenarios|--urban|--validate]` (Gazebo).
  - `bash measure_physics.sh`.
  - `stop_sar.sh` cleans up.

## 5. Domain 2: AGRICULTURE (`~/Documents/robot/ros2_ws`, package `agri_ugv`)

**World**
- A cotton farm, `worlds/field.world`, which includes `model://vigil_cotton_farm`.
- **23 crop rows**, read from the world's `row_XX_seg` links:
  - first row y = −13.414, spacing 1.219 m
  - crop x from −13.9 to 14.4
  - turn points x = −14.5 (west) and 17.0 (east)
- It contains static workers, weeds, rocks and bushes.
- A static tractor stands at (17.9, −9.6) in the east headland near row 4. Another tractor is
  near (17.6, 17.4) with a 3.5 m keep-out.
- The rover spawns at (−14.5, −13.418) facing +x (the start of row 1). It **straddles** the row:
  the tyres run in the two aisles either side and the chassis passes over the plants.

**Run**
- `./run.sh` builds with colcon every time. It sets `ROS_DOMAIN_ID=91`, `GZ_PARTITION=agri_ugv`
  and `ROS_LOG_DIR=$PWD/log/runtime`.
- Then open http://localhost:8080. The state is READY; press **START ROBOT**.
- Options: `row_count:=N` (0 = all rows), `autostart:=true`, `autonomy:=false`,
  `dashboard:=false`, `moisture:=false`, `headless:=true rviz:=false`, `stage:=N`.
- `./stop.sh` stops everything. `./demo.sh` runs the scripted demo
  (with `row_mission:=false explore:=false`).

**Bringup** (`launch/field.launch.py`, 12 stages):
- Gazebo, robot_state_publisher, ros_gz_bridge, spawn, then controllers (joint_state_broadcaster,
  wheel_controller, suspension_controller), then `drive.py`.
- Visualisation/RViz; `odometry.py` + EKF; RTAB-Map SLAM; `perception.py`.
- Nav2 (`navigation.launch.py`): controller, planner, smoother, behavior, bt_navigator,
  velocity_smoother and collision_monitor, managed by lifecycle manager `navigation_lifecycle`.
- `environment.py`, lighting and suspension; the row mission; moisture; vision; dashboard;
  resource_guard (stops the run below 1 GB of free disk).

**Command chain (important)**

```
crop_row_driver.py --/crop_row/cmd_vel_request--> row_start_gate.py (forwards only while RUNNING)
  --/cmd_vel_nav--> Nav2 velocity_smoother --/cmd_vel_smoothed--> collision_monitor --/cmd_vel-->
  drive.py (ramp, 2 m min turn radius in rows / 0.85 m in headland mode) --> wheel_controller
```

**Main nodes** (`src/agri_ugv/scripts/`)
- **`crop_row_driver.py`**: the boustrophedon mission, row 1 → last row.
  - Row-centring law at 1.2 m/s. A row is complete only once the robot has passed the last plant.
  - Headland U-turn: 0.85 m commanded radius (about 0.61 m physical) at 0.45 m/s.
  - Cross-track crop-safety stop at 0.36 m. Tractor keep-out. Camera hazard stop.
  - Obstacle handling:
    - **yield** to moving obstacles
    - **detour** round static ones on the neighbouring row line, then return to the same row
    - **end the row early** if something blocks the row end
    - **reverse U-turn** if the headland ahead is blocked
    - **(new)** if both turn areas are blocked: back down the covered row up to 3 m, then turn
  - Also: supervisor hooks (RECOVER, SPEED, HOLD/RESUME, END_ROW) and coverage bookkeeping.
  - Publishes `crop_row/progress`, `exploration/status`, `crop_row/path` and `crop_row/mode`.
- **`row_start_gate.py`**: the START/STOP gate. States: READY / RUNNING / STOPPED / COMPLETE /
  HALTED. Topics: `agri_dashboard/command`, `agri_dashboard/robot_state`.
  - **(new)** Nav2 chain watchdog: if velocity_smoother or collision_monitor is not active, it
    forwards the commands directly to `/cmd_vel`.
- **`agri_obstacles.py` + `agri_obstacle_core.py`**: the LiDAR + depth obstacle detector and
  tracker (alpha-beta filter, dynamic flag). Publishes `/agriculture/obstacles`.
  - **(new)** No phantom "moving" objects when a big static object comes into view piece by piece.
- **`agri_supervisor.py` + `agri_autonomy_core.py`**: the autonomy supervisor.
  - Mission states: READY, FIELD_ANALYSIS, ROW_NAVIGATION, OBSTACLE_AVOIDANCE, ROW_REJOIN,
    ROW_RECOVERY, MOISTURE_MAPPING, COVERAGE_VERIFICATION, REVISIT_REQUIRED, MISSION_COMPLETE, FAULT.
  - It also tracks: sensor-health levels; localization confidence; no-progress detection
    (6 s / 0.25 m) → RECOVER; END_ROW after 20 s blocked by a static obstacle; metrics; an event log.
  - Topics: `agriculture/autonomy/status` and `/command`.
  - **(new)** The no-progress FAULT hold no longer loops. It auto-retries after 30 s, and progress
    is measured on `/cmd_vel`.
- Perception: `agri_vision.py` / `agri_vision_core.py` detect crops, weeds, rocks, humans,
  equipment and so on, and publish `agri_vision/tracks`.
- Moisture: `agri_moisture_sensor.py` / `agri_moisture_map.py` produce a simulated soil probe and
  a live moisture map (`agriculture/moisture`, `agriculture/moisture_map/status`).
- Dashboard: `agri_dashboard.py` + `dashboard/index.html`. It shows camera, detections, map,
  coverage, moisture, the AGRICULTURE AUTONOMY panel, and the START/STOP buttons.
- Base nodes: `drive.py`, `odometry.py`, `perception.py`, `exploration.py` (non-row mode),
  `environment.py`, `lighting.py`, `suspension.py`, `resource_guard.py`.

**Config and documentation**
- One config for the mission: `config/agriculture.yaml`. It has field, mission, obstacles,
  moisture, perception and autonomy sections.
- Nav2 config is `config/navigation.yaml`. EKF and SLAM have their own YAML files.
- Docs: `README.md`, `ACCEPTANCE.md`, `AGRI_DASHBOARD.md`, `HACKATHON.md`, `CAD_UPDATE.md`.
- Backups: `.autonomy_backup/2026-09-24/` and `.navfix_backup/2026-09-24/`.

**Latest fix: "robot not moving / not doing rows"** (24 Sep; offline-tested, needs a Gazebo run)
- Root causes found in the runtime logs:
  - (a) The Nav2 lifecycle bringup sometimes stalls ("Configuring behavior_server"), so no
    command reaches `/cmd_vel`.
  - (b) The supervisor then misread that as no-progress, and a hold-release bug made it loop
    between HOLD and RESUME.
  - (c) At the east headland, the parked tractor plus a phantom "moving building" track blocked
    both U-turn options, and the driver held there forever (after row 1 or row 3).
- All three are fixed as described above.
- Offline tests: `python3 -m pytest -q tests` gives **72 passed**, including
  `tests/test_agri_navfix.py`. That file covers a full 23-row sweep with the east-headland
  tractor, and a both-turn-areas-blocked back-off that still finishes every row.
- Still to verify in Gazebo: a complete 23-row run. Logs are in `ros2_ws/log/runtime/`.
- Tests: `python3 -m pytest -q tests` and `python3 tools/check_structure.py`. No Gazebo is needed.
  The harness `tests/agri_mission_sim.py` runs the real driver on ROS stand-ins with a kinematic
  rover.

## 6. Domain 3: ROCK TERRAIN (`vigil_rough_terrain_ws`, package `vigil_rough_terrain`)

- **World:** an 18 × 18 m rocky terrain (`worlds/rock_terrain.sdf`, meshes in
  `models/rocky_terrain_n4/`), plus 13 test worlds in `worlds/test/`: flat, small_rocks,
  moderate_slope, small_drop, cliff_front, cliff_left, cliff_right, narrow_route, too_narrow,
  no_route and others.
- **Robot:** eight sprung wheels and two rockers, all simulated by physics (no scripted poses).
  A is (−5.4, −3.6).
- **Navigation:**
  - A depth-camera terrain classifier with classes safe, uneven, possible drop, steep, cliff and
    obstacle.
  - A* replanning (`cliff_navigator.py`) with a tilt limit (38°).
  - The dashboard shows the classified camera image, the A→B map with current and previous paths,
    and an event log such as `CLIFF DETECTED → SAFE PATH FOUND`.
- **Config:** `config/vision_nav.yaml` holds A, B, speeds, cliff and slope thresholds, and the
  footprint.
- **Launch:**
  - `vision_nav.launch.py [scenario:=cliff_front]` is the main A→B run.
  - `comparison.launch.py` runs 8 wheels against 4 wheels and writes the result to
    `/tmp/vigil-comparison.json`.
  - `terrain_sim.launch.py` starts the rover alone, for teleop.
- **Status:**
  - Verified in Gazebo: the 8-wheel rover reached B (27.7 s simulated). The 4-wheel rover got stuck
    (it did not flip).
  - Offline closed-loop tests pass: `python3 src/vigil_rough_terrain/test/closed_loop_sim.py`
    (about 4 min) and `colcon test`.
- **Not done:**
  - Nav2 on this terrain.
  - Validated contact data.
  - Proof of crossing any terrain (earlier long routes exceeded the 38° tilt limit).
  - The **rock-terrain autonomy supervisor** (terrain_autonomy_core / terrain_supervisor)
    was drafted but **never written to the device**. Treat it as not started.
- Docs: `docs/VALIDATION.md`, `docs/VISION_NAVIGATION.md`.

---

## 7. The "Autonomy Intelligence Upgrade" spec (ongoing)

Add to each domain **separately**:
- an autonomy supervisor with mission states
- decisions: CONTINUE / SLOW_DOWN / AVOID / REPLAN / BACKTRACK / RECOVER / WAIT TEMPORARILY /
  RESUME / ABORT CURRENT SUBTASK / COMPLETE MISSION
- recovery from being stuck, and no-progress detection
- a *real* localization confidence (not a fake one)
- sensor health levels: NORMAL / DEGRADED / LIMITED / SAFETY HOLD
- metrics (coverage, clearance, near misses, recoveries, manual interventions) and an event log
- changes to the existing dashboards

Status: **Agriculture done. Rock terrain not started on disk. SAR not started.**

## 8. Quick run commands

```bash
# Agriculture
cd ~/Documents/robot/ros2_ws && ./run.sh            # browser: http://localhost:8080 -> START ROBOT
# Military SAR
cd ~/Documents/military_world/sar_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash \
  && export ROS_DOMAIN_ID=72 && ros2 launch vigil_sar sar_mission.launch.py
# Rock terrain
cd ~/Documents/robot/vigil_rough_terrain_ws && source /opt/ros/jazzy/setup.bash && source install/setup.bash && export ROS_DOMAIN_ID=71 \
  && ros2 launch vigil_rough_terrain vision_nav.launch.py
# Menu
vigil
```

## 9. How to work on this project

- Before editing: read the domain's README and config, and look at the latest runtime logs.
  Agriculture logs are in `ros2_ws/log/runtime/`; ROS logs are in `~/.ros/log`.
- Keep changes inside one domain. Back up first. Add offline tests for every behaviour change,
  and run the domain's whole test suite.
- Report Gazebo verification separately from offline checks, and never claim a Gazebo result you
  did not see.
