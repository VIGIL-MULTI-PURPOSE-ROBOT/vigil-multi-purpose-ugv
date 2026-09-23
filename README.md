# VIGIL multi-purpose robot — simulation projects

ROS 2 **Jazzy** / Gazebo **Harmonic** simulations of the eight-wheel VIGIL rover (twin rockers,
independently sprung wheels), in three separate environments, plus a launcher menu that starts
one of them at a time.

| Folder | Environment | Package(s) | Main launch file |
|---|---|---|---|
| [`military_world/`](military_world) | **Military search and rescue** — 300 × 300 m disaster area, thermal human detection | `vigil_sar` (in `military_world/sar_ws`) | `sar_mission.launch.py` |
| [`ros2_ws/`](ros2_ws) | **Agriculture** — cotton farm, crop-row missions | `agri_ugv`, `agri_ugv_description`, `agri_ugv_setup` | `field.launch.py` (via `run.sh`) |
| [`vigil_rough_terrain_ws/`](vigil_rough_terrain_ws) | **Rock terrain** — 18 × 18 m rocky terrain, cliff detection | `vigil_rough_terrain` | `vision_nav.launch.py` |
| [`vigil/`](vigil) | **Master launcher** — menu, no robot code | — | `master_launcher/vigil_launcher.py` |

The three projects are **independent**: each has its own packages, world, robot description,
controllers, parameters, dashboard and ROS_DOMAIN_ID, and builds on its own. Nothing is shared
between them, and only one runs at a time.

## Requirements

Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, `ros-jazzy-ros-gz`, `ros-jazzy-gz-ros2-control`,
`ros-jazzy-ros2-controllers`, `ros-jazzy-xacro`, Python 3 with `numpy`, `opencv-python`, `pyyaml`.
For the agriculture project also `rosdep install --from-paths ros2_ws/src --ignore-src -r -y`
(Nav2, RTAB-Map, robot_localization).

## Quick start: the launcher

```bash
bash vigil/setup_vigil.sh                        # once: installs the `vigil` command
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc   # once, if needed
vigil
```

```
=============================================
          VIGIL SIMULATION LAUNCHER
=============================================

Select Environment:

1. Military Search and Rescue
2. Agriculture
3. Rock Terrain
4. Exit

Select option [1-4]:
```

For the chosen project the launcher sources `/opt/ros/jazzy` and **only that project's**
`install/setup.bash`, runs its own launch file, and hands it the terminal. Ctrl+C stops it and
returns to the menu; anything it left running is stopped, and processes still running from any
of the three projects are stopped before the next one starts. The launcher never builds; build
each project once (below). It expects the projects at `~/Documents/military_world`,
`~/Documents/robot/ros2_ws` and `~/Documents/robot/vigil_rough_terrain_ws` (see `vigil/README.md`).

## Build (each project separately, in a fresh terminal)

```bash
# Military SAR
cd military_world/sar_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_sar --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# Agriculture
cd ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install

# Rock terrain
cd vigil_rough_terrain_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```

---

## 1. Military search and rescue — `military_world/`

`military_world/` is the disaster-response world (authored in Blender, exported to Gazebo; no
weapons or combat content). `military_world/sar_ws` is the ROS 2 workspace that runs the rover in it.

**Run:** `ros2 launch vigil_sar sar_mission.launch.py` (ROS_DOMAIN_ID 72), dashboard
<http://localhost:8080>. Set point B on the dashboard map; the rover drives there. The **SAR**
switch is available at any time: it searches 10 points around B, scans building faces with the
thermal camera and reports each person once (H1, H2, …) with a map marker.

- **Sensors:** 4K RGB display camera, depth camera (terrain / cliff mapping), segmentation,
  thermal camera (320 × 240, 60°, human band 296.5–311 K), LiDAR, IMU.
- **Human detection:** temperature band + contrast + size/shape filters, range from the depth
  image (ground plane when out of depth range), 4-frame confirmation, permanent IDs.
- **World scale:** 1 Gazebo unit = 1 m, verified by `scale_check.py` (person 1.74 m, car
  4.4 × 1.8 m, ISO container 6.06 m, roads 4–6 m, doors 1.04 m; rover 1.53 × 1.12 m).
  The rover works in a configurable 170 × 170 m **operational zone** (`world.zone_*`) inside the
  300 m world; B is kept inside it and the planner searches only a local window around A and B.
- **Driving:** 3.0 m/s on open road. The target speed follows the clear distance ahead
  (≈10 m → 3.0 m/s, 2 m → 2.0, 1 m → 1.0, below 0.5 m no forward motion): the rover turns,
  replans or reverses instead of stopping. Cliffs are hard constraints; climbable hills are
  climbed. Smooth jerk-limited acceleration (1.0 m/s², braking 2.0 m/s²) keeps the front wheels
  down (`rover_stability.py` computes the wheelie limit from the URDF).
- **Physics:** Earth gravity 9.81 m/s², 1 ms step, DART, ground friction 1.0, tyre 0.85,
  150 N·m per wheel (`config/physics.yaml`).
- **Config:** everything in `config/sar_mission.yaml` and `config/physics.yaml`.
- **Tests:** `python3 src/vigil_sar/test/test_sar_offline.py` (offline, no Gazebo);
  `bash test_motion.sh --validate` (Gazebo: scale check, acceleration / braking / turning /
  wheel contact, flat road / slope / steep hill / obstacle / cliff scenarios, A→B + SAR search).
- Details: [`military_world/sar_ws/README.md`](military_world/sar_ws/README.md),
  world: [`military_world/README.md`](military_world/README.md).

## 2. Agriculture — `ros2_ws/`

Eight-wheel rover in the supplied cotton farm (rover CAD: see
[`ros2_ws/CAD_UPDATE.md`](ros2_ws/CAD_UPDATE.md)).

```bash
cd ros2_ws
./run.sh row_mission:=true row_count:=23     # builds, then starts Gazebo + RViz (ROS_DOMAIN_ID 91)
./run.sh headless:=true rviz:=false row_mission:=true row_count:=3
./stop.sh                                    # from another terminal
```

Includes EKF odometry, RTAB-Map 3D SLAM, traversability perception, Nav2 (Smac Hybrid-A* +
regulated pure pursuit), frontier exploration, lighting and hydraulic suspension. Tests:
`python3 -m pytest -q tests`.

Status: simulation prototype. Row missions use Gazebo ground truth; full 23-row completion, crop
clearance and tractor avoidance have **not** passed an end-to-end acceptance run. See
[`ros2_ws/README.md`](ros2_ws/README.md), [`HACKATHON.md`](ros2_ws/HACKATHON.md) and
[`ACCEPTANCE.md`](ros2_ws/ACCEPTANCE.md).

## 3. Rock terrain — `vigil_rough_terrain_ws/`

The rover on an 18 × 18 m rocky terrain with depth-camera cliff detection, footprint-aware A*
replanning, hill climbing and a web dashboard.

```bash
ros2 launch vigil_rough_terrain vision_nav.launch.py            # ROS_DOMAIN_ID 71, http://localhost:8080
ros2 launch vigil_rough_terrain vision_nav.launch.py scenario:=cliff_front   # test worlds
ros2 launch vigil_rough_terrain comparison.launch.py            # 8-wheel vs 4-wheel comparison
```

Details and measured results: [`vigil_rough_terrain_ws/README.md`](vigil_rough_terrain_ws/README.md),
[`VISION_NAVIGATION.md`](vigil_rough_terrain_ws/src/vigil_rough_terrain/docs/VISION_NAVIGATION.md).

---

## Status

Simulation only. Each project's README states what has been validated in Gazebo and what has
only been tested offline. Build output (`build/`, `install/`, `log/`) is not in the repository.
