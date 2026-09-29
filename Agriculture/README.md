# VIGIL multipurpose UGV: agriculture application

VIGIL is an eight-wheel multipurpose rover. This ROS 2 Jazzy and Gazebo
Harmonic workspace implements and evaluates its agriculture role in a cotton
farm; it does not limit the platform to agricultural use.

## Current workspace and default mission

All commands below assume you begin at the repository root; `cd ros2_ws` enters
the workspace. Its three packages, meshes and farm assets live in `src/`.

The default launcher now selects the crop-row controller:

```bash
cd ros2_ws
./run.sh row_mission:=true row_count:=23
```

It starts at row 1 and attempts a serpentine sweep with headland U-turns.
Use `row_count:=3` for a shorter run. This controller uses Gazebo ground truth,
commands a 1.20 m/s cruise speed and slows for turns. The complete field sweep
and crop/tractor clearance are unverified. Its tractor stop uses only the fixed
configured coordinate; it does not track every tractor in the field or plan a
detour. Camera/LiDAR collision-monitor inputs are currently disabled in
`navigation.yaml`; the row node's RGB-D stop is not a verified replacement,
and headland turns bypass that RGB-D stop. Do not present obstacle avoidance as
validated. The structure and algorithm checks do not validate driving safety.

The older frontier/return workflow below requires `row_mission:=false`.
Historical reports describe earlier configurations and should be read with
`ACCEPTANCE.md`; they do not certify the current row controller.

A runnable **provisional** 286.11 kg, eight-wheel rover in the supplied cotton-farm
world. All twelve implementation stages are present. Full-field exploration
completion and an exact CAD digital twin are **not yet validated**. See
`HACKATHON.md` for presentation commands and `ACCEPTANCE.md` for measured results.

## Start the simulation

```bash
cd ros2_ws
./stop.sh
./run.sh row_mission:=false explore:=false
```

This builds all three packages, starts Gazebo and RViz, spawns the rover, enables
sensors, EKF, RTAB-Map, perception, Nav2, headlights and active suspension, and
records the start pose. It pauses frontier selection for a controlled demo.
Allow approximately 30 seconds for rendering, controllers and map startup.

In another terminal:

```bash
cd ros2_ws
./demo.sh
```

The short demo requests a curved goal, returns using a Dijkstra-derived route,
tests daylight/darkness switching, and cycles suspension modes. It reports a
failure if a goal is blocked; it never labels this short demo as full exploration.
For open-ended autonomous exploration, use `./run.sh row_mission:=false` instead, or `./demo.sh start`.
Stop the launch with **Ctrl-C** before starting another. `run.sh` prevents duplicate
runs of this workspace using a process-held lock; do not delete the lock file.

For a lighter display use `./run.sh headless:=true explore:=false` (RViz remains
visible). For no windows use `headless:=true rviz:=false`.

## Ordered build and test stages

Every `run.sh stage:=N` starts a fresh simulator. Stop the preceding run first.
Diagnostic commands need the setup below in their terminal:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=91 GZ_PARTITION=agri_ugv
export ROS_LOG_DIR="$PWD/log/checks"
```

| Step | Implementation files under this workspace | Executable command | Expected result |
|---|---|---|---|
| 1. Environment | `src/agri_ugv_setup/` | `colcon build --symlink-install` then `ros2 run agri_ugv_setup check_environment.py` | Three packages build; installed dependencies pass. |
| 2. Robot | `src/agri_ugv_description/urdf/`, `meshes/` | `ros2 launch agri_ugv_description display.launch.py` | Articulated provisional rover in RViz; eight CAD-positioned wheels and two inferred rocker joints. |
| 3. World and sensors | `src/agri_ugv/worlds/`, `models/`, `config/bridge.yaml` | `./run.sh stage:=3` then `python3 tools/check_live.py --output reports/stage3_live.json` | Provided field; live RGB-D, 3D LiDAR, IMU and six encoders. |
| 4. Visualization | `config/field.rviz`, `scripts/visualization.py` | `./run.sh stage:=4` then `python3 tools/check_tf.py --root base_footprint --seconds 15 --output reports/stage4_tf.json` | All eight wheel and sensor frames connected; rear poses used only for visualization. |
| 5. Odometry | `scripts/odometry.py`, `config/ekf.yaml` | `./run.sh stage:=5` then `ros2 topic echo /odom --once` | Wheel/IMU EKF and odom transform. |
| 6. 3D SLAM | `launch/slam.launch.py`, `config/slam.yaml` | `./run.sh stage:=6` then `python3 tools/check_mapping.py` | Map, 3D cloud, camera image and map transform. |
| 7. Traversability | `scripts/perception.py` | `./run.sh stage:=7` then `ros2 topic echo /perception/status --once` | Ground fit and hazard samples; camera hazards join LiDAR in costmaps. |
| 8. Navigation | `config/navigation.yaml`, `behavior_trees/` | `./run.sh stage:=8` then `python3 tools/test_navigation.py --x 3 --y 0.5 --yaw 0.15 --output reports/navigation.json` | Smac Hybrid-A* curved path, reverse-aware RPP tracking, no spin commands. |
| 9. Exploration/return | `scripts/exploration.py`, `scripts/map_search.py` | `./run.sh stage:=9 row_mission:=false` | Frontier goal logs. `./demo.sh return` requests Dijkstra-guided return. |
| 10. Lighting | `scripts/environment.py`, `scripts/lighting.py` | `./run.sh stage:=10 row_mission:=false explore:=false` then `./demo.sh night` and `./demo.sh day` | Spotlight beam and gain-boosted image switch automatically. |
| 11. Suspension | `scripts/suspension.py`, `scripts/hydraulics.py` | `./run.sh stage:=11 row_mission:=false explore:=false` then `python3 tools/check_suspension.py` | Finite stroke/force, bounded effort; level/terrain/hybrid mode checks. |
| 12. Integrated run | `run.sh`, `demo.sh`, `launch/field.launch.py` | `./run.sh row_mission:=false explore:=false` then `./demo.sh` | Short integrated demonstration; use `./demo.sh start` for frontier exploration. |

Validate structure and algorithms:

```bash
python3 tools/check_structure.py
python3 -m pytest -q tests
rosdep check --from-paths src --ignore-src --rosdistro jazzy
xacro src/agri_ugv_description/urdf/agri_ugv.urdf.xacro > /tmp/agri_ugv.urdf
check_urdf /tmp/agri_ugv.urdf
```

## Data and important limits

- The supplied world is retained as `worlds/original.world`. The runnable copy
  patches heightmap collision pose/friction, lighting and unsupported actor
  definitions. Existing worker meshes provide the moving obstacle. Farm assets
  were reused from the supplied archive's extracted directory.
- The STL is an assembly mesh without joint axes/material properties. All 90 components of the supplied
  `full shhhh_activesuspension.stl` are retained as 21 articulated visual meshes.
  Wheel locations and the raised rear-wheel stance follow the export. The two
  rocker pivots are inferred from the rear cross-shaft; wheel axles are fixed to
  each rocker. Inertia, cylinder anchors, joint dynamics and standing angle remain
  provisional. The shell is 26.4 kg; assumed hardware/payload yields 286.11 kg total.
  Cylinder visuals follow the rocker without a closed-loop hydraulic mechanism.
  See [CAD update notes](CAD_UPDATE.md) for reproducibility and validation.
- Wheel commands drive all eight joints. Only the first three wheels per side
  expose encoder feedback. Rear wheel transforms come from simulator poses for
  display only. EKF/RTAB-Map do not consume these poses; the separate row mission does use
  `/sim/ground_truth` to track the supplied field coordinates.
- EKF uses six-wheel velocities and IMU, with increased uncertainty during slip.
  EKF alone does not eliminate position drift; RTAB-Map supplies map corrections.
  The EKF is planar; graph SLAM and sensor clouds remain 3D.
- Nav2 uses Smac Hybrid-A* with 2 m minimum radius and Regulated Pure Pursuit,
  reversing enabled and heading rotation disabled. MPPI tuning is retained in
  `navigation_mppi.yaml` as an experimental alternative, not the default.
- Frontier bounds default to map-frame `[-10,60] x [-10,60]` metres, surrounding
  the agricultural plot from the original exploration test spawn at world `(-25,-25)`; the current row spawn is
  `(-14.5,-13.418)` and frontier bounds need revalidation. These are an
  operational boundary, not satellite coordinates. Unknown cells remain blocked.
- Dijkstra computes the shortest route in an inflated eight-connected grid;
  Nav2 fits drivable paths through spaced waypoints. This does not prove the
  shortest curvature-constrained trajectory. Arrival tolerance is 0.30 m and
  0.30 rad, not mathematically exact pose equality.
- LDR is a synthetic illumination scale (sun intensity times 1000), tied to
  actual commanded Gazebo lighting. The test zone dims global sun illumination
  when entered; it is not a physical local photometric sensor or natural shadow
  model. A spotlight physically changes rendering. Gain boost is on
  `/camera/low_light_image`; mapping uses raw RGB aided by the real spotlight.
- Cylinder debug force is computed from commanded joint torque by virtual work;
  it is not a measured hydraulic pressure or a full fluid simulation.
- Logs are under `log/`; reports/images under `reports/`; each SLAM session and
  mission journal goes under `run/`. Keep disk space available. The resource
  guard shuts down below 1 GiB free; a SLAM process exit shuts down the launch.
  Unlinked observations are not retained in the database, and working memory is
  capped at 500 graph nodes. Archived maps are not loaded into a new run.

Jazzy/Harmonic pairing: https://gazebosim.org/docs/harmonic/ros_installation/
RPP configuration: https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/configuring_regulated_pp/
