# VIGIL repository audit

SIH26126, *Vision Based Autonomous Navigation for Unmanned Ground Vehicle for Outdoor environment*.
Audit date: 25 November 2026. This audit inspected the code and ran the offline test suites. **It did not run ROS 2 or Gazebo.** Runtime claims below come from the repository's own earlier records, and they are labelled that way.

[Validation report](VALIDATION_REPORT.md) · [Demo plan](DEMO_PLAN.md) · [Project overview](../README.md)

## 1. Baseline

| Item | Value |
|---|---|
| Branch | `main` (remote also has `feature/cross-platform-runtime` and `revert/docker-runtime-2026-10`) |
| Commit audited | `4f9c8c1` "Revert portable Docker runtime" (15 Nov 2026) |
| Working tree before edits | clean |
| Target stack, as declared | Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, plus Nav2, RTAB-Map and `robot_localization` (Agriculture and New Mining) |
| Build type | All 6 packages use `ament_cmake` and install Python scripts with `install(PROGRAMS …)` |
| Audit machine | Ubuntu 24.04, Python 3.13, no ROS 2, no Gazebo, no GPU |

## 2. Package inventory

| Folder | ROS package(s) | Role | Entry point |
|---|---|---|---|
| `Rock Terrain/` | `vigil_rough_terrain` | Main SIH26126 pipeline: depth camera → geometric terrain map → footprint-aware A* → `/cmd_vel` → 8 wheels; web dashboard; 13 test worlds | `ros2 launch vigil_rough_terrain vision_nav.launch.py` (domain 71) |
| `Agriculture/` | `agri_ugv`, `agri_ugv_description`, `agri_ugv_setup` | Cotton-field world: crop-row controller, EKF, RTAB-Map, Nav2 (Smac Hybrid-A* + RPP), perception, lighting, active suspension, soil-moisture simulation | `./run.sh` → `ros2 launch agri_ugv field.launch.py` (domain 91) |
| `Military Search and Rescue/` | `vigil_sar` (in `sar_ws/`) | 300 × 300 m disaster world, rough-terrain navigator extended with dynamic-obstacle tracking, thermal human detection and a search-mission manager | `ros2 launch vigil_sar sar_mission.launch.py` (domain 72) |
| `New Mining/` | `vigil_new_mining` | Underground mine: RGB-D visual odometry, RTAB-Map and a wheel/IMU/visual EKF; grid A* with pure pursuit; optional YOLO; synthetic gas, temperature and humidity | `./run.sh` → `ros2 launch vigil_new_mining mining.launch.py` (domain 74) |
| `Existing Mining/` | none | Documentation and images only | n/a |
| `Mechanical Design/` | none | Active-suspension CAD export (STL) | n/a |
| `vigil/` | none | Interactive launcher menu (options 1–5) and maintainer scripts | `python3 vigil/master_launcher/vigil_launcher.py` |

The workspaces are independent: each has its own copy of the rover description, meshes and (for Rock Terrain and SAR) the terrain/planner cores. `terrain_core.py` differs by 20 lines between Rock Terrain and SAR; `planner_core.py` differs by 286 lines.

## 3. Architecture: Rock Terrain (the clearest SIH26126 path)

```
gz sim (rock_terrain.sdf | worlds/test/<scenario>.sdf)
  └─ ros_gz_bridge (config/bridge.yaml)
       /camera/depth_image, /camera/image, /camera/camera_info, /imu/data,
       /lidar/points, /encoders, /sim/ground_truth, /sim/link_poses, /clock
terrain_mapper.py   depth + camera_info + /sim/ground_truth (+ segmentation labels)
   → /vision/terrain_classes, /vision/traversability, /vision/slope, /vision/roughness,
     /vision/cliff, /vision/terrain, /vision/overlay           (OccupancyGrid frame: "world")
cliff_navigator.py  classes + slope + roughness + /sim/ground_truth + /navigation/goal | /goal_pose
   → /cmd_vel, /navigation/path, /navigation/status
drive.py            /cmd_vel → wheel_controller/commands (8 wheel velocities; ramp, 0.6 s watchdog)
ugv_dashboard.py    http://localhost:8080
```

- **Frames:** grids and paths use the `world` frame. The robot pose comes straight from `/sim/ground_truth`, not from TF. All nodes set `use_sim_time: true`.
- **QoS:** sensor topics use `qos_profile_sensor_data`, and grids use depth 2. These are consistent between publisher and subscriber.
- **Parameters:** `config/vision_nav.yaml` derives every robot dimension and threshold from the URDF and a measured flat-ground calibration, and documents each one.

### Sensor usage (Rock Terrain)

| Sensor | Bridged | Consumed by navigation? |
|---|---|---|
| RGB-D depth | yes | **Yes.** It is the only geometric input to the terrain map. |
| RGB / HD camera | yes | Display overlay only |
| Segmentation labels (simulator) | yes (`segmentation:=true`) | Marks "no ground" rays only. It is a simulator oracle with no real-sensor equivalent. |
| LiDAR | yes | **No** |
| IMU | yes | **No** (tilt comes from the ground-truth orientation) |
| Wheel encoders | yes | **No** |
| Simulator ground-truth pose | yes | **Yes**, as the only pose source |

## 4. Localization by environment

| Environment | Pose used by the navigation loop | Visual localization in the loop? |
|---|---|---|
| Rock Terrain | `/sim/ground_truth` | **No** |
| Search and rescue | `/sim/ground_truth` (navigator, terrain mapper, obstacle tracker, thermal detector, mission manager) | **No** |
| Agriculture, row mission (default) | `/sim/ground_truth` (`crop_row_driver.py`, `agri_supervisor.py`) | **No** |
| Agriculture, Nav2 / frontier mode (`row_mission:=false`) | EKF (wheels + IMU) with RTAB-Map map correction | Partly: RTAB-Map runs, but its accuracy has not been measured |
| New Mining | TF `map → base_footprint` from RTAB-Map plus an EKF fusing RGB-D visual odometry, wheel odometry and IMU | **Yes, structurally.** Localization error against ground truth has not been measured. |

**Implication for SIH26126:** the headline rough-terrain demonstration is *vision-based terrain perception with simulator localization*. It is not GPS-denied visual navigation. New Mining is the only workspace whose navigation loop runs on a sensor-derived pose.

## 5. Terrain, obstacles and planning (Rock Terrain)

- **Terrain classification is geometric and rule-based.** It covers plane-fit slope over a 0.9 m window, roughness, step and drop. The thresholds come from wheel radius, ground clearance and the tilt limit. There is no learned model.
- **Terrain costs do affect planning.** In the offline runs, `hill_choice` avoids the 38° face and climbs a 22° flank (peak tilt 22.4°), `cliff_front` detours 3.15 m sideways, and `big_rocks` treats rocks as cost rather than obstacle (0 obstacle cells). See the [validation report](VALIDATION_REPORT.md).
- **Obstacles** are whatever the depth geometry classifies as STEP, OBSTACLE or CLIFF. There is no LiDAR input and no tracking of moving obstacles in this package. SAR adds dynamic-obstacle tracking from depth plus simulator segmentation labels.
- **Update, 25 Nov 2026:** bounded failure handling, replanning hysteresis and new cliff-safety checks were added afterwards. See [IMPROVEMENT_REPORT.md](IMPROVEMENT_REPORT.md). The paragraph below describes the code as originally audited.
- **Failure handling (as audited):** there is a "never give up" escalation: local replan → whole-map replan → hill-climb replan → heading search (`CLIFF_AVOIDANCE`) → reverse-and-turn recovery. The mission ends only at B. **There is no bounded safe-stop**, so an unreachable goal is reported as continuous `SEARCHING`/`CLIFF_AVOIDANCE` rather than a terminal failure. That is acceptable in simulation, but it is a real-world safety gap.

## 6. Feature status

| Capability | Status | Basis |
|---|---|---|
| 8-wheel rover model, rockers, passive wheel suspension (4 workspaces) | IMPLEMENTED | All four URDFs expand: one root, no broken joints, all 29/21 meshes resolve. Runtime physics was not re-run in this audit. |
| Depth-based geometric terrain/cliff classification | IMPLEMENTED | Offline closed-loop test passes 13/13 scenarios (idealized depth, perfect pose) |
| Footprint-aware A* with terrain costs and replanning | IMPLEMENTED | Same offline test |
| Live Gazebo A-to-B run on rock terrain | PARTIALLY IMPLEMENTED / NOT VERIFIED here | One earlier run reported `GOAL_REACHED` ([evidence](validation.md)). No repeated trials and no collision instrumentation. |
| Visual / GPS-denied localization in the navigation loop | PARTIALLY IMPLEMENTED (New Mining only) | Error against ground truth not measured |
| Learned terrain segmentation / AI perception | NOT IMPLEMENTED | Rule-based geometry. New Mining's YOLO is optional, and its weights are not in the repository. |
| LiDAR-based obstacle avoidance | PARTIALLY IMPLEMENTED (Agriculture Nav2 costmaps) | Collision-monitor sources are disabled in `navigation.yaml`; the row mission does not use Nav2 |
| Dynamic-obstacle avoidance | PARTIALLY IMPLEMENTED (SAR) | Offline tests pass. Uses simulator segmentation labels and ground-truth pose. |
| Thermal human detection | IMPLEMENTED (SAR, simulated thermal camera) | Offline tests pass; full mission not verified |
| Gas / temperature / humidity monitoring | Simulated values only (New Mining) | A configured field is sampled at the simulator's true position. These are not sensor models. |
| Active hydraulic levelling | IMPLEMENTED (Agriculture) | `test_hydraulics.py` passes. The Rock Terrain copies are unused (see §7). |
| Bounded safe-stop on unreachable goal / sensor loss | IMPLEMENTED 25 Nov 2026 (offline tests and Gazebo simulation regression, 18/18 PASS; not tested on hardware) | Was NOT IMPLEMENTED when audited; see [IMPROVEMENT_REPORT.md](IMPROVEMENT_REPORT.md) |
| Existing Mining environment | PLANNED | No package |
| Real hardware | PLANNED | Simulation only |

## 7. Issues found

| # | Severity | Issue | Action |
|---|---|---|---|
| 1 | High (onboarding) | The root README Quick start ran `cd vigil_rough_terrain_ws`, which doesn't exist. Six README links pointed at old folder names (`vigil_rough_terrain_ws/`, `ros2_ws/`, `military_world/`). | **Fixed** |
| 2 | High (claims) | The root README said the mining application was "developed and tested … with human detection and hazardous gas monitoring". `Existing Mining/` has no code, and New Mining's gas values are synthetic. | **Fixed:** split into separate Existing Mining and New Mining rows with accurate scope |
| 3 | Medium | Launcher option 5 bypassed `New Mining/run.sh`'s check for the git-ignored licensed mine assets, so a fresh clone would start an incomplete world. | **Fixed:** the launcher now runs the same check |
| 4 | Medium | Launcher option 4 printed only "launch file not found"; the package exists only on the author's PC. | **Fixed:** the message now explains this and points to option 5 |
| 5 | Medium | The launcher's "Build it once" hint printed unquoted paths, so `cd …/Military Search and Rescue/sar_ws` failed when pasted. | **Fixed:** paths are quoted |
| 6 | Medium (process risk) | `vigil/publish_to_github.sh` still maps to the old layout and overwrites `README.md` with a local `REPO_README.md`. | **Not changed** (publishing script). A warning was added to `vigil/README.md`. Update the folder map before using it. |
| 7 | Low | `Agriculture/README.md` and `AGRI_DASHBOARD.md` said `cd ros2_ws` and referenced a missing `HACKATHON.md`. `Rock Terrain/README.md` used `/home/user/...` paths and listed 10 of 13 scenarios. | **Fixed** |
| 8 | Low | `vigil/README.md` described 3 projects, listed files that don't exist (`COLCON_IGNORE`, `REPO_README.md`) and had a broken `#4-build` link. | **Fixed** |
| 9 | Low | `docs/setup.md` rosdep command omitted `New Mining/src`. | **Fixed** |
| 10 | Low | `Rock Terrain/.../scripts/suspension.py` and `hydraulics.py` are byte-identical copies of the Agriculture files. They are not installed or launched, they subscribe to `imu/with_covariance` (not bridged there), and they target an unconfigured `suspension_controller`. | Documented in `Rock Terrain/README.md`; not deleted |
| 11 | Low (clutter) | 17 files under `Agriculture/.autonomy_backup/`, `.navfix_backup/` and `.coverage_dynamic_backup/` are tracked even though `.gitignore` ignores `.*/`. | Not deleted (needs owner approval) |
| 11b | Low (test coverage) | Two SAR tests that check the SAR rover and physics match the rough-terrain package looked only in the author's `~/Documents/robot/vigil_rough_terrain_ws`, so they were silently skipped in any clone. | **Fixed:** `test_sar_offline.py` now prefers the repository's `Rock Terrain/` copy (and `$VIGIL_ROUGH_TERRAIN` still overrides). Both tests now run and pass. |
| 12 | Info | `Agriculture/.../gazebo_gui.config` fails `xmllint` | Expected: Gazebo GUI configs have several root elements by design. Not a defect. |
| 13 | Info | Duplicated cores and meshes across workspaces (about 82 MB of meshes in 3 copies) | Recommendation below |

No hard-coded secrets were found. pyflakes reports no undefined names across all 126 Python files.

## 8. Prioritized recommendations

1. **Put a sensor-derived pose in the Rock Terrain loop.** Add a `pose_topic` parameter, defaulting to `/sim/ground_truth`, to `terrain_mapper.py` and `cliff_navigator.py`. Feed it from an EKF (wheel + IMU + RGB-D visual odometry), reusing New Mining's `rgbd_odometry` and `ekf.yaml` pattern. Log the error against ground truth. This is the single biggest step toward the actual SIH26126 requirement. It was not done in this audit because it cannot be validated without Gazebo.
2. **Add a bounded safe-stop:** a terminal `GOAL_UNREACHABLE` state after N escalations or T seconds, plus stopping on stale depth or pose. Test it with the existing `too_narrow` and `no_route` scenarios.
3. **Add repeated runtime trials with metrics:** goal reached, time, path length, minimum clearance and recoveries for `flat`, `cliff_front`, `narrow_route` and `hill_choice`, at least 5 runs each, using the same structure as `test/closed_loop_sim.py`.
4. **Add noise to the offline test:** depth noise, dropouts and pose drift. Today it passes with perfect sensing.
5. **Fix `publish_to_github.sh`** before using it again (issue 6).
6. With owner approval, remove the tracked backup folders and the unused suspension copies in Rock Terrain.
7. Longer term, move the shared rover description and the terrain/planner cores into one package that the workspaces depend on.
