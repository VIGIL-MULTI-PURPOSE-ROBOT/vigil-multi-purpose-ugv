# VIGIL SAR UGV: search and rescue in military_world

ROS 2 **Jazzy** + Gazebo **Harmonic** workspace inside `military_world/sar_ws`.

It reuses the VIGIL eight-wheel rover and its full perception and navigation stack from
the rock-terrain project (`vigil_rough_terrain_ws`). That workspace was only read, never written.

> **Status (24 Sep 2026).** In Gazebo: driving at 3.0 m/s with smooth acceleration is verified
> (motion tests 16/16, flat road / slope / steep hill / obstacle / cliff scenarios). The detection
> and SAR mission logic pass offline tests, including a full simulated mission. **Open:** the full
> world runs far below real time (measured 6.5 %). A collision-shape reduction was added on 24 Sep,
> and its speed-up is not yet measured. The long urban SAR run in Gazebo is not finished yet.
> See [Simulation speed](#simulation-speed).
>
> **25 Sep 2026: dynamic obstacles.** Added: collision boxes on walking people, obstacle tracking and
> prediction, collision status, the ACCELERATION + button and a 4× simulation-speed target (see
> [Dynamic obstacles](#dynamic-obstacles-collisions-acceleration-and-simulation-speed)). All of it is
> tested offline (36 tests, closed loop with the real navigator). It has **not run in Gazebo yet**,
> and the 4× target has not been measured on a real PC.
>
> Paths such as `~/Documents/military_world` below are the author's. In a clone of the
> repository, use `<repo>/military_world` instead.
On top of that stack it adds:

- a 4K RGB camera
- a thermal camera
- thermal human detection and tracking
- a 10-point search-and-rescue mission
- a SAR control-station dashboard

```
military_world/                      (your world, unchanged)
├── gazebo_export/military_world.sdf   source world - never modified
├── sar_metadata.json                  building footprints used for SAR planning
└── sar_ws/                            NEW
    ├── README.md  stop_sar.sh
    ├── generated/military_sar.sdf     written at every launch by build_sar_world.py
    └── src/vigil_sar/
        ├── config/sar_mission.yaml    THE configuration file (all parameters)
        ├── config/bridge.yaml  config/controllers.yaml
        ├── urdf/  (rover xacros + sar_sensors.xacro)   meshes/ (31 rover CAD STLs)
        ├── launch/ sim | navigation | sar | dashboard | sar_mission (all-in-one)
        ├── scripts/  nodes + cores (below)
        ├── dashboard/index.html
        └── test/test_sar_offline.py  test/test_dynamic_obstacles.py
```

## Mission

1. `sar_mission.launch.py` starts `military_world`, then spawns **one** rover at A: the `SAR_ROBOT_START` marker (0, −8), facing +Y.
2. On the dashboard, click **SET GOAL B**, then click on the map. The default B is (80, 40) on Urban Street D.
3. Press **START A→B**. The existing navigator drives with depth-camera terrain and cliff detection, the footprint-aware A*, replanning, hill climbing, and PATH BLOCKED → REPLAN → alternative route → recovery. "No safe path" never makes it stop and wait.
4. The robot reaches B and waits. The switch shows `SAR READY`, and SAR does **not** start by itself.
5. Press the **SAR** switch. It changes to `SAR ACTIVE`, and 10 search points are generated as a 2×5 boustrophedon around B, pushed out of buildings where needed.
6. At each search point the robot scans with the thermal camera. The default is 8 headings; headings already seen on arrival are skipped.
7. Every building within 25 m gets building views. For each face the robot stops in front of the wall, turns the thermal camera to face it, slows down and sweeps ±25°.
8. When a warm candidate appears, the robot turns to it to confirm it (humans come first). After 4 consecutive frames the event `HUMAN Hn DETECTED` fires. A marker appears on the map and an entry in the human list, and neither is ever duplicated.
9. After all points, building views and a final scan, the mission reports `SAR COMPLETE` with the number of humans found and 10/10 search points. The robot then holds its position.

Throughout, people, vehicles and moving obstacles are tracked. The rover keeps its distance from them, predicts their motion and plans round it. See the next section.

## Dynamic obstacles, collisions, acceleration and simulation speed

**Collision geometry.** People are physical bodies, not visuals only. The robot's collision covers its whole body.

**Why the walker overlapped the rover (the screenshot of 24 Sep):**
- People used controller 1, which is kinematic: `sar::WaypointSystem` **set the person's pose every physics step**.
- A pose that is set is not simulated. The person was placed on its route even when the rover stood there, and the contact solver could only shove the rover afterwards.
- On top of that, the rover's collision was a 1.07 × 0.50 m chassis box. The rear body, the sides beyond ±0.25 m, the rear tower and the mast were visual only: 8 % of the CAD body.

| what | now (`build_sar_world.py` step 10, `urdf/collision_proxies.xacro`) |
|---|---|
| every person (37: walking, standing, lying, trapped, inside buildings) | a **primitive collision body** in the person's own link, so it moves with the person:<br>• sphere head<br>• cylinder trunk and hips<br>• cylinder legs and arms<br>Each primitive sits on the exported body-part pose (a leaning leg stays leaning), with radii covering the part (`world.human_collision: primitives`) |
| walking people | **`people_controller: 2`** (military_world's contact-aware controller):<br>• dynamic rigid bodies under gravity<br>• walked by a **force** and kept upright by a balance torque<br>• no pose is ever set<br>A person blocked by the rover stops and waits. The rover can never overlap them. Walking speed is `human_contact_speed: 1.3` m/s. |
| contact groups (collide bitmask; a pair collides when the masks share a bit) | ground 0x01, static world 0x02, walking people 0x05, rover 0xffff (untouched):<br>• person × rover, person × ground and person × person: **yes**<br>• person × wall or prop: no (their authored routes pass door frames, as before) |
| robot | chassis box plus **hull (full 0.70 m width), rear body, rear tower, mast, LiDAR, thermal mast and camera** (collision only, no mass change, ≥ 0.39 m above flat ground), 8 wheel cylinders, rocker beams. It now covers **100 %** of the CAD body (was 91.6 %). |
| physics engine | unchanged: DART with the rover-validated ode detector (`people_contact_detector: world`) |
| buildings, walls, vehicles, rocks, debris | the export's boxes and cylinders; everything the rover can reach keeps its collision (test-checked) |
| segmentation labels | HUMAN 10, VEHICLE 20, BUILDING 30, ROCK 40, DEBRIS 50, VEGETATION 60, OBJECT 70 in the zone |

**Gazebo proof** (real contact physics, judged on the true poses): `bash test_motion.sh --scenario human_block --scenario human_standing --scenario human_crossing`

- `human_block`: the test itself drives the rover **into** a standing person at 0.8 m/s, with no navigator. The contact must stop it, with no overlap beyond a few cm of solver penetration.
- `human_standing`, `human_crossing`: the navigator must never touch the standing or walking person, must keep its distance, and must reach B.

**Dynamic obstacles** (`scripts/obstacle_core.py`, node `obstacle_tracker.py`, section `obstacles:`):

1. **Detection.** Depth + segmentation + pose (existing sensors). The depth frame is paired with the segmentation frame of the same stamp and the pose at that stamp, so it stays correct at any simulation speed. The points are clustered per class.
2. **Tracking.** Each track has a position, velocity, speed and direction over the frames (alpha-beta filter). It is DYNAMIC above 0.3 m/s. Walls, rocks, debris and vegetation never get a velocity. Output: `/perception/obstacles`.
3. **Prediction and planning.** `cliff_navigator` stamps people, vehicles and anything moving into the planner's map, with the class safety distance plus `robot_footprint_margin`. Moving ones are swept along the path predicted over `collision_prediction_time`. The unchanged planner then inflates this by the rover's half-width, replans, and slows down smoothly through its clearance speed profile. The rover's own planned motion is checked against the predicted obstacles, which gives **COLLISION RISK**, a replan, and a smooth yield (never a turn, never faster).
4. **Humans first.** People get the largest distance (2.0 m). The rover slows within 4 m of a person even beside the path, and yields to a person crossing.
5. **No stop-forever.** After `yield_timeout` (6 s) the rover plans round where the obstacle *is*; the navigator's escalation (wide replan, climb, recovery) does the rest.
6. **Cliffs** are never relabelled by the overlay; they stay hard constraints.
7. **Narrow paths.** Static walls and objects are not stamped again (the terrain map has them), so every gap the 1.12 m footprint fits through stays open. A 1.7 m gap is tested.
8. While the tracker runs, `terrain_mapper` leaves people's pixels to it (`terrain.tracked_labels_excluded: [10]`), so a walking person leaves no trail of stale obstacle cells.

| parameter (`obstacles:`) | default |
|---|---|
| `dynamic_obstacle_distance` | 1.5 m |
| `human_safety_distance` | 2.0 m |
| `vehicle_safety_distance` | 1.5 m |
| `static_obstacle_distance` | 0.3 m |
| `collision_prediction_time` | 4.0 s |
| `robot_footprint_margin` | 0.15 m |

**Dashboard additions:**

- **COLLISION** pill: `CLEAR` / `COLLISION RISK` / `AVOIDING`.
- **DYNAMIC OBSTACLES** panel: count, class, distance, direction, speed, and status (STATIC / MOVING / APPROACHING / TOO CLOSE).
- **Map:** tracked obstacles with their safety ring, velocity arrow and predicted path.
- **ACCELERATION +** (and −): the speeding-up limit steps NORMAL 1.0 → HIGH 1.5 → MAX 2.0 m/s² (`physics.drive.accel_levels`).
  - It works through `drive.py`'s jerk-limited DriveLaw, so the change is smooth.
  - Top speed (3.0 m/s), braking and every safety check are unchanged. There is no force and no teleport.
  - Each level is capped at 50 % of the wheelie limit at the current pitch: 7.7 m/s² on flat ground, 1.9 m/s² on a 33° climb.
- **SIM SPEED:** 1× / 2× / 4× buttons (`gz set_physics`), with the target and the **measured** real-time factor.

**Simulation speed:** `world.simulation_speed: 4.0` writes `<real_time_factor>4</real_time_factor>`
into the generated world. `sim_speed:=1|2|4` on the launch line or the dashboard buttons change it.

- **Execution speed only:** the 1 ms step, gravity, masses, friction, torques and sizes are unchanged.
- Every node runs on simulation time, so sensors, navigation and the drive stay in step.
- If the PC cannot keep up, Gazebo runs as fast as it can, and the dashboard shows the real factor.
- Before the collision pruning, the full world ran at **6.5 %** of real time. Whether 4× is reachable depends on the PC: run `bash measure_physics.sh`, whose last line is a real mission with all sensors, and report the number. The 1 ms step is not raised to 2 ms, because the rover's suspension was validated at 1 ms.

**Check in Gazebo:** while a mission runs, `ros2 run vigil_sar dynamic_check.py` prints and saves (`diagnosis/dynamic_check.json`):
- the closest approach to people and vehicles, against the safety distances
- time spent CLEAR / COLLISION RISK / AVOIDING
- navigator states (a stop-forever would show up here)
- acceleration levels used
- the measured simulation speed

## Build

```bash
cd ~/Documents/military_world/sar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_sar --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_DOMAIN_ID=72          # different from vigil_rough_terrain (71) so the two never mix
chmod +x src/vigil_sar/scripts/*.py src/vigil_sar/test/*.py stop_sar.sh
```

The walking people need the `sar::WaypointSystem` plugin in `military_world/build/`. The plugin is not in
the repository, so build it once (needs `cmake`, `g++` and gz-sim8; source `/opt/ros/jazzy/setup.bash` first):

```bash
cd ~/Documents/military_world
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target sar-waypoint-system -j2
```

## Run: everything in one command

```bash
ros2 launch vigil_sar sar_mission.launch.py            # then open http://localhost:8080
#   gui:=false                     server only
#   goal_x:=80 goal_y:=40          preset B (it can still be changed on the dashboard)
#   If Gazebo crashes on this PC, run  bash find_crash.sh  once (see Troubleshooting).
```

## Run: separate terminals

Each terminal needs this first: `source /opt/ros/jazzy/setup.bash && source ~/Documents/military_world/sar_ws/install/local_setup.bash && export ROS_DOMAIN_ID=72`

| step | command |
|---|---|
| world + rover + sensors (4K RGB, depth, segmentation, thermal, lidar, IMU) + controllers | `ros2 launch vigil_sar sim.launch.py` |
| terrain / cliff perception + A→B navigation | `ros2 launch vigil_sar navigation.launch.py` |
| thermal detection + human tracker + SAR manager | `ros2 launch vigil_sar sar.launch.py` |
| dashboard (http://localhost:8080) | `ros2 launch vigil_sar dashboard.launch.py` |

Instead of the dashboard buttons you can use topics:
```bash
ros2 topic pub --once /navigation/goal geometry_msgs/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 80.0, y: 40.0}}}"
ros2 topic pub --once /navigation/command std_msgs/String "data: START"
ros2 topic pub --once /sar/command std_msgs/String "data: START"     # SAR ON  (STOP = off)
ros2 topic echo /sar/humans        # detected human list (JSON)
ros2 topic echo /sar/events        # HUMAN Hn DETECTED, SEARCH POINT k COMPLETE, ...
```

## Stop / restart

```bash
Ctrl+C                                   # in the launch terminal
~/Documents/military_world/sar_ws/stop_sar.sh     # kills anything left over (only vigil_sar processes)
ros2 launch vigil_sar sar_mission.launch.py       # restart; leftover vigil_sar Gazebo servers are also
                                                  # stopped automatically (cleanup:=true), so a second rover never appears
```

## Offline tests (no ROS or Gazebo needed)

```bash
cd ~/Documents/military_world/sar_ws/src/vigil_sar
python3 test/test_sar_offline.py                  # ~8 min (includes a full simulated SAR mission)
python3 test/test_dynamic_obstacles.py            # ~20 s: collision geometry, tracker, prediction, 14 closed-loop
                                                  # scenarios (people standing / crossing / head-on, vehicle, building,
                                                  # narrow gap, cliff, hill, several obstacles, B stop, blocked gap),
                                                  # acceleration levels, simulation speed
```

## Topics

| topic | type | from → to |
|---|---|---|
| `/camera/image`, `/camera/depth_image`, `/camera/camera_info` | Image / CameraInfo | rgbd camera (unchanged, 320×240) → terrain_mapper, thermal detector |
| `/camera/hd/image` | Image rgb8 | **4K** display camera → overlay, dashboard 4K frame |
| `/camera/segmentation/labels_map` | Image | segmentation camera → terrain_mapper |
| `/thermal/image_raw`, `/thermal/camera_info` | Image mono16 (0.01 K/count) | **thermal camera** → thermal_human_detector |
| `/vision/terrain_classes`, `/vision/traversability`, `/vision/slope`, `/vision/roughness`, `/vision/terrain`, `/vision/cliff`, `/vision/overlay` | | terrain_mapper (unchanged topics) |
| `/vision/thermal_human`, `/vision/thermal_overlay` | String JSON / Image | thermal_human_detector |
| `/sar/humans`, `/sar/human_markers`, `/sar/events` | String JSON / MarkerArray | human_tracker |
| `/sar/state`, `/sar/search_points`, `/sar/command` | String | sar_manager ↔ dashboard |
| `/navigation/goal`, `/navigation/path`, `/navigation/previous_path`, `/navigation/status`, `/cmd_vel` | | cliff_navigator (unchanged topics) |
| `/navigation/hold`, `/navigation/speed_limit`, `/navigation/command` | Float64 / Float64 / String | NEW: scan heading, SAR speed cap, START |
| `/perception/obstacles` | String JSON | NEW: obstacle_tracker → cliff_navigator, terrain_mapper, dashboard (tracks: class, x, y, vx, vy, speed, direction, distance, dynamic) |
| `/navigation/status` → `collision`, `collision_status` | (fields) | NEW: CLEAR / COLLISION RISK / AVOIDING, time to collision, speed cap |
| `/drive/accel_command`, `/drive/status` | String / String JSON | NEW: ACCELERATION + (UP / DOWN / NORMAL) → drive.py; level, active limit, stability cap |

## What was changed and why

| FILE | CHANGE | REASON |
|---|---|---|
| `urdf/agri_ugv.urdf.xacro`, `cad_geometry.xacro`, `simulation.xacro` | copied; `vigil_rough_terrain` renamed to `vigil_sar`; one optional include added | same rover geometry, wheels, joints, suspension, controllers and sensors |
| `urdf/vision_sensors.xacro` | HD camera defaults changed to 3840×2160 at 5 Hz | 4K RGB requirement (set from `rgb_camera.*`) |
| `urdf/sar_sensors.xacro` | NEW thermal camera on a front mast (`thermal_camera` + `ThermalSensor`, L16); no `<noise>` element | primary SAR sensor; mount/FOV/resolution from `thermal_camera.*`. A `<noise>` element segfaults gz-sensors 8 - see the crash section |
| `meshes/*.stl`, `config/controllers.yaml` | copied unchanged | rover |
| `config/bridge.yaml` | copied, plus 4K, segmentation and thermal bridges | one bridge for every sensor |
| `config/sar_mission.yaml` | NEW single config (robot/camera/terrain/navigation values copied from `vision_nav.yaml`) | no hard-coded mission values |
| `scripts/terrain_core.py`, `planner_core.py` | copied **unchanged** | cliff detection, robot-size safety, A*, replanning, hill climbing |
| `scripts/drive.py` | same wheel maths; watchdog and ramp moved to the node (simulation) clock | at ~5 % real time the wall-clock watchdog zeroed the wheels between commands and the rover never moved |
| `scripts/ros_common.py` | `node_time()` | one clock for every freshness check |
| `scripts/ugv_dashboard.py`, `dashboard/index.html` | simulation speed (`sim NN% of real time`) in the header | the number that explains why everything feels slow |
| `launch/sim.launch.py` | `fast:=true` (no walking people, 1080p display camera; physics unchanged) | a faster demo without editing the config |
| `scripts/terrain_window.py` | NEW: runs the unchanged analysis only around the camera | 170 m map: full-map analysis took 1.1 s per frame; a test shows identical results |
| `scripts/terrain_mapper.py` | windowed mapper, grid rate throttle, 4K decoded only when drawn, overlay width | real-time on the large map and with 4K |
| `scripts/vision_overlay.py` | 16:9 crop and configurable output width | 4K is 16:9 while depth is 4:3 at the same horizontal FOV |
| `scripts/cliff_navigator.py` | scan hold (turns only if the turning circle is clear), speed limit, START/PAUSE, throttled costmap | SAR scans and speed cap without a second `/cmd_vel` publisher; manual START |
| `scripts/ros_common.py` | faster OccupancyGrid (de)serialisation, 16-bit images, new parameter sections | 2.9 M-cell grids, L16 thermal images |
| `scripts/thermal_core.py`, `thermal_human_detector.py` | NEW | temperature band, contrast, blobs, depth/ground range, size/aspect filters |
| `scripts/human_tracker.py` | NEW | N-frame confirmation, permanent IDs H1, H2…, no duplicate events |
| `scripts/sar_core.py`, `sar_manager.py` | NEW | 10 points, building views, scans, confirm-first, skip-on-timeout, completion |
| `scripts/ugv_dashboard.py`, `dashboard/index.html` | extended / rewritten in the same style | SAR switch, thermal panel, human list, H-markers, event log, START, SET GOAL B |
| `scripts/build_sar_world.py`, `sar_paths.py` | NEW | human heat from config, building casualties, physics step, labels |
| `scripts/build_sar_world.py` | replaces hull box collisions (fence, parapets, building interiors) with the model's own mesh, and checks the robot start is clear | the exported fence box was a solid 50 x 34 x 2 m block with the spawn inside it - the rover hung in mid air and could not reach B |
| `launch/*.py` | NEW (logic taken from `terrain_sim` / `vision_nav`) | the world is military_world, and exactly one rover spawns |
| `launch/sim.launch.py` | GUI killed the moment the server dies, GUI output to its own log, server line-buffered (`emulate_tty`) into `launch.log`, `debug:=true` runs it under gdb, sensors readable from a render profile | the "not responding / Force Quit" hang, and a crash that used to leave no evidence |
| `urdf/simulation.xacro` | rgbd camera and lidar wrapped in `rgbd:=` / `lidar:=` switches | the self-test has to be able to run the rover with no rendering sensor at all |
| `scripts/build_sar_world.py` | drops per-visual Thermal plugins whose temperature IS the ambient temperature (~390 of them) | identical thermal image (<1 K), far less load in the GUI, no warning flood |
| `find_crash.sh`, `scripts/find_crash.py` | NEW self-test: real launches, real subscribers, sensor and world bisect, writes `generated/render_profile.yaml` | finds and works around the Gazebo crash on this machine without guesswork |

**Not copied:** `worlds/rock_terrain.sdf`, `worlds/test/*`, `models/rocky_terrain_n4`, the four-wheel comparator
(`make_four_wheel.py`, `comparison_driver.py`, `comparison.launch.py`), and `build/`, `install/`, `log/`, `run/`, `docs/validation`.

## Human heat signature (how it works in Gazebo Harmonic)

A Gazebo thermal camera reads a temperature per **visual** from `gz::sim::systems::Thermal`. Any object
without that plugin renders at the world `<atmosphere>` temperature. The export already gave every visual a
temperature. `build_sar_world.py` rewrites the temperature only where the model has
`"sar_object": "HUMAN"`, using `human_thermal.*`:

- head 307.65 K
- torso 305.15 K
- limbs 303.65 K
- covered, low-contrast casualties 297.65 K
- `thermal_contrast` scales all of these

Nothing else is changed: concrete 293 K, rock 287 K, rubble 295 K, the warm-case decoy 314 K.
Every launch checks that **no non-human object falls inside the human band**.

Detection never uses RGB. For every thermal frame:

1. **Band.** Pixels between 296.5 and 311 K are kept. Engines, generators and warm cases are all hotter, so they are rejected.
2. **Two levels.** Blobs are formed separately for *core* pixels (≥ 301 K: skin, clothed torso) and *lukewarm* pixels.
   - A person standing next to a sun-warmed car therefore never merges with the car.
   - A lukewarm blob is discarded if it touches a person's core, and rejected if it touches an engine.
   - A lukewarm blob only counts as a *covered casualty* if it is ≤ 299.5 K, ≤ 1.9 m and ≤ 0.9 m².
3. **3-D range.** Range comes from the depth camera (up to 15 m). Beyond that, the lowest pixel of the blob is intersected with the ground plane.
4. **Size.** Metric size, area and aspect limits are applied (0.25–2.3 m).
5. **Tracker.** A blob becomes a human only after 4 consecutive *well-ranged* frames, meaning depth-ranged or ground-ranged within 12 m. Farther blobs stay as candidates: the robot turns towards them but never places a marker from them.

The offline tests sweep 240 robot poses around a 335 K engine, a 314 K warm case, a 300 K car body and a
0.15 m warm pipe, and none of them produces a confirmable false human.

## Parameters

| parameter | value | key in `sar_mission.yaml` |
|---|---|---|
| RGB resolution | 3840 × 2160 | `rgb_camera.width/height` |
| RGB FPS | 5 Hz (4K = 24.9 MB/frame) | `rgb_camera.fps` |
| overlay / dashboard width | 1920 px | `rgb_camera.overlay_width` |
| thermal resolution | 320 × 240, HFOV 60° | `thermal_camera.resolution_*`, `horizontal_fov` |
| thermal FPS / processing | 10 Hz / 5 Hz | `thermal_camera.update_rate`, `human_detection.processing_rate` |
| thermal mount | 1.00 m above ground, 0.52 m forward, pitch 0 | `thermal_camera.camera_height/mount_x/camera_pitch` |
| thermal detection range | 30 m | `thermal_camera.detection_range` |
| human temperature band | 296.5 – 311.0 K (23.4 – 37.9 °C) | `thermal_camera.temperature_threshold`, `human_max_temperature` |
| min contrast vs background | 1.5 K | `human_detection.min_contrast_k` |
| human size | 0.25 – 2.3 m, aspect ≤ 6, area 0.04 – 2 m² | `minimum_human_size`, `max_human_size`, … |
| human core / covered limits | ≥ 301 K core; covered ≤ 299.5 K, ≤ 1.9 m, ≤ 0.9 m² | `human_detection.min_peak_temperature`, `covered_*` |
| well-ranged for confirmation | depth, or ground-plane ≤ 12 m | `human_detection.confirm_max_range_no_depth` |
| same-human merge distance | 1.5 m | `human_detection.merge_distance`, `sar.revisit_distance` |
| human confirmation frames | 4 | `sar.human_confirmation_frames` |
| human body / head / limb temperature | 305.15 / 307.65 / 303.65 K | `human_thermal.*` |
| search radius (square around B) | 30 m | `sar.search_radius` |
| number of search points | 10 (2 rows × 5) | `sar.search_points`, `search_rows` |
| search speed / near buildings | 0.45 / 0.30 m/s | `sar.search_speed`, `building_scan_speed` |
| search scan angles | 8 (45° steps), 1.5 s each | `sar.scan_angles`, `scan_dwell` |
| building search distance | 25 m (views at 7 m, 3.5 m in narrow streets) | `sar.building_search_distance`, `building_standoff*` |
| SAR timeout per point / whole mission | 300 s / 2400 s | `sar.search_timeout`, `max_search_time` |
| max safe slope / climbable slope | 25° / 33° | `terrain.max_safe_slope_deg`, `max_climb_slope_deg` |
| max safe drop (cliff) | 0.17 m | `terrain.max_safe_drop` |
| max climbable step | 0.23 m | `terrain.max_climb_step` |
| robot footprint / safety margin | 1.53 × 1.12 m / 0.15 m | `robot.*`, `terrain.safety_margin` |
| cruise speed | 0.60 m/s | `navigation.cruise_speed` |
| map | 170 × 170 m at 0.10 m, origin (−40, −30) | `terrain.map_*` |
| physics step | 1 ms (rover springs validated at 1 ms) | `world.physics_step` |

## The "Force Quit" crash: cause and fix (2026-09-23)

**Symptom.** The Gazebo server died with `exit code -11` about 5 s after the rover spawned; the GUI
was left without a server, froze, and the desktop offered **Force Quit**. Every vigil_sar node had
started normally.

**Cause** (from the gdb backtrace produced by `debug:=true`):

```
[Err] [Noise.cc:63] Image noise requested. Please use ImageNoiseFactory::NoiseModel instead
Thread 8 "ruby" received signal SIGSEGV
#0  libgz-sensors8-thermal_camera.so.8
#1  gz::sensors::ThermalCameraSensor::SetScene(...)
#2  gz::sim::systems::Sensors::CreateSensor(...)
#4  gz::sim::RenderUtil::Update()
```

`ThermalCameraSensor::CreateCamera()` in gz-sensors 8 (the version Jazzy ships) builds the camera's
noise model with `NoiseFactory` instead of `ImageNoiseFactory`. That factory refuses image noise,
prints the `[Err]` line above and returns a **null** pointer, which the next line dereferences
(`dynamic_pointer_cast<ImageGaussianNoiseModel>(null)->SetCamera(...)`). Any `<noise>` element on a
`thermal_camera` triggers it - **including `stddev 0`**, which is what our sensor had. It happens
while the rover's sensors are created, which is why it always struck ~5 s after the spawn, on the
Intel iGPU and on the NVIDIA GPU alike, and never in the rig tests (those thermal cameras had no
`<noise>` element). Upstream fix: gz-sensors PR #597 (March 2026), newer than Jazzy's 8.x.

**Fix in this workspace**

| FILE | CHANGE | REASON |
|---|---|---|
| `urdf/sar_sensors.xacro` | the `<noise>` element is gone from the thermal sensor, behind `thermal_noise_sdf:=true` (default false) | removes the null-pointer path: the server no longer crashes |
| `scripts/thermal_human_detector.py`, `config` key `thermal_camera.noise_in_sdf` | `noise_stddev` K of gaussian noise is added to the thermal image in software | same noise, same units, no Gazebo bug |
| `test/test_sar_offline.py` | `ThermalSensorSdfTests` | the `<noise>` element can never come back unnoticed |
| `launch/sim.launch.py` | GUI is SIGKILLed the instant the server exits; GUI output to its own log; server line-buffered into `launch.log`; `debug:=true` runs it under gdb | a dead server can no longer leave a frozen window - no Force Quit dialog, shutdown in ~2 s |
| `scripts/build_sar_world.py`, config key `world.drop_ambient_thermal_plugins` | drops the 387 per-visual Thermal plugins that only repeated the ambient 293.15 K (370 kept, humans untouched) | the GUI loads every visual plugin as a system on its Qt main thread; that is what made it stop answering while loading |

**If Gazebo ever crashes again on this or another PC**

```bash
ros2 launch vigil_sar sar_mission.launch.py gui:=false debug:=true   # backtrace in the terminal
sudo apt install gdb                                                 # needed for that
cd ~/Documents/military_world/sar_ws && bash find_crash.sh           # ~10-15 min, fully automatic
```

`find_crash.sh` rebuilds the workspace and runs the *real* launch headless, one sensor set at a
time, with the bridge subscribed so the cameras genuinely render (a Gazebo camera nobody subscribes
to never renders - which is why the older `diagnose_sar.sh` / `gpu_test.sh` rigs all passed):

| phase | what it answers |
|---|---|
| 1 | all sensors under gdb → the crash backtrace |
| 2 | rover with **no** rendering sensor → camera problem, or world/physics? |
| 3 | rgbd / lidar / segmentation / 4K / thermal one at a time (4K retried at 1080p) |
| 4 | everything that survived alone, together → the set the mission will use |
| 5 | world-content bisect (buildings / scatter / props / vehicles / base / people / decoys) |

It writes `sar_ws/diagnosis/find_crash.log`, `find_crash.json` and `sar_ws/generated/render_profile.yaml`;
**every later launch reads that profile by itself**, so the normal command then runs with the sensors
this PC can render. Manual switches (they override the profile, `profile:=none` ignores it):

```bash
ros2 launch vigil_sar sar_mission.launch.py thermal:=false       # no thermal camera (SAR needs it)
ros2 launch vigil_sar sar_mission.launch.py segmentation:=false  # no segmentation camera
ros2 launch vigil_sar sar_mission.launch.py hd_camera:=false     # no 4K camera
ros2 launch vigil_sar sar_mission.launch.py hd_width:=1920 hd_height:=1080
ros2 launch vigil_sar sar_mission.launch.py rgbd:=false lidar:=false   # no camera at all
ros2 launch vigil_sar sar_mission.launch.py gpu:=intel           # gpu:=nvidia is the default when present
ros2 launch vigil_sar sar_mission.launch.py headless:=true       # server renders with EGL
ros2 launch vigil_sar sar_mission.launch.py gui:=false           # dashboard only, no Gazebo window
```

While a 700-mesh world loads, GNOME may still show "not responding" for a few seconds: click
**Wait**, or raise the limit with `gsettings set org.gnome.mutter check-alive-timeout 60000`.

## Rover hangs in mid air / cannot reach B (2026-09-23, fixed)

First run after the crash fix: Gazebo stayed up, but the rover floated at spawn height, the
dashboard showed `TERRAIN HIGH CLIFF  DROP 0.66 m  SAFE 0.0 m`, and no path to B existed.

**Cause.** The export gives every model one box collision sized from its mesh bounding box. For
`SAR_Base_Fence` - the perimeter fence of the home base - that box is a **solid 50 x 34 x 2 m
block** centred on (0, -16), and the robot start (0, -8) is *inside* it. The rover spawned inside an
invisible block and was held there; the depth camera then saw the real ground 0.66 m lower and
called it a high cliff, and the whole base was one obstacle, so the planner had nowhere to go.

**Fix** (`build_sar_world.py`, `world.fix_hull_collisions`): a box collision that covers the robot
start, or that is a big hull box (`world.hull_collision_area`, 60 m², both sides > 2 m) around a
`WALL` / `FENCE` / `STRUCTURE` model, is replaced by that model's **own mesh** as the collision -
exact posts, rails and the 12.4 m north gate the rover drives out through. Ten models are fixed:
the base fence, both bridge parapets, five building interiors and two collapsed walls. Everything
else - visuals, poses, terrain, slabs, crates, vehicles, rocks - keeps the exported collisions.
The spawn height also went from 0.80 m to 0.30 m (the apron is at 0.08 m), so the rover settles
instead of free-falling onto its springs. `build_sar_world.py` prints every fix and warns if
anything still stands on the robot start; `test_sar_offline.py` checks both.

**After it settles, press START A→B** on the dashboard (or `ros2 topic pub --once
/navigation/command std_msgs/String "data: START"`). `READY - PRESS START` means the navigator has
a goal but no START yet - by design, the robot never drives off on its own.

## Rover has a path but does not move (2026-09-23, fixed)

Second run: the rover stood on the ground, `TERRAIN SAFE`, `NAVIGATING`, `initial path to B: 93.0 m`
- and `SPEED 0.00 m/s` for minutes. Both controllers were active, so the wheels were simply never
commanded.

**Cause: wall-clock timing in a simulation that runs at ~5 % of real time.** `drive.py` (copied from
vigil_rough_terrain, where the world ran near real time) used `time.monotonic()` for its cmd_vel
watchdog and ramped the wheel speed one step per timer tick. In military_world the navigator's
10 Hz command stream arrives roughly once every **two seconds of wall clock**, so the 0.6 s watchdog
fired between every pair of commands and reset the wheels to zero, and the ramp needed ~25 s of wall
clock to reach cruise speed. The rover twitched at most.

**Fix.** `ros_common.node_time(node)` returns the node clock (simulation time, since every vigil_sar
node runs with `use_sim_time`). `drive.py` now uses it for the watchdog and ramps by elapsed
simulation time (0.5 units/s - the same acceleration as before at real-time speed). The same
wall-clock trap was fixed in `thermal_human_detector.py` (thermal frame age 2 s, depth age 1 s -
with the wall clock the depth image was *always* considered stale, so ranges fell back to the
ground-plane estimate) and in `terrain_mapper.py` (4K frame age, grid publish rate).
`test_sar_offline.py::NodeClockTests` guards all of it.

### Simulation speed

The dashboard header shows `sim NN% of real time`; Gazebo shows the same number as RTF (bottom
right). All speeds (3.0 m/s, walking people) are in *simulation* time, so below 100 % everything
looks slow on screen.

**Measured on the author's PC** (`bash measure_physics.sh`, rover standing, no GUI):

| world variant | real time |
|---|---|
| full world as exported | 6.5 % |
| people frozen | 17 % |
| no collisions on objects (terrain + rover only) | 51 % |
| ground only | 100 % |

Sensors (4K, thermal, segmentation) changed it by only a few percent (`measure_speed.sh`). The cost
is the physics engine checking ~3,100 collision shapes (mostly small vegetation, rock and rubble
boxes) and the walkers' collisions every millisecond.

**Fix (24 Sep 2026, `build_sar_world.py` step 9, set in `config/sar_mission.yaml`):**

| key | default | effect |
|---|---|---|
| `world.collision_zone_only` | `true` | drop object collisions outside the operational zone |
| `world.collision_zone_margin` | `5.0` | ... plus this margin (m) |
| `world.collision_max_bottom` | `1.3` | drop collisions whose bottom is higher than this (m): the rover cannot touch them |
| `world.walker_collisions` | `true` (was `false`) | walking people collide with the rover (primitive body, force-driven contact controller; see [Dynamic obstacles](#dynamic-obstacles-collisions-acceleration-and-simulation-speed)) |

The launch prints the result, e.g. `collision shapes: 955 kept; removed 1953 outside the zone, 81 out
of the rover's reach, 106 of walkers`. Visuals, thermal and depth images and the rover's physics
(1 ms step, springs, limits) are unchanged. **The resulting real-time factor has not been measured
yet.** Run `bash measure_physics.sh` and check `diagnosis/measure_physics.log`.

Other options:

```bash
ros2 launch vigil_sar sar_mission.launch.py fast:=true        # no walking people, 1080p display camera (physics unchanged)
ros2 launch vigil_sar sar_mission.launch.py segmentation:=false
```

## Known limits: check these on your machine

- **First real launch reached sensor creation on 2026-09-23** and exposed the gz-sensors thermal-noise segfault, now fixed (see the crash section). The rest was developed in a cloud container without ROS or Gazebo. Everything was tested offline instead: detection on ray-cast thermal images, the full SAR mission, the world builder, xacro processing and the ported cores. The first real launch should be checked for:
  - `/thermal/image_raw` arriving as `mono16` (`ros2 topic echo /thermal/image_raw --field encoding`)
  - human pixels near 30500 (305 K)
- **Real-time factor.** The full world runs below real time; see [Simulation speed](#simulation-speed). The 4× target is what Gazebo *tries*; the dashboard shows what the PC achieves.
- **Dynamic obstacles are not yet checked in Gazebo.** They are tested offline, including rendered depth and segmentation images and closed-loop runs. The first Gazebo run should confirm three things:
  - the segmentation labels arrive (`/perception/obstacles` → `"segmentation": true`)
  - walkers are tracked as HUMAN with a velocity
  - `dynamic_check.py` reports no person closer than the safety distance
- **People do not avoid the rover.** They follow their route. With the contact controller, a person who walks into a stopped rover is stopped by the contact (and pushes with at most 600 N). The rover gives way first, because it plans round every person.
- **The contact people cost more physics time** than kinematic ones: 16 bodies under gravity on the terrain. `measure_physics.sh` shows the effect. `people_controller: 1` brings back the old kinematic walkers, but with them the rover and people can overlap again.
- **Pose source.** Pose comes from `/sim/ground_truth`, exactly as in vigil_rough_terrain; there is no SLAM.
- **Building footprints** come from `sar_metadata.json`. That is mission-planning information, not ground truth: the casualty ground truth is never used at runtime.
- **Walls block thermal.** Casualties inside buildings are found through doors and openings (LWIR does not see through walls). Humans placed deep inside, out of line of sight, are not detected. That is physically correct.
- **Walking people also count.** They are real humans with heat signatures, so they are detected too. A person seen again at a different spot more than 1.5 m away may get a new ID.
- **Missing baseline person.** `SAR_StaticPerson_001`, the open-field baseline person, exists in the ground truth but is missing from the exported SDF. Re-export in Blender to restore it.
