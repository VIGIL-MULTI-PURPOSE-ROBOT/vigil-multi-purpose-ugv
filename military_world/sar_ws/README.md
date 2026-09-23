# VIGIL SAR UGV: search and rescue in military_world

ROS 2 **Jazzy** + Gazebo **Harmonic** workspace inside `military_world/sar_ws`.

It reuses the VIGIL eight-wheel rover and its full perception and navigation stack from
`~/Documents/robot/vigil_rough_terrain_ws`. That workspace was only read, never written.
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
        └── test/test_sar_offline.py
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

## Build

```bash
cd ~/Documents/military_world/sar_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_sar --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_DOMAIN_ID=72          # different from vigil_rough_terrain (71) so the two never mix
chmod +x src/vigil_sar/scripts/*.py src/vigil_sar/test/*.py stop_sar.sh
```

The walking people need the `sar::WaypointSystem` plugin in `military_world/build/`, which is already
built there. If it is ever missing, build it once with `cd ~/Documents/military_world && bash run_gazebo.sh --check`.

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
| `launch/sim.launch.py` | `fast:=true` (4 ms step, no walkers) | a usable demo speed without editing the config |
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

The dashboard header now shows `sim NN% of real time`. At ~5 %, 93 m of driving is ~50 minutes of
wall clock. To trade accuracy for speed:

```bash
ros2 launch vigil_sar sar_mission.launch.py fast:=true        # 4 ms physics step, no walking people
ros2 launch vigil_sar sar_mission.launch.py hd_width:=1920 hd_height:=1080   # 4K is 25 MB per frame
ros2 launch vigil_sar sar_mission.launch.py segmentation:=false
```

or, permanently, in `config/sar_mission.yaml`: `world.physics_step: 0.004`,
`world.moving_people: false`, `rgb_camera.width/height`, `thermal_camera.update_rate`.

## Known limits: check these on your machine

- **First real launch reached sensor creation on 2026-09-23** and exposed the gz-sensors thermal-noise segfault, now fixed (see the crash section). The rest was developed in a cloud container without ROS or Gazebo. Everything was tested offline instead: detection on ray-cast thermal images, the full SAR mission, the world builder, xacro processing and the ported cores. The first real launch should be checked for:
  - `/thermal/image_raw` arriving as `mono16` (`ros2 topic echo /thermal/image_raw --field encoding`)
  - human pixels near 30500 (305 K)
- **Real-time factor.** A 1 ms step in a 300 m world with 20 walking people, plus a 4K camera, will run below real time. Faster options:
  - `world.physics_step: 0.002`
  - 1920×1080 RGB
  - `world.moving_people: false`
- **Pose source.** Pose comes from `/sim/ground_truth`, exactly as in vigil_rough_terrain; there is no SLAM.
- **Building footprints** come from `sar_metadata.json`. That is mission-planning information, not ground truth: the casualty ground truth is never used at runtime.
- **Walls block thermal.** Casualties inside buildings are found through doors and openings (LWIR does not see through walls). Humans placed deep inside, out of line of sight, are not detected. That is physically correct.
- **Walking people also count.** They are real humans with heat signatures, so they are detected too. A person seen again at a different spot more than 1.5 m away may get a new ID.
- **Missing baseline person.** `SAR_StaticPerson_001`, the open-field baseline person, exists in the ground truth but is missing from the exported SDF. Re-export in Blender to restore it.
