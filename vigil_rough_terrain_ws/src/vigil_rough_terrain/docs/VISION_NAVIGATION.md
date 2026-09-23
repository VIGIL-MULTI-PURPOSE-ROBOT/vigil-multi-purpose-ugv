# Vision cliff detection + A-to-B navigation + dashboard

This adds depth-based terrain/cliff detection, cliff-aware replanning and a web
dashboard to the existing VIGIL rough-terrain simulation. Nothing that existed
before was removed. `drive.py`, `comparison_driver.py`, `comparison.launch.py`,
the robot model and the rock terrain behave exactly as before unless you use
the new launch file.

## Architecture

```
Gazebo rgbd_camera ──/camera/depth_image, /camera/image, /camera/camera_info──┐
(opt.) segmentation ─/camera/segmentation/labels_map─────────────────────────┤
OdometryPublisher ───/sim/ground_truth (base_footprint pose)─────────────────┤
                                                                              ▼
                     terrain_mapper.py  (terrain_core.py + vision_overlay.py)
                       depth → world points → 2.5-D elevation map (0.1 m cells)
                       edge scan: ground that "vanishes" → CLIFF / POSSIBLE DROP
                       step / drop / slope / roughness → class per cell
   /vision/terrain_classes  /vision/traversability  /vision/terrain  /vision/cliff  /vision/overlay
                                   │
                                   ▼
                     cliff_navigator.py  (planner_core.py)
                       footprint-inflated A* (0.2 m grid) + footprint sweep check
                       replans when new terrain blocks the path ahead
   /navigation/path  /navigation/previous_path  /navigation/status  ──►  /cmd_vel
                                                                           │
                                          drive.py (unchanged) ◄───────────┘
                                          → /wheel_controller/commands (8 wheels)

                     ugv_dashboard.py  →  http://localhost:8080
```

## Terrain classes

| Class | Meaning | Planner |
|---|---|---|
| SAFE | observed, gentle | cost 1 |
| UNEVEN | slope > 10°, roughness > 3 cm or step > 6 cm, still within limits | +2 per cell |
| POSSIBLE DROP | ground disappears behind an edge but the camera can't yet prove it is steeper than 25° | +25 per cell (not blocked) |
| STEEP | footprint-scale slope > 25° | robot centre may not enter |
| OBSTACLE | step up > 0.17 m (boulder, wall) | inflated by robot size |
| HIGH CLIFF | drop > 0.17 m, confirmed steeper than 25° | inflated by robot size |

## How a cliff is detected (depth geometry, not colour)

1. Each depth pixel is back-projected with the camera intrinsics and the fixed
   camera mount from the URDF. It is then moved into the world frame with the
   robot pose from `/sim/ground_truth`.
2. **Edge scan**: each image column is walked from near to far. The last point
   on real ground is the reference. If the next ground point is lower by more
   than `max_safe_drop`, the ground has "vanished". The same happens if a
   downward ray finds nothing within 15 m where flat ground would be within 6 m.
3. A camera 0.65 m above the ground can never see a cliff face. It sees the top
   edge, a gap, then lower ground or nothing. Two lower bounds give the steepness
   of the hidden part:
   - it lies under the grazing ray, so it is at least as steep as that ray's angle θ
   - it loses Δz over a horizontal gap Δh, so somewhere it is at least atan(Δz/Δh) steep (mean-value theorem)

   If either bound is 25° or more, the edge is a **HIGH CLIFF** (lethal).
   Otherwise it is a **POSSIBLE DROP** (expensive but allowed). From a distance,
   a real cliff and a legal 20° downslope look identical. As the rover gets
   closer, θ grows until the edge is either confirmed or seen to be a normal
   slope.
4. **Map-space checks** on observed cells:
   - local drop/step between neighbouring cells, allowing for a 25° slope
   - slope from a plane fit over the robot's own contact patch
   - "elevated" and "depressed" tests, so a boulder top is labelled OBSTACLE rather than cliff
5. Cells the rover has physically stood on are known SAFE. The camera can't see
   the ground under the robot; its lowest ray reaches the ground 1.11 m ahead.

## Where every threshold comes from (config/vision_nav.yaml)

| Parameter | Value | Derivation |
|---|---|---|
| robot.length | 1.53 m | front of wheel-1 tyre (+0.785) to rear of wheel-4 tyre (−0.746) |
| robot.width | 1.12 m | outer tyre faces: wheel-1 centre y 0.466 + half tyre 0.093 |
| robot.wheelbase | 0.82 m | ground-bearing wheels 1→3 (wheel 4 is 0.33 m above flat ground) |
| robot.ground_clearance | 0.3375 m | measured, docs/validation/flat.json |
| camera.z / pitch | 0.599 m / 10° down | camera_mount on base_link, which is 0.5872 m above base_footprint |
| terrain.max_safe_drop / max_safe_step | 0.17 m | smallest wheel radius 0.1766 m: a bigger edge makes the wheel fall or strike instead of roll. Clearance (0.3375) is larger, so the wheel is the limit |
| terrain.max_safe_slope_deg | 25° | successful traverse peaked at 35° tilt; the existing driver trips at 38°. Margin covers side-slope roll |
| terrain.slope_window | 0.9 m | max(wheelbase 0.82, track 0.90): body tilt follows the plane through the wheels |
| terrain.safety_margin | 0.15 m | clearance kept between footprint and hazard |
| straight-line inflation | 0.71 m | width/2 + margin |
| turn-in-place clearance | 1.10 m | half-diagonal of footprint (0.95) + margin |
| navigation.cruise_speed / max_angular / turn_in_place_error / goal_tolerance | 0.18 / 0.35 / 0.65 / 0.30 | same as comparison_driver.py |

## Robot-size rules in the planner

- The body must never overlap a CLIFF/OBSTACLE edge. A* cells closer than
  width/2 to an edge are forbidden, and cells inside the margin band only near
  A or B.
- Every candidate path is swept with the **oriented footprint rectangle**
  (length × width + margin) along its heading, so the corners are checked too.
- Rotating in place is only allowed when the circumscribed circle is clear.
  Otherwise the rover arcs forward slowly or stops.
- Emergency stop if a CLIFF/OBSTACLE lies within 0.45 m ahead of the front bumper.
- Unobserved ground is assumed drivable at a small extra cost, the usual
  "optimistic" planning. So that an unreachable B ends as NO SAFE PATH instead
  of endless exploration, the search is bounded:
  - planning stays within 6 m of the A–B line
  - a route longer than 3× the direct distance (+3 m) counts as no route
  - at most 30 cliff replans per goal

## Navigation states

`WAITING` (6 s settle) → `NAVIGATING` → `CLIFF DETECTED` / `STEEP DETECTED` / `OBSTACLE DETECTED`
→ `REPLANNING` → `SAFE PATH FOUND` → … → `GOAL REACHED`, or `NO SAFE PATH`
(stopped; replanning continues in case new data opens a route), or
`EMERGENCY STOP` (tilt > 38°).

## Verification done

`test/closed_loop_sim.py` ray-casts synthetic depth images from the same
geometry as the Gazebo test worlds. It feeds them through the real
`terrain_core`/`planner_core` code, moves the robot with the commanded
velocity, and takes pitch/roll from its 6 wheel contacts. Checks include
"robot body never over a drop".

| Scenario | Result | Behaviour |
|---|---|---|
| flat | GOAL REACHED | straight, no replans |
| small_rocks (4–12 cm) | GOAL REACHED | straight, rocks not avoided |
| moderate_slope (15°) | GOAL REACHED | drove up it |
| small_drop (12 cm) | GOAL REACHED | drove off it |
| cliff_front (1.5 m pit on the line) | GOAL REACHED | detoured ~3 m sideways |
| cliff_left (edge 1.4 m to the side) | GOAL REACHED | kept going straight |
| cliff_right (edge cuts into route) | GOAL REACHED | shifted left |
| narrow_route (2.0 m bridge) | GOAL REACHED | found and crossed the bridge |
| too_narrow (1.2 m bridge < 1.42 m needed) | NO SAFE PATH | refused, stopped |
| no_route (full-width chasm) | NO SAFE PATH | stopped before the edge |
| real rock terrain (25 × 25 m version), spawn → B | GOAL REACHED | |

The ROS nodes were also run end-to-end with a stand-in rclpy (no ROS was
available in the build sandbox). **They have not yet been run in real Gazebo.**
Watch the first run for encoding or QoS differences.

## Known limits (honest)

- A cliff seen only side-on and more than ~1.4 m away stays POSSIBLE DROP
  (orange), not red. For example, the cliff_left edge the rover passes 1.4 m
  from. The geometry can't tell it apart from a legal slope. It is still
  avoided if the path ever needs to go near it.
- Pose comes from simulation ground truth (`/sim/ground_truth`). On real
  hardware, replace it with your localisation. It's the only pose input.
- The map is a fixed 32 × 32 m world grid, sized for this terrain. Increase
  `terrain.map_size` for bigger worlds.
- Thresholds are derived from geometry. They still need tuning in Gazebo with
  real dynamics, especially `max_safe_drop` (try 0.12–0.22 in the small_drop world).

## Adding YOLO later

`terrain_mapper.py` already receives RGB and segmentation aligned pixel-for-pixel
with depth. A detector node can publish a label image, and
`TerrainMapper.process(seg=...)` can mark those pixels' cells as OBSTACLE.
Depth stays the source of geometry.


---

## Update: goal-seeking navigation (no more "NO SAFE PATH → stop")

**Cause of the stop** (in the old `scripts/planner_core.py`, `NavigatorCore._plan`): after 3 failed
A* searches the navigator set `NO SAFE PATH`, cleared the path and returned `(0, 0)` forever. It also
stopped on tilt > 38° (`EMERGENCY STOP`), treated long detours / many replans as "no route", and
treated every slope > 25° (STEEP) as forbidden.

### Terrain: hills are cost, cliffs are obstacles

| Class | Rule | Planner |
|---|---|---|
| UNEVEN | slope > 10°, roughness > 3 cm, step > 6 cm | cost |
| **CLIMBABLE** (new) | slope 25°–33°, or step 0.17–0.23 m | cost, **not inflated** |
| STEEP | slope > 33° (`max_climb_slope_deg`) | robot centre avoids it; allowed at high cost in the climbing fallback |
| OBSTACLE | step > 0.23 m (`max_climb_step`) | hard, inflated |
| HIGH CLIFF | drop > 0.17 m (`max_safe_drop`) and hidden face steeper than 33° | hard, inflated by footprint + margin |

On top of the class, every cell also costs `slope_cost·(slope/33°)²` and `roughness_cost·(roughness/3 cm)`.
Roughness is now measured **around the local plane**. Before this fix, a smooth hill counted as rough.
A 28° hill costs about 2.2× flat ground, so a detour is only taken if it is shorter than that.

### Escalation instead of stopping

1. Local replan inside the A–B corridor.
2. Replan over the whole map.
3. **Hill-climbing replan**: STEEP allowed at `steep_climb_cost`; cliffs stay hard → state **CLIMBING**.
4. **CLIFF_AVOIDANCE**: score 24 short headings on progress toward B (×3), slope, revisits
   and turning, then drive the best one. This repeats, with occasional full replans.
5. **RECOVERY**: reverse 0.8 m over non-hazard ground, turn toward the best heading, then replan.
   Triggered by: stuck (driving but no motion for 10 s), a hazard in front for 4 s, or tilt ≥ 38°.

The mission ends only at **GOAL_REACHED**. The rover never drives into a HIGH CLIFF / OBSTACLE.

### Climbing

- Speed scales down from 15° to 0.45× cruise at 33° (tilt or slope just ahead).
- Tilt ≥ 33° (`max_tilt − tilt_margin`): the spot ahead gets a cost penalty and the rover
  replans another climb line.
- Tilt ≥ 38°: RECOVERY.

### States

WAITING, NAVIGATING, REPLANNING (reason: HILL AHEAD / CLIFF AHEAD / PATH BLOCKED / TILT LIMIT),
CLIMBING, CLIFF_AVOIDANCE, RECOVERY, GOAL_REACHED.

### New map topics

`/vision/slope` (degrees) and `/vision/roughness` (cm), both OccupancyGrid, published by
terrain_mapper and used by cliff_navigator.

### Test results (offline closed loop)

| Scenario | Result |
|---|---|
| flat, small_rocks, small_drop, cliff_left | GOAL_REACHED, straight |
| moderate_slope (15°) | GOAL_REACHED, CLIMBING shown |
| **steep_hill (28°, whole width)** | **climbed straight over it**, GOAL_REACHED, tilt 28° |
| **hill_choice (38° face, 22° flanks)** | climbed a 22° flank, never the 38° face, GOAL_REACHED |
| **big_rocks (15–21 cm)** | GOAL_REACHED; rocks classified as cost, never OBSTACLE |
| cliff_front, cliff_right, narrow_route | GOAL_REACHED around the cliffs |
| too_narrow (1.2 m bridge), no_route | B is physically unreachable without crossing a cliff: the rover **keeps searching and never stops, never falls** |

### Terrain resized to 20 × 20 m

`models/rocky_terrain_n4/model.sdf` mesh scale `2.5 2.5 4` → `2.0 2.0 3.2`: length, width and
elevation all ×0.8, so slopes keep their angles (the tallest point drops from 4.57 m to 3.66 m).
A, B and the comparison course moved with the terrain (×0.8): spawn (−6.0, −4.0, 2.1),
B (−3.36, −2.4). Mesh files unchanged; `terrain_conversion.json` updated.

After the resize, B sits 0.85 m from a real 0.3 m ledge (58° face) in the terrain. Heading
straight at B, the footprint plus margin would overhang the ledge. So when B is within 1.10 m
of a cliff/obstacle edge, reaching the closest safe point within `goal_hazard_tolerance`
(1.0 m) counts as GOAL_REACHED. Offline result: GOAL_REACHED 0.37 m from B, no stop, no drop.

### Terrain resized to 18 × 18 m and faster driving

- Mesh scale `1.8 1.8 2.88`: 0.72 of the original 25 × 25 m in length, width **and elevation**.
  Every cliff/ledge is now 0.72× as high (tallest point 3.29 m); slopes keep their angles.
- A (−5.4, −3.6), B (−3.024, −2.16), spawn z 1.9; comparison course and traverse route ×0.72.
- Speed: `cruise_speed` 0.18 → **0.40 m/s**, `max_angular` 0.35 → 0.50 rad/s, `lookahead` 1.0 m,
  `stop_distance` 0.45 → 0.70 m (stopping 0.16 m at drive.py's 0.5 m/s² ramp + ≤0.3 s perception
  latency). On hills the speed still drops to 0.45× cruise at the 33° climbing limit (0.18 m/s).
  The two-rover comparison demo keeps its own 0.18 m/s so its results stay comparable.

### Elevation halved

Mesh scale `1.8 1.8 1.44`: still 18 × 18 m, height halved (tallest point 1.65 m). Cliffs, ledges and
hills are half as high and slopes gentler. Spawn z lowered to 1.2 m (0.5 m above the ground at A).

### Robot not moving (fixed), faster driving, HD camera

**Robot not moving:** a Gazebo server from an earlier launch was still running. Its ROS
`/controller_manager` (same ROS domain) answered the new launch's spawner ("can not be configured
from 'active' state"), so the new rover's wheel controllers never started. `vision_nav.launch.py`
now stops every process left over from earlier launches of this package at start-up (identified
by `GZ_PARTITION=vigil...` in their environment; nothing else is touched). Disable with `cleanup:=false`.

**Speed:** `cruise_speed` 0.60 m/s, `max_angular` 0.60 rad/s (drive.py limits 0.7 / 0.6),
`lookahead` 1.3 m, `stop_distance` 1.0 m (0.36 m braking at 0.5 m/s² + 0.18 m latency).
Offline: all scenarios still pass; flat run 16.7 s (was 39.4 s at 0.18 m/s).

**Camera:** new HD display camera (`hd_camera`, `/camera/hd/image`) on the same mount, FOV and 4:3
aspect as the rgbd camera; default 1920×1440 @ 10 Hz; the dashboard overlay is drawn at its full
resolution (≈31 ms/frame). Launch args `camera_width`, `camera_height`, `camera_rate`.
8K (`camera_width:=7680 camera_height:=5760 camera_rate:=2`) works but each frame is ~130 MB,
so expect the simulation to slow down. Cliff detection keeps using the 320×240 depth image.
