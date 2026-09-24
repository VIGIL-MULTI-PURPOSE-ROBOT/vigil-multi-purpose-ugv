# Agriculture: whole-field mission, obstacle avoidance, soil moisture, perception, dashboard

Everything lives in the `agri_ugv` package of this workspace, and all settings are in one file,
**`src/agri_ugv/config/agriculture.yaml`**.

## Run

```bash
cd ros2_ws
chmod +x src/agri_ugv/scripts/*.py            # once, after new scripts were added
./run.sh                                      # builds, then starts Gazebo + RViz + dashboard
# or: vigil -> 2 Agriculture   (build first: colcon build --symlink-install)
# open http://localhost:8080 and press START ROBOT
```

| Launch option | Effect |
|---|---|
| `row_count:=0` (default) | every crop row of the field; the count comes from the world model |
| `row_count:=N` | only the first N rows (for tests) |
| `dashboard:=false` | no perception, no dashboard, no START gate (the robot starts by itself) |
| `autostart:=true` | dashboard, but the mission starts without pressing START |
| `moisture:=false` | no moisture sensor or map |
| `autonomy:=false` | no autonomy supervisor (the mission runs exactly as before) |

## 1. Whole-field row mission (`crop_row_driver.py`)

- **Number of rows:** set by `field.auto_row_detection`.
  - `world`: read from the `row_XX_seg` links of `models/vigil_cotton_farm/model.sdf`. For the supplied field this gives **23 rows**, 1.219 m apart.
  - `dimensions`: `round(field_width / row_spacing) + 1`.
  - `fixed`: `total_rows`.
- **Pattern:** boustrophedon. Row 1 is driven east, then a headland U-turn, then row 2 west, and so on until the last row, then **FIELD COMPLETE**. There is no hard-coded row number anywhere.
- **Row completion:** a row is complete only when the robot centre has passed the last plant (`crop_area_start` / `crop_area_end`) by `row_end_tolerance`. It is not based on a fixed distance.
- **Unchanged from before:** the straddle drive, the row-centring control law, 1.2 m/s, the U-turn (0.85 m commanded at 0.45 m/s), the cross-track safety limit and the tractor keep-out.

**Why it used to stop after row 3:** a tractor is parked at (17.9, −9.6) in the east headland of rows 3–4. The old stop-only obstacle pause waited there forever.

## 2. Obstacle avoidance (`agri_obstacles.py` + driver)

**Detection.** The 360° LiDAR plus the depth camera give 3D points. From these:
- the ground is removed, along with anything lower than `obstacle_min_height`
- **crop-row bands** below `crop_max_height` are removed (they are the crop, not obstacles)
- the robot itself is removed

The rest is clustered and tracked with an alpha-beta filter, which gives position and velocity. An obstacle is **dynamic** when its speed stays above `dynamic_speed`. Tracks are published on `/agriculture/obstacles`.

**Behaviour in a row:**

| Situation | What the robot does |
|---|---|
| Moving obstacle in the robot's corridor | **YIELD**: waits, up to `yield_timeout`, then treats it as static |
| Static obstacle inside the crop area | **DETOUR**: moves to the neighbouring row line (tyres still in aisles), passes, moves back to the same row and continues |
| Obstacle at the row end (in the last `row_end_zone`, reaching past the last plant) | The row ends in front of it; any shortfall stays visible in the coverage |
| Headland U-turn area blocked | Waits if the obstacle moves; if not, a **REVERSE U-TURN** into the next row, so the pattern continues |
| No free way | Holds and retries. The mission is never abandoned. |

The current row, direction, row start and end, and progress are kept throughout, and published on `/crop_row/progress`. The existing depth-camera stop remains as a last resort, covering the last `hazard_stop_distance`.

## 3. Simulated soil moisture (`agri_moisture_sensor.py`, `agri_moisture_map.py`)

**The probe:**
- It is `moisture_probe_link` in `agri_ugv.urdf.xacro`: a visual only, with no mass.
- It sits under the chassis, 0.30 m left of the crop row, so it reads soil rather than plants.

**The moisture field:**
- It is smooth and spatially correlated, generated from `moisture.seed` (the same seed always gives the same field).
- Values range over `min_moisture`–`max_moisture`, with dry, normal and wet regions.
- Readings add `noise` (standard deviation, default 0.8 %).

**Topics:**
- `/agriculture/moisture` (JSON: `moisture`, `x`, `y`, `stamp`, `row`, `category`), at `update_rate`, only over the crop field.
- `/agriculture/moisture_map` and `.../confidence` (OccupancyGrid, also visible in RViz).
- `.../status` (coverage and statistics).
- `/agriculture/moisture_events`.

**How the map is built:**
- Each sample is spread over nearby cells with a Gaussian weight inside `interpolation_radius`, so faded areas on the map are interpolated.
- Cells farther than that from every sample stay **UNKNOWN**; nothing is invented.
- **Mapping %** is the surveyed field area, meaning cells within `coverage_radius` of a sample, not a message count.
- The map is **COMPLETE** at `complete_threshold`, or when every row has been driven. In that case the area it could not reach is reported.

## 4. Perception (`agri_vision.py`)

Humans, crop, weeds, unknown plants and other objects are detected as before. **A crop plant is never classified as a weed; uncertain plants are UNKNOWN.** Changed:
- non-green crop parts (cotton bolls) on the row lines are no longer reported as objects
- the person detector is stricter, so fence posts are no longer reported as humans

## 5. Dashboard (http://localhost:8080)

- **Top row:** the RGB camera with the detection overlay, and the field/row map (rows, completed rows, current row, robot, trail, tracked obstacles with motion arrows, humans, weeds).
- **Middle row:** the **live moisture map** (continuous DRY → WET colour scale, faded where interpolated, grey where not surveyed, sample dots, robot, field boundary, current reading), and the **robot status**:
  - row X / N, speed, direction, soil moisture, x/y
  - row state (FOLLOWING ROW / AVOIDING OBSTACLE / RETURNING TO ROW / YIELDING / TURNING / REVERSE U-TURN / FIELD COMPLETE)
  - crop coverage % and moisture mapping %
  - obstacle distance, direction and state
- **Below:** detection counters, the mission summary (crop coverage, moisture map, rows, humans, weeds, obstacles, detours, reverse turns, moisture range), **START / STOP ROBOT**, and the event log. The log shows rows, obstacles, and moisture samples and regions, but not every sensor frame.

## 6. Autonomy supervisor (`agri_supervisor.py` + `agri_autonomy_core.py`)

The supervisor runs around the row mission and never replaces it: row following, detours, yields, reverse U-turns and START/STOP are unchanged. It works only from the real topics (pose, IMU, wheel encoders, the commanded velocity, the driver's progress, the obstacle tracker, the moisture map, and camera, depth and LiDAR arrival). It publishes `/agriculture/autonomy/status`. `autonomy:=false` switches it off.

| question | how it is answered |
|---|---|
| Where am I, how sure? | **Localization confidence** = pose freshness × IMU yaw-rate agreement × wheel-encoder agreement (slip) × no pose jumps. Each factor is shown (hover the value). |
| Can I still see? | **Sensor health** (rate and age of every stream) sets the autonomy level:<br>• **FULL**<br>• **DEGRADED**: speed cap 0.8 m/s<br>• **LIMITED**: 0.5 m/s, obstacle tracker silent<br>• **SAFETY HOLD**: only with no pose, or no LiDAR *and* no depth; it resumes by itself |
| What is the field? | Rows, spacing, direction and boundary come from the world model (23 rows); nothing is hard-coded |
| What is done? | Completed rows (≥ 98 % of the row driven), remaining rows, **interrupted rows** (detour or row ended early) and **uncovered stretches** of rows already driven |
| Am I making progress? | Commanded ≥ 0.08 m/s for 6 s but moved < 0.25 m gives **NO_PROGRESS**:<br>1. Straight back-off along the row (tyres stay in the aisles), up to 0.6 m.<br>2. The same row is retried.<br>After 3 back-offs within 2 minutes: **FAULT** (hold and manual check) instead of an endless loop. |
| Blocked for good? | A **static** obstacle holding the rover with no free side for 20 s gives **ABORT CURRENT SUBTASK**:<br>1. The row ends there.<br>2. The rest is recorded for a revisit.<br>3. The lawnmower continues.<br>Before this, the driver could hold for ever. |
| Is the survey complete? | Moisture-map %, current reading, dry / normal / wet share and unmapped %, until **MOISTURE MAP COMPLETE** |

**States:** READY → FIELD_ANALYSIS → ROW_NAVIGATION ⇄ OBSTACLE_AVOIDANCE → ROW_REJOIN; ROW_RECOVERY / RECOVERY; at the end MOISTURE_MAPPING → COVERAGE_VERIFICATION → MISSION_COMPLETE or **REVISIT_REQUIRED** (with the list of rows and stretches); FAULT.

**Decisions:** CONTINUE, SLOW_DOWN, AVOID, BACKTRACK (reverse U-turn), RECOVER, WAIT TEMPORARILY, RESUME, ABORT CURRENT SUBTASK, COMPLETE MISSION.

**Dashboard:**
- The **AGRICULTURE AUTONOMY** panel shows mission, state, decision, row X / N, coverage, moisture mapping, current moisture, navigation, obstacle, localization and perception confidence, robot progress, autonomy level, recovery, manual interventions, rows done / left, interrupted rows, uncovered stretches and sensor health.
- The **AUTONOMY METRICS** panel shows mission success, collisions, near misses, replans, recoveries, no-progress events, minimum clearance, distance, mission time, field coverage, rows completed / revisited, moisture map and unmapped area.
- On the **field map**: per-row coverage colours, interrupted rows (orange), and uncovered stretches (red dashes).
- The supervisor's decisions appear in the **event log**.

**Not automated yet:** driving back to revisit a skipped stretch. It is reported (REVISIT_REQUIRED, with rows and x ranges), and "rows revisited" stays 0.

## Tests (offline, no ROS)

```bash
python3 -m pytest -q tests/        # all, ~25 s
```

- **Mission:** 3-, 5- and 10-row fields and the full world (23 rows from the model) all run to the last row with 100 % coverage. The row-end rule is tested, and there are no hard-coded row numbers.
- **Obstacles:** static obstacle (detour back to the same row), person crossing (yield), vehicle in the headland (wait), obstacle entering from the side, obstacle leaving the row, blocked headland (reverse U-turn), and the full world with its tractor and workers (23/23 rows, no contact).
- **Obstacle tracker:** crop rows are never obstacles; velocity of a moving person; track expiry.
- **Moisture:** reproducible seed, dry, normal and wet regions, spatial correlation, small noise, readings that follow location, coverage by area, unknown areas never invented, events that are meaningful rather than every reading.
- **Perception:** the 10 crop/weed/human scenes, and the rule that a CROP never becomes a WEED.
- **Autonomy (`test_agri_autonomy.py`, A1–A10):** the real driver and supervisor run closed loop.
  - **A1** complete coverage, and field understanding from the world
  - **A2 / A6 / A7** static obstacle, interrupted row, rejoining the same row
  - **A3** moving human
  - **A4** moving vehicle in the headland
  - **A5** animal
  - **A8** stuck spot, then back-off and resume; also a permanent stuck, which ends in FAULT and not a loop
  - **A9** a blocked row is ended and reported for a revisit, instead of holding for ever
  - **A10** moisture progress up to MOISTURE MAP COMPLETE
  - **Sensor loss:** DEGRADED, then LIMITED, then SAFETY HOLD, then resume
  - **Localization confidence:** falls with wheel slip and with a stale pose

## Status

Everything above is **tested offline** (kinematic rover model, synthetic sensors). **It has not run in Gazebo yet.**

On the first Gazebo run, check these:
- the obstacle tracker picks up the parked tractor and the workers
- detours and reverse U-turns stay within the cross-track safety limit on real soil
- the real crop heights stay under `crop_max_height`

In the offline test of the supplied world, coverage is 99.3 %. The row ends directly behind the parked tractor and one worker cannot be reached without touching them.
