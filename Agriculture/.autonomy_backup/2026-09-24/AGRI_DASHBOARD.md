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

