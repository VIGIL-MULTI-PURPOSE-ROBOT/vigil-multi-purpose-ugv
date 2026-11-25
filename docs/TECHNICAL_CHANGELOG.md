# Technical changelog: Rock Terrain navigation (25 November 2026)

Branch `vigil-audit-2026-11-25`, on top of commit `4f9c8c1` plus the uncommitted audit changes. Nothing was committed.
The scope is `Rock Terrain/` only; no other environment was modified.
See [IMPROVEMENT_REPORT.md](IMPROVEMENT_REPORT.md) for the evidence behind each change and [REGRESSION_RESULTS.md](REGRESSION_RESULTS.md) for results.

Originals of every modified file were backed up, outside the repository, to
`~/vigil-baseline-validation/backups/2026-11-25-before-rock-terrain-improvement/` (with a `SHA256SUMS` file).

## Modified files (`Rock Terrain/src/vigil_rough_terrain/`)

### `scripts/planner_core.py` (navigation logic)

**Replanning hysteresis.** A hazard or blocked replan runs only on a new terrain map (`map_version`), after `min_replan_interval` (1.0 s), or for a stuck or tilt reason. A pending reason is kept until then.

**One clearance rule everywhere.**
- `Planner.check_path()` gains `near_margin`/`near_radius`. Within `escape_radius` (1.3 m) of the robot, a route may keep the robot's current clearance (`start_clearance()`: 0.15, 0.10, 0.05 or 0 m) instead of the full margin. It never goes below 0 (no body overlap) and never closer than the current clearance.
- The same rule is applied when A* candidates are validated (`_plan`), when the current path is rechecked on new maps, and when the heading search runs.

**Heading search (`_best_heading`).** It now sweeps the oriented footprint along the whole segment, and rejects headings that need a turn in place where turning is not clear. Previously it checked the robot's centre line plus an end pose with zero margin.

**Motion shield (`_shield`, `_motion_clear`).**
- Every path-following command is projected `shield_horizon` (0.6 s) along its arc. The footprint must keep min(`shield_margin` 0.10 m, current clearance) from mapped CLIFF/OBSTACLE cells.
- If not, the shield tries turning only, then stops. A veto lasting longer than `blocked_timeout` starts a bounded recovery.
- This replaces the original behaviour of turning in place even when `rotation_clear()` was false.

**Unknown terrain.**
- `Planner.lethal_at(strict=...)`: `strict=1` treats UNKNOWN cells within `unknown_near_cliff` (1.0 m) of a mapped drop, plus POSSIBLE_DROP cells, as blocked. It is used for turn-dominant motion and `rotation_clear()`.
- `strict=2` treats all UNKNOWN and POSSIBLE_DROP cells as blocked. It is used for reversing (`rear_clear()`, recovery, and shield checks of commands with v < 0).
- Cells under the robot's current footprint never block.

**Body-overlap rule (`set_body_cells`).** Hazard cells already under the robot's body are excluded from motion checks until the body leaves them, so motions are judged only on new hazard cells.

**Hazard slowdown.** Speed falls linearly from 1.0 to `hazard_min_speed_factor` (0.5) as the robot centre approaches CLIFF/OBSTACLE cells (`hazard_slow_band` 0.6 m).

**Bounded failure handling.**
- **Terminal states:** `GOAL_UNREACHABLE`, `RECOVERY_LIMIT_REACHED`, `SENSOR_TIMEOUT` and `LOCALIZATION_LOST`. Each forces zero velocity, has a `failure_reason`, and is cleared by `set_goal()`.
- **`HOLD`:** zero velocity on a stale map or stale/invalid pose, before the timeouts expire.
- **Unreachable goal:**
  - no route even through unknown terrain for `unreachable_confirm_time` and over `unreachable_confirm_maps` new maps;
  - or `route_search_timeout` with no adoptable route;
  - or no net progress of `progress_min_step` toward B for `no_progress_failure_time`.
- **Recovery limit:** `max_recoveries`.
- **Startup:** no first pose or first map within `startup_timeout`.
- **Status:** `status()` adds `terminal`, `failure_reason`, `recoveries`, `explore_attempts`, `map_version`, `map_age`, `no_route_for`, `no_progress_for`, `hold_reason`, `body_overlap_ticks`, `shield_vetoes` and `hazard_speed_factor`.
- **API (backward compatible):** `tick(t, pose, pose_t=None)` and `update_map(..., t=None)`.

### `scripts/terrain_core.py` (perception)

`_edge_scan` / `_apply_events`: when no ground has been seen yet in an image column, the drop reference is the robot's own front. Such an event no longer writes `edge_drop`/`edge_theta` (confirmed HIGH CLIFF) to that cell. The shadow strip still marks every unobserved cell between the bumper and the landing point. Event tuples gain a 9th field, `front_ref`.

### `scripts/terrain_mapper.py`

Each depth frame is integrated once (by sequence number). Grids are not published while the newest depth frame is older than `depth_timeout`, or the pose older than `pose_timeout` (sim time), so stale perception reaches the navigator as a stale map. Non-finite poses are ignored.

### `scripts/cliff_navigator.py`

- Passes pose and map receipt times (node clock) to the core, and ignores non-finite poses and goals.
- Ticks and publishes `/cmd_vel` (zero) even before the first pose arrives.
- Logs terminal failures at ERROR level, and adds `pose_age` to the status.

### `scripts/scenarios.py`

`too_narrow` and `no_route`: the expected outcome changed from `SEARCHING` to `SAFE_STOP`, and the descriptions were updated. **Geometry, spawn and goal are unchanged for all 13 scenarios** (checked programmatically), and the Gazebo SDF worlds are untouched.

### `config/vision_nav.yaml`

New, documented parameters:
- `navigation`: `min_replan_interval`, `escape_radius`, `shield_margin`, `shield_horizon`, `hazard_slow_band`, `hazard_min_speed_factor`, `unknown_near_cliff`, `max_recoveries`, `route_search_timeout`, `unreachable_confirm_time`, `unreachable_confirm_maps`, `no_progress_failure_time`, `progress_min_step`, `map_stale_time`, `sensor_timeout`, `pose_stale_time`, `localization_timeout`, `startup_timeout`.
- `terrain`: `depth_timeout`, `pose_timeout`.

**No existing value was changed.** That includes footprint, safety margin, speeds, stop distance and thresholds.

### `test/closed_loop_sim.py`

- Passes map timestamps.
- Accepts a terminal safe stop for the `SAFE_STOP` expectation, and requires zero motion after it.
- New check: the footprint must keep at least `MIN_CLEARANCE` (0.10 m) from the true drop edges (`drop_clearance()`).
- The idle-time rule is unchanged (≤ 1.0 s) for all goal scenarios. For the two `SAFE_STOP` scenarios it is bounded at 2 × `blocked_timeout` + 1 s.

### `dashboard/index.html`

Colours and decision labels for `HOLD`, `SAFETY STOP` and the four terminal failure states.

### `CMakeLists.txt`

Registers `navigation_safety_unit` (pytest) under `colcon test`.

## New files

- `Rock Terrain/src/vigil_rough_terrain/test/test_navigation_safety.py`: 20 pytest unit tests.
- `Rock Terrain/tools/regression/`: `vigil_regression.sh`, `monitor.py`, `metrics.py`, `summarize.py`, `test_metrics.py` (10 tests) and `README.md`.
- `docs/IMPROVEMENT_REPORT.md`, `docs/REGRESSION_RESULTS.md` and this file.

## Validation tooling fix (25 Nov 2026, 19:05)

`Rock Terrain/tools/regression/vigil_regression.sh` (and the earlier `~/vigil-baseline-tools/vigil_baseline_validation.sh`):

- **Bug:** `sweep_partition()` looped `for d in /proc/[0-9]*` without declaring `d` local. Bash local variables are dynamically scoped, so this overwrote the caller's `local d` (the per-trial output directory) with the last `/proc/<pid>` visited. All post-processing after the sweep then read and wrote `/proc/<pid>/launch.log`, `runtime_*.txt` and `result.json`, causing `FileNotFoundError`. The monitor's own `result.json` was already written to the correct folder before the sweep.
- **Fix:**
  - All helper variables are local.
  - The trial folder variable is renamed `rundir`, and a path guard refuses any trial folder outside `$OUT/runs/`.
  - Only this user's own processes are inspected (`[ -O ]`), and the `environ` redirection itself is silenced (this caused the `Permission denied` messages).
  - Missing `launch.log` is tolerated.
  - Post-processing never crashes: a missing or corrupt `result.json` becomes `MONITOR_FAILURE` / FAIL, missing optional diagnostics are listed in `diagnostics_missing`, and a failed run is never reported as a pass.
- **Navigation, perception and control code:** not touched.

## Gazebo safety fixes (25 Nov 2026, 19:35–20:05)

Trigger: Gazebo run 1 of the improved code (`regression-20261125-185120`, see `REGRESSION_RESULTS.md` §3). The footprint overhung the plateau edge by up to ≈ 0.1 m in cliff_front t2 and too_narrow t1 and t3, and two runs stopped 1.4 m from the real edge behind a phantom cliff cell. Backup of every file below, taken before the change: `~/vigil-baseline-validation/backups/2026-11-25-before-gazebo-safety-fixes/` (`SHA256SUMS`, `RESTORE.txt`).

### `scripts/planner_core.py`
- **Stopping distance in the motion shield (`_motion_clear`).** `drive.py` ramps the wheels at 0.01 per 20 ms (0.5 m/s², 0.5 rad/s²). The navigator now keeps `vel_est`, the expected actual speed under the same ramp. The shield predicts the robot's motion in four phases, and the footprint plus margin must stay clear of CLIFF, OBSTACLE and (new) POSSIBLE_DROP cells the whole way:
  1. the previous command for `command_latency`;
  2. ramping to the new command;
  3. holding the new command for `shield_horizon`;
  4. braking to a standstill.

  Before, the shield assumed the robot stops instantly.
- **Shield fallbacks:** the same command at 50 % and then 25 % speed, then turning only, then stop. The robot slows down instead of halting.
- **Seen-terrain speed cap (`_seen_speed_cap`).** Forward speed is limited so the robot can stop before its footprint reaches terrain not yet known to be safe (unobserved, POSSIBLE_DROP, CLIFF or OBSTACLE). The stopping distance allows for `perception_latency + command_latency`, then braking. The floor is `creep_speed` (0.15 m/s), so the robot can still start from an unseen spawn. Offline, the first version of the fix still reached 0.00 m: the robot was at 0.6 m/s when the edge first appeared in the map, only 0.35 m ahead.
- **Unconfirmed drops are avoided by the shield** (`lethal_at(drops=True)`, `start_clearance(drops=True)`). When the robot drives *alongside* an edge, the grazing angle never steepens enough for the mapper to confirm a cliff, so the cells stay POSSIBLE_DROP. Offline, cliff_front came within 0.00 m of such a 1.5 m drop. Planning is unchanged: A* still treats POSSIBLE_DROP as cost.
- **Proven ground (`mark_proven`, `Planner.build`).** Applies only while the robot is level (tilt < `proven_max_tilt`):
  - UNKNOWN cells touched by its footprint count as observed. The robot can therefore reverse along its own track, including the spawn area the forward camera never saw.
  - CLIFF and POSSIBLE_DROP cells more than `proven_drop_inset` (0.30 m) inside the footprint are artefacts, because the robot is standing level on them. They are cleared for navigation only. The mapper's own map, the dashboard and `/vision/*` are unchanged.
  - Cells near the footprint's sides are never cleared (unit test).
- **Recoveries that cannot move.** The turn phase now also accepts a turn whose swept arc, braking included, keeps the margin, and tries the other direction once. After `null_recovery_limit` (2) consecutive recoveries that moved less than 5 cm and turned less than 0.1 rad, the navigator stops with RECOVERY_LIMIT_REACHED and a reason ("could not move at all"). It no longer stands still while using up all 6 recoveries.
- **Back off once stopped.** If the shield allows no motion, the robot is at rest, and a newer map still allows nothing, the recovery starts after `shield_rest_timeout` (0.5 s) instead of `blocked_timeout` (4 s).
- **Status:** new fields `vel_est`, `null_recoveries`, `proven_cells` and `seen_speed_cap`.

### `config/vision_nav.yaml` (new parameters only; no existing value changed)
`drive_linear_accel 0.5`, `drive_angular_accel 0.5` (must match `drive.py`), `command_latency 0.15`, `perception_latency 0.3`, `creep_speed 0.15`, `proven_max_tilt 10.0`, `proven_drop_inset 0.30`, `null_recovery_limit 2`, `shield_rest_timeout 0.5`.

### `test/closed_loop_sim.py`
- **Drive model (default):** a one-tick command delay, then the `drive.py` ramp. `--ideal-drive` reproduces the old instant model, for comparison only.
- **Idle check:** the "motors must not sit idle" check now looks at the robot's **actual** motion. A robot braking after a zero command is still moving. In `--ideal-drive` mode this is the same as the old rule.
- **No pass criterion was relaxed:** `MIN_CLEARANCE` is still 0.10 m, and the idle limits are unchanged.

### `test/test_navigation_safety.py`: 20 → 29 tests
- **New tests:**
  - shield uses stopping distance (from rest vs at 0.6 m/s);
  - `vel_est` follows the drive ramp;
  - speed cap before unseen terrain (helper plus path following);
  - POSSIBLE_DROP blocks motion onto it;
  - proven ground clears a phantom cliff and the unseen track (the mapper's map is untouched);
  - proven ground never clears a real edge beside the robot;
  - a tilted robot proves nothing;
  - recoveries that cannot move end quickly.
- **Changed test:** `test_motion_shield_blocks_turn_into_cliff` now starts with the robot already turning at 0.6 rad/s. A robot turning from rest really does turn only ≈ 0.2 rad before it can stop. The assertion that the shield blocks the turn is unchanged.
- **Mutation check:** disabling any one of the six new rules makes at least one test fail.

### Regression tooling (`Rock Terrain/tools/regression/`)
- **`metrics.verdict` includes clearance:** PASS needs the expected outcome **and**, in the cliff scenarios, a footprint clearance of at least `MIN_CLEARANCE` = 0.10 m to the true drop edge. `monitor.py` passes the measured clearance and writes `verdict_note`.
- **`summarize.py`:**
  - recomputes every verdict with the current rule and stars runs whose recorded verdict differs;
  - names the files whose content changed during a run;
  - tolerates missing snapshot files.
- **`vigil_regression.sh`:** the Git snapshot also records per-file hashes (`diff_files.sha256`) and `git diff --summary`. Run 1's tracked-diff hash changed even though no tracked file's modification time did, and the cause could not be traced.
- **`test_metrics.py`:** +1 test (clearance verdict).

### Not changed
- **Perception:** `terrain_core.py` and `terrain_mapper.py` are unchanged.
- **Other code:** `drive.py`, the footprint, `safety_margin`, `stop_distance`, `cruise_speed` and the scenario geometry are unchanged.
- **Phantom cell:** the source of the phantom HIGH CLIFF cell in Gazebo (too_narrow t2, no_route t3) was not found; the navigator now tolerates it instead.

## Recovery-margin fix (25 Nov 2026, 21:30–21:55)

**Trigger:** Gazebo run 2 (`regression-20261125-205430`) passed 17/18. The exception was too_narrow t1, which reached 0.000 m during a slow recovery reverse. Backup of the versions this replaces: `~/vigil-baseline-validation/backups/2026-11-25-before-recovery-margin-fix/`.

**`scripts/planner_core.py`:**
- **`_recover`:**
  - **Reverse:** needs `rear_clear` (observed ground behind) **and** `_motion_clear(-0.12, 0, margin)`.
  - **Turn:** needs `_motion_clear(0, ±0.35, margin)`. The margin-free full-circle `rotation_clear` shortcut was removed from the recovery.
  - The margin is min(shield_margin, current clearance incl. POSSIBLE_DROP), as for path following.
- **`Planner.clearance_cont`:**
  - Computes the body outline's clearance to CLIFF, OBSTACLE and POSSIBLE_DROP cells from a bilinearly interpolated distance field (`dist_haz`, built in `build()`). Cells under the body are excluded.
  - `_motion_clear`: when the robot is inside the safety band (margin < shield_margin), no predicted pose may have a continuous clearance below the current one minus `band_clearance_tolerance` (0.005 m).
  - Before this, the band rule worked on 0.05 m margin levels, which let slow motions creep down one level at a time.
- **`_penalise_here`:** when the shield starts a recovery, the spot 0.5 m ahead gets the same planner penalty as a hazard right in front, so the route does not lead back into the same deadlock.

**Other files:**
- **`config/vision_nav.yaml`:** new parameter `band_clearance_tolerance: 0.005`.
- **`test/test_navigation_safety.py`:** new test `test_slow_reverse_cannot_creep_towards_an_edge_inside_the_band`, built from the Gazebo geometry. The old reverse check allows this motion and the new rule blocks it; reversing parallel to the edge stays allowed. Mutation-checked.
- **`tools/regression/vigil_regression.sh`:** the navigator's ERROR-level terminal-state reports (`GOAL_UNREACHABLE`, `RECOVERY_LIMIT_REACHED`, `SENSOR_TIMEOUT`, `LOCALIZATION_LOST`) are now written to `terminal_state_log.txt` and no longer counted as runtime errors.

## Regression runner: simulator start-up failures (25 Nov 2026, 22:30)

**Trigger:** Gazebo run 3 (`regression-20261125-215202`). In 2 of 18 launches the ros2_control controller manager's service response to the spawner timed out, and the wheel controller was never activated.

**`tools/regression/vigil_regression.sh`:**
- **Detection:** a trial whose `launch.log` lacks `Configured and activated wheel_controller` and whose `controllers.txt` lists no active `wheel_controller` is classified as a simulator failure, unless it reached the goal:
  - category becomes `SIMULATION_FAILURE (wheel controller never activated)`, verdict FAIL;
  - the navigation category and verdict are kept in `navigation_*_before_override`;
  - a `sim_failure` marker is written.
- **Retry:** such a trial is re-run once. The failed attempt is kept and marked `superseded_by_retry`. The count goes to `notes.txt`.

**`tools/regression/summarize.py`:**
- Superseded attempts stay in the every-run table, marked "(superseded: retried)".
- They are listed above the per-scenario table and excluded from it and from the baseline comparison.

**Launch fix (`launch/terrain_sim.launch.py`, 22:45):**
- **Spawner:** `--service-call-timeout` is lowered from 100 s to 20 s. A normal spawn takes about 4 s.
- **Bounded retries:** if the spawner fails, a fresh one starts, up to 4 attempts. Each attempt is logged ("Controller spawner attempt k/4 failed - retrying").
- **Clear failure:** after 4 failures the launch shuts down with the reason "wheel controller could not be activated", instead of running a rover that cannot move.
- **Tested:**
  - with stand-in launch classes: success at once, success on attempt 3, 4 failures, rover spawn failure;
  - not yet in Gazebo.
- **Known limitation:** if an attempt failed *after* loading a controller, the next attempt may report it as already loaded. That case was not observed: in both Legion failures nothing was loaded.
- `--switch-timeout 90` and `--controller-manager-timeout 120` are unchanged.

## Documentation updated because the behaviour changed

- `README.md`: the recovery and safe-stop sentence.
- `docs/PROJECT_AUDIT.md`: failure-handling and safe-stop status.
- `docs/VALIDATION_REPORT.md`: a pointer to the new results (the 25 Nov record is kept as it was).
- `Rock Terrain/README.md`
- `Rock Terrain/src/vigil_rough_terrain/docs/VISION_NAVIGATION.md`: the never-give-up section.

## Not changed (and why)

- **Robot footprint (1.53 × 1.12 m, centre offset 0.02 m):** verified against the expanded URDF (wheel envelope 1.531 × 1.119 m, centre 0.019 m).
- **Safety margin 0.15 m, stop distance 1.0 m, planning resolution 0.20 m, cruise speed 0.60 m/s, max tilt 38°:** unchanged.
- **`drive.py`:** its 0.6 s command watchdog uses wall time while its 50 Hz timer uses sim time. This only matters at a low real-time factor (the baseline RTF was 0.96–0.99). It is reported, not changed.
- **Localization:** still `/sim/ground_truth`. No EKF, visual odometry or SLAM was added.
