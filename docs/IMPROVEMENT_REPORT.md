# Rock Terrain navigation: improvement report (25 November 2026)

[Regression results](REGRESSION_RESULTS.md) · [Technical changelog](TECHNICAL_CHANGELOG.md) · [Project audit](PROJECT_AUDIT.md)

**Scope:** `Rock Terrain/` (`vigil_rough_terrain`) only.

**Evidence status:**
- **Offline:** every result marked *offline* was measured in the closed-loop ray-cast simulator. It runs the real perception and navigation code with idealised sensing: noise-free depth, perfect pose, and kinematic motion without slip.
- **Gazebo baseline:** recorded on the Legion (RTX 5070, ROS 2 Jazzy, Gazebo 8.11) on 25 Nov 2026 with the original code.
- **Gazebo, improved code, run 1 (25 Nov, 18:51):**
  - **Gains:** narrow_route 281.8 → 30.1 s, and both unreachable scenarios now end in terminal safe stops.
  - **Safety problem:** the footprint overhung the plateau edge by up to ≈ 0.1 m (cliff_front t2, too_narrow t1 and t3). That makes it 13/18 under the corrected verdict, not the 18/18 the runner printed.
  - **Fixes since:** the braking-aware shield, the "don't outrun the map" speed cap, avoiding unconfirmed drops and proven ground. Offline they give 13/13 with a realistic drive model, on both library sets.
  - **Gazebo run 2 (20:54):** **17/18**:
    - cliff_front and narrow_route kept 0.30 m in every trial;
    - narrow_route took 34.7 s (baseline 281.8 s);
    - the one failure, too_narrow t1, was a slow recovery reverse creeping to the edge.
  - **Since run 2:** recovery motions also pass the shield, and inside the safety band the body may not get closer at all. Offline: 13/13 on both library sets.
  - **Gazebo run 3 (21:52):** all 16 valid trials PASS.
    - Every trial kept at least 0.20 m from the true edge.
    - too_narrow passed 3/3.
    - Two launches failed at simulator start-up: the wheel controller was never activated. They are not navigation results; the runner now detects them and retries.
  - **Gazebo run 4 (22:21), final version incl. the spawner retry: 18/18 PASS.**
    - Minimum clearance was 0.15 m.
    - Against the original: narrow_route 281.8 → 34.5 s and cliff_front 48.1 → 38.0 s, and both unreachable scenarios stop safely.
    - hill_choice is slower: 29.8 → 43.6 s.
    - The spawner retry path was not triggered in this run.
  - **Details:** see `REGRESSION_RESULTS.md` §3–§5 and the changelog section "Gazebo safety fixes". The offline numbers in this report's later sections predate that section and used the idealised drive model.

Localization in this subsystem is still simulator ground truth (`/sim/ground_truth`). Nothing here is GPS-denied visual localization.

## 1. Initial issues (Gazebo baseline, original code)

| Observation | Value |
|---|---|
| `narrow_route` | `GOAL_REACHED` after 281.8 s; 641 replans (135 hazard); 10 recoveries; minimum footprint clearance 0 m |
| `cliff_front` | `GOAL_REACHED` after 48.1 s; minimum clearance 0.05 m |
| `too_narrow`, `no_route` | no terminal state; still searching at the 120 s limit; minimum clearance 0 m |
| Monitor path length | `narrow_route` reported 440.88 m (implausible) |

## 2. Root causes (with the evidence for each)

**1. Monitor over-counted distance.** On every ground-truth message (~100 Hz) it added the distance from the last 0.5 s trajectory sample, so the same stretch was counted repeatedly. Replaying that logic on a true 10 m path at 100 Hz gives **215.9 m**. *Fix:* `tools/regression/metrics.PathLength` integrates time-ordered samples, ignores duplicate stamps, rejects jumps and accumulates no jitter (10 unit tests).

**2. Replanning at tick rate.**
- The "hazard directly ahead" check sets a replan reason on every 0.1 s tick, and any replan with a reason was adopted.
- Instrumenting `_plan` offline showed **180 of 210** plans in `narrow_route`, 241 of 300 in `no_route` and 40 of 51 in `cliff_front` were less than 0.15 s after the previous plan.
- The Gazebo navigation events show the same pattern: "PATH BLOCKED → SAFE PATH FOUND" every 0.1 s.
- A* on an unchanged map returns the same answer, so these replans add nothing.

**3. A* and the footprint sweep disagreed.**
- A* allows the safety-margin band within 1.3 m of the robot, so it can leave a tight spot. The footprint sweep that validates every route rejected that band anywhere beyond 0.3 m.
- Whenever the robot was near an edge, every route was found and then rejected. In `narrow_route` offline, 42 replans had all three planning modes rejected by the sweep, and in `cliff_front` 39 of 51 did. That forced the fallback heading search (`CLIFF_AVOIDANCE`).

**4. The heading-search fallback was the least safe path.**
- It checked only the robot's centre line plus the end pose with zero margin. It never swept the body along the segment.
- In all four cliff scenarios in Gazebo, the minimum clearance occurred 13–15 s in, right after entering `CLIFF_AVOIDANCE`. For example, `cliff_front` picked heading 45° from (2.4, −1.9), toward the pit corner, and reached 0.05 m.
- The trajectory plot shows the rover in `narrow_route` circling the start plateau for about 250 s, repeatedly reaching its outer edges.

**5. Turning in place without clearance.** When `rotation_clear()` was false, `_follow` still commanded `(0, w)`. An offline trace showed clearance going 0.15 → 0.10 → 0 → overlap during one such turn.

**6. Perception false positives at the robot's front.**
- `_edge_scan` starts each image column with the robot's front as the "last ground" reference. When the first valid point in a column was already the pit floor, the HIGH CLIFF edge was written at the robot's own front cell.
- Offline this happened 55× in `cliff_front` and 165× in `narrow_route`. Examples: a cell at (3.45, 1.25) 0.72 m inside solid plateau, exactly at the bumper; another at (2.3, 0.7), 1.2 m inside the plateau.
- Once the new checks respected the map, these phantom cells froze the robot.

**7. Blind reversing and turning near cliffs.**
- Recovery reversed over cells that were merely *not mapped as hazards*. Unknown cells behind the robot (the camera looks forward) counted as free. One offline trace reversed toward an unobserved chasm edge until true clearance reached 0.
- Turns swept the robot's corners into unobserved terrain next to a mapped drop in the same way.

**8. No terminal failure state.** `TERMINAL = ('GOAL_REACHED',)`; the design was "never give up".

**9. Stale inputs were not detected.** The navigator kept driving on the last pose if `/sim/ground_truth` stopped. `terrain_mapper` kept re-integrating and republishing the last depth frame, so a frozen camera looked like a live map.

## 3. Changes

Changes 2–9 are described per file in the [technical changelog](TECHNICAL_CHANGELOG.md).

| Finding | Change |
|---|---|
| 2 | Replan only on new map data or after `min_replan_interval` |
| 3 | A single clearance rule for A* validation, path rechecks and the heading search: full margin, or the robot's current clearance within `escape_radius`, never below 0 |
| 4 | The heading search sweeps the whole footprint and rejects turns that are not clear |
| 5 | Motion shield on every command (0.6 s projection, 0.10 m) |
| 6 | Events referenced at the robot front no longer mark that cell; unobserved cells ahead stay marked via the shadow |
| 7 | Reversing requires observed ground; turns treat unknown terrain within 1 m of a mapped drop as blocked |
| 8 | Terminal states `GOAL_UNREACHABLE`, `RECOVERY_LIMIT_REACHED`, `SENSOR_TIMEOUT` and `LOCALIZATION_LOST`, with zero velocity, a published reason and restart on a new goal. Unreachability needs repeated evidence (time plus several new maps), never one failed plan. |
| 9 | `HOLD` (zero velocity) on a stale pose or map; terminal after the timeouts; the mapper stops publishing on stale depth or pose |

Additionally, the rover slows to 0.5× cruise near CLIFF/OBSTACLE edges, reducing stopping distance plus latency. **No existing parameter value was changed**: footprint, 0.15 m safety margin, 1.0 m stop distance, 0.20 m planning resolution, 0.60 m/s cruise and 38° tilt limit all stay the same.

## 4. Changes I tried and rejected

1. **The motion shield without the body-overlap rule.** Safe, but it froze on the phantom cliff cells (finding 6). `narrow_route` ended in a false `RECOVERY_LIMIT_REACHED`, and `cliff_front` took 41 s instead of 28 s. It was kept only together with the overlap rule and the perception fix.
2. **The perception fix without the reverse and turn restrictions.** `no_route` clearance fell to 0.00–0.05 m because recovery reversed and turned into unobserved terrain. That is a safety regression, so it was fixed (finding 7) before acceptance.
3. **A no-progress budget of 90 s with a 0.05 m progress step.** `too_narrow` did not terminate within 120 s, because small creeping gains kept resetting the timer. The step is now 0.5 m of net progress toward B and the budget 75 s.

## 5. Tests

| Test | Result (sandbox, Python 3.13) |
|---|---|
| `test/closed_loop_sim.py`, 13 scenarios (offline) | **13/13 PASS** |
| `test/closed_loop_sim.py --terrain` (real rock mesh) | PASS: `GOAL_REACHED` at 4.9 s, as in the baseline |
| `test/test_navigation_safety.py` (new, 20 tests) | **20/20 PASS** |
| Mutation check: disable the shield, front-ref fix, hysteresis or strict reverse | the corresponding test fails in each case |
| `tools/regression/test_metrics.py` (new, 10 tests) | **10/10 PASS** |
| `py_compile` / pyflakes on changed files | no errors; remaining unused-import warnings existed before |
| `vision_nav.yaml` parses | yes |
| Regression runner dry run (fake ROS, my sandbox) | Git checks, offline stage, install check, process isolation and cleanup, failure classification and summary all work. **This is not runtime evidence.** |
| `colcon build`, Gazebo scenarios | **Not run here.** No ROS or Gazebo in the sandbox, and this session has no shell on the Legion. |

## 6. Results so far (offline, original vs improved; same harness, same scenarios)

| Scenario | Original | Improved |
|---|---|---|
| cliff_front | GOAL_REACHED 27.9 s, 47 replans (8 hazard), clearance 0.18 m | GOAL_REACHED 28.1 s, 5 replans (4 hazard), clearance 0.20 m |
| flat | GOAL_REACHED 16.7 s | identical |
| hill_choice | GOAL_REACHED 21.4 s | GOAL_REACHED 25.1 s (slower near the steep face; slowdown band) |
| narrow_route | GOAL_REACHED 81.7 s, 206 replans (113 hazard), path 30.3 m, clearance 0.15 m | GOAL_REACHED **23.2 s**, **5 replans (4 hazard)**, path 11.0 m, clearance 0.21 m |
| too_narrow | searching at 120 s (no terminal state), clearance 0.10 m | **GOAL_UNREACHABLE at 80.5 s**, zero motion after, clearance 0.20 m |
| no_route | searching at 120 s (no terminal state), clearance 0.10 m | **RECOVERY_LIMIT_REACHED at 72.0 s**, zero motion after, clearance 0.20 m |

Full per-metric tables are in [REGRESSION_RESULTS.md](REGRESSION_RESULTS.md).

## 7. Remaining limitations

- **Simulation only.** The final version passed the Gazebo regression (run 4: 18/18, minimum cliff-edge clearance 0.15 m). It has not been tested on hardware. Two mechanisms have not yet been exercised in Gazebo: the controller-spawner retry path, and the proven-ground filter for false cliff cells.
- **Localization is simulator ground truth.** LiDAR, IMU and wheel encoders are still unused by this navigation loop.
- **No body-collision instrumentation.** Clearance is geometric.
- **Unreachable goals end in either `GOAL_UNREACHABLE` or `RECOVERY_LIMIT_REACHED`** (Gazebo run 4, no_route: 2 and 1). Both are safe stops at zero velocity with a reason. no_route t1 needed 134.5 s of the 150 s limit to stop.
- **`hill_choice` is slower in Gazebo** (29.8 → 43.6 s mean). The time is lost in the flat approach (≈ 8 → 12 s) and the 22° climb (8.0 → 13.8–20.3 s). This fits the seen-terrain speed cap near the crest, but it is not confirmed: the offline simulator does not reproduce the slowdown.
- **Untested robustness:** sensor noise, depth dropouts, pose drift and moving obstacles are not tested. The offline simulator is idealised.
- **Wall-time watchdog:** `drive.py` uses wall time for its 0.6 s watchdog.

## 8. Recommended next phase

1. **Run the Gazebo regression** (6 scenarios × 3 trials) and fill in REGRESSION_RESULTS §3.
2. **Add noise to the offline simulator** (depth noise, dropouts, pose drift) and repeat the comparison.
3. **Use sensor-derived pose:** a `pose_topic` parameter, an EKF over wheels, IMU and RGB-D visual odometry, and logged error against ground truth.
4. **Add a body-contact sensor** to the chassis in simulation for real collision counts.
