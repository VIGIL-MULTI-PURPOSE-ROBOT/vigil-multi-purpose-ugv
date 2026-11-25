# Rock Terrain regression results

[Improvement report](IMPROVEMENT_REPORT.md) · [Technical changelog](TECHNICAL_CHANGELOG.md) · [Tools](../Rock%20Terrain/tools/regression/README.md)

Clearance throughout is the minimum distance from the 1.53 × 1.12 m footprint rectangle (ground-truth pose) to the true drop edge of the scenario geometry. 0 means part of the footprint was over the drop. **Body collisions are not instrumented anywhere, so no collision-free claim is made.**

## 1. Gazebo baseline: original code (measured 25 Nov 2026, 17:03, Legion)

Measured with monitor v1: one trial each, RTX 5070, Gazebo 8.11, ROS 2 Jazzy, real-time factor 0.96–0.99.

| Scenario | Outcome | Goal time (sim s) | Replans | Hazard replans | Recoveries* | Min clearance m | Max tilt ° |
|---|---|---:|---:|---:|---:|---:|---:|
| cliff_front | GOAL_REACHED | 48.1 | 116 | 50 | n/r | 0.05 | 5.1 |
| flat | GOAL_REACHED | 29.1 | 3 | 0 | n/r | — | 4.5 |
| hill_choice | GOAL_REACHED | 29.8 | 3 | 1 | n/r | — | 28.1 |
| narrow_route | GOAL_REACHED | 281.8 | 641 | 135 | 10 (state entries) | 0.00 | 5.6 |
| too_narrow | **no terminal state** (RECOVERY at 120 s) | — | 367 | 34 | n/r | 0.00 | 5.2 |
| no_route | **no terminal state** (CLIFF_AVOIDANCE at 120 s) | — | 339 | 16 | n/r | 0.00 | 5.2 |

\* n/r = not recorded by monitor v1. Path length from v1 is not used: it over-counted (440.88 m for `narrow_route`).

## 2. Offline: original vs improved (measured 25 Nov 2026, same harness)

Both versions ran in the closed-loop ray-cast simulator against the same scenario definitions: idealised depth, perfect pose, kinematic motion, 0.1 s step, map every 0.5 s. Each run is deterministic, so each was run once.

> **Caveat found later (§3):** the §2 numbers used an *idealised* drive (the robot moved exactly as commanded and stopped instantly), and the 13/13 result depended on library versions: with Ubuntu 24.04's own numpy 1.26 / scipy 1.11 / OpenCV 4.6 the same code failed `too_narrow` and `no_route` (idle 12–16 s). Both problems are fixed in §4.

| Scenario | Version | Final state | Time (sim s) | Replans | Hazard | Recoveries | Path m | Min clearance m | Max roll / pitch ° | Longest no-progress s | Motion after terminal |
|---|---|---|---:|---:|---:|---:|---:|---:|---|---:|---:|
| cliff_front | original | GOAL_REACHED | 27.9 | 47 | 8 | 0 | 13.41 | 0.18 | 0.0 / 0.0 | 7.6 | — |
| | improved | GOAL_REACHED | 28.1 | 5 | 4 | 0 | 12.62 | **0.20** | 0.0 / 0.0 | 2.0 | — |
| flat | original | GOAL_REACHED | 16.7 | 1 | 0 | 0 | 9.72 | — | 0.0 / 0.0 | 0.6 | — |
| | improved | GOAL_REACHED | 16.7 | 1 | 0 | 0 | 9.72 | — | 0.0 / 0.0 | 0.6 | — |
| hill_choice | original | GOAL_REACHED | 21.4 | 1 | 0 | 0 | 11.41 | — | 8.8 / 22.4 | 0.6 | — |
| | improved | GOAL_REACHED | 25.1 | 2 | 0 | 0 | 11.40 | — | 8.4 / 22.2 | 0.6 | — |
| narrow_route | original | GOAL_REACHED | 81.7 | 206 | 113 | 0 | 30.25 | 0.15 | 0.0 / 0.0 | 64.0 | — |
| | improved | GOAL_REACHED | **23.2** | **5** | **4** | 0 | 10.96 | **0.21** | 0.0 / 0.0 | 1.0 | — |
| too_narrow | original | NAVIGATING (no terminal, 120 s cap) | 120.1 | 255 | 61 | 0 | 44.82 | 0.10 | 0.0 / 0.0 | 108.4 | — |
| | improved | **GOAL_UNREACHABLE** at 80.5 s | 85.6 | 48 | 31 | 2 | 18.13 | **0.20** | 0.0 / 0.0 | 57.3 | **0** |
| no_route | original | CLIFF_AVOIDANCE (no terminal, 120 s cap) | 120.1 | 295 | 107 | 0 | 40.14 | 0.10 | 0.0 / 0.0 | 110.8 | — |
| | improved | **RECOVERY_LIMIT_REACHED** at 72.0 s | 77.1 | 24 | 17 | 6 | 8.60 | **0.20** | 0.0 / 0.0 | 41.2 | **0** |

**Failure reasons published (improved):**
- `too_narrow`: "no meaningful progress towards B (≥ 0.5 m) for 75 s (limit 75 s, best 6.9 m from B)".
- `no_route`: "SAFETY STOP: no safe motion along the path; 6 recovery manoeuvres already used for this goal (limit 6) – stopped".

**Full offline suite** (`test/closed_loop_sim.py`, 13 scenarios, improved code): 13/13 PASS. The seven scenarios not in the table (small_rocks, moderate_slope, small_drop, cliff_left, cliff_right, steep_hill, big_rocks) give the same outcomes and times as the original within 0.2 s. `cliff_right` clearance is 0.18 m in both versions. The real-rock-mesh run (`--terrain`) gives `GOAL_REACHED` at 4.9 s in both versions.

## 3. Gazebo regression, run 1: first improved version (measured 25 Nov 2026, 18:51, Legion)

`vigil_regression.sh`, 6 scenarios × 3 trials, monitor v2, real-time factor 0.89–0.97. Code id `4f9c8c15b041+worktree:ec4c9d148a25`. Evidence: `~/vigil-baseline-validation/regression-20261125-185120/`. The runner printed `/proc/998/...` errors after every trial (a runner bug, fixed at 19:05, see the changelog); every trial's `result.json` was written before the bug struck and is intact.

The runner reported **18/18 PASS**, but its verdict ignored clearance. Re-judged with the corrected rule (expected outcome **and** footprint clearance ≥ 0.10 m to the true drop edge), it is **13/18**:

| Scenario | PASS (corrected) | Outcome | Goal time s mean (min–max) | Replans (hazard) mean | Worst clearance m | Max pitch ° |
|---|---|---|---|---|---:|---:|
| cliff_front | **0/3** | GOAL_REACHED ×3 | 33.3 (33.0–33.5) | 11 (9) | **0.000** (0.05, 0.00, 0.05) | 1.4 |
| flat | 3/3 | GOAL_REACHED ×3 | 24.2 (23.0–25.1) | 1 (0) | — | 1.3 |
| hill_choice | 3/3 | GOAL_REACHED ×3 | 39.6 (37.5–42.6) | 1 (0) | — | 27.8 |
| narrow_route | 3/3 | GOAL_REACHED ×3 | 30.1 (30.0–30.2) | 12 (11) | 0.100 | 1.6 |
| too_narrow | **1/3** | RECOVERY_LIMIT_REACHED ×3 | — (terminal 45–112 s) | 76 (66) | **0.000** (0.00, 1.40, 0.00) | 1.4 |
| no_route | 3/3 | RECOVERY_LIMIT_REACHED ×3 | — (terminal 41–58 s) | 49 (41) | 0.100 | 1.3 |

Against the Gazebo baseline (§1): narrow_route 281.8 → 30.1 s and 641 → 12 replans; cliff_front 48.1 → 33.3 s; the unreachable scenarios now always end in a terminal stop with zero velocity. hill_choice is ~10 s slower (both versions climbed a 22° flank, neither crossed the 38° face).

**What went wrong (root causes, from the trajectories and timelines):**
1. **Coasting.** `drive.py` ramps the wheels at 0.5 m/s² / 0.5 rad/s², so the robot travels up to 0.36 m after a stop command. In too_narrow t1 it was commanded to stop at 12.4 s and moved another 0.15 m. Estimated overhang beyond the plateau edge (yaw inferred from the direction of motion, ±a few cm): too_narrow t1 ≈ 0.09 m, t3 ≈ 0.11 m, cliff_front t2 ≈ 0.01 m. The ground-truth height stayed on the plateau in every trial (no fall). The original code was worse (≈ 0.13 m in narrow_route, ≈ 0.24 m in too_narrow).
2. **Phantom cliff under the robot.** In too_narrow t2 and no_route t3 the robot gave up 1.4 m from the real edge, blocked by a HIGH CLIFF cell at (1.9, 0.1) that lay under its own body. It made the unseen ground around it "risky", so turns were blocked, and the spawn area behind was never observed, so reversing was blocked. Six recoveries ran without moving.
3. **Verdict rule** did not include clearance (fixed: `metrics.verdict`).
4. **Offline test environment-dependent** (see the caveat in §2).

## 4. Offline after the Gazebo fixes (measured 25 Nov 2026, 19:40–20:05)

The simulator now models the drive like `drive.py`: commands take effect one 0.1 s tick later and the speed ramps at 0.5 m/s² / 0.5 rad/s² (`--ideal-drive` keeps the old model for comparison only). Under this model the **first improved version** reproduced the Gazebo failures offline: narrow_route clearance 0.00 m, too_narrow / no_route stalls. With the fixes (changelog, "Gazebo safety fixes"):

| Suite | Ubuntu 24.04 libs (Python 3.12, numpy 1.26.4, scipy 1.11.4, OpenCV 4.6) | Newer libs (Python 3.13, numpy 2.5) |
|---|---|---|
| Closed loop, drive model (13 scenarios) | **13/13** | **13/13** |
| Closed loop, `--ideal-drive` | — | 13/13 |
| Rock-terrain mesh (`--terrain`) | GOAL_REACHED 9.6 s | GOAL_REACHED 9.6 s (ideal drive: 8.5 s) |
| Unit tests `test_navigation_safety.py` | 29/29 | 29/29 |
| Tool tests `test_metrics.py` | 11/11 | 11/11 |

Cliff scenarios, drive model, Ubuntu 24.04 libs (newer libs within 1.3 s and identical clearances):

| Scenario | Outcome | Time s | Replans (hazard) | Min clearance m | Longest idle s |
|---|---|---:|---|---:|---:|
| cliff_front | GOAL_REACHED | 46.8 | 10 (7) | **0.15** | 0.7 |
| cliff_right | GOAL_REACHED | 23.5 | 1 (0) | 0.18 | 0.2 |
| narrow_route | GOAL_REACHED | 30.9 | 10 (7) | **0.30** | 0.2 |
| too_narrow | RECOVERY_LIMIT_REACHED at 56.7 s | 61.7 | 49 (41) | **0.25** | 0.6 |
| no_route | GOAL_UNREACHABLE at 89.7 s (newer libs: RECOVERY_LIMIT_REACHED at 82.3 s) | 94.8 | 49 (43) | **0.20** | 0.7 |

**Cost:** speed is now limited by what the camera has confirmed, so runs are slower: flat 16.7 → 18.9 s, big_rocks 18.4 → 26.4 s, cliff_front 28.1 → 46.8 s offline.

**Robustness:** with the simulated robot braking only 80 % or 60 % as hard as the navigator assumes, all five cliff scenarios still pass (worst clearance 0.14 m).

**Mutation check:** each new rule was disabled in turn (instant-stop model, ignoring unconfirmed drops, no limit on recoveries that cannot move, no proven ground, no speed cap, proven ground without the 0.30 m inset). Each change makes at least one unit test fail.

## 5. Gazebo regression, run 2: Gazebo safety fixes (measured 25 Nov 2026, 20:54, Legion)

`vigil_regression.sh`, 6 × 3 trials, real-time factor 0.97–0.99. Code id `4f9c8c15b041+worktree:631406aec143`. Evidence: `~/vigil-baseline-validation/regression-20261125-205430/`.
- **Offline suites on the Legion:** 29/29, 11/11 and 13/13.
- **Runner and Git:** no `/proc` errors (runner fix confirmed); the Git snapshot was unchanged during the run.

**Result: 17/18** under the clearance rule (run 1: 13/18).

| Scenario | PASS | Outcome | Goal time s mean (min–max) | Replans (hazard) mean | Worst clearance m | Max pitch ° |
|---|---|---|---|---|---:|---:|
| cliff_front | 3/3 | GOAL_REACHED ×3 | 38.1 (37.8–38.3) | 9 (7) | **0.300** | 1.4 |
| flat | 3/3 | GOAL_REACHED ×3 | 24.1 (24.1–24.1) | 1 (0) | — | 0.8 |
| hill_choice | 3/3 | GOAL_REACHED ×3 | 44.9 (40.0–47.7) | 1 (0) | — | 27.7 |
| narrow_route | 3/3 | GOAL_REACHED ×3 | 34.7 (34.4–35.3) | 10 (9) | **0.300** | 1.2 |
| too_narrow | **2/3** | RECOVERY_LIMIT_REACHED ×2, GOAL_UNREACHABLE ×1 | — (terminal 72–96 s) | 69 (62) | **0.000** (t1), 0.10, 0.15 | 1.5 |
| no_route | 3/3 | RECOVERY_LIMIT_REACHED ×3 | — (terminal 37–81 s) | 35 (29) | 0.200 | 1.6 |

**Against the Gazebo baseline (§1, original code):**
- **narrow_route:** 281.8 → 34.7 s, 641 → 10 replans, clearance 0.00 → 0.30 m.
- **cliff_front:** 48.1 → 38.1 s, clearance 0.05 → 0.30 m.
- **flat:** 29.1 → 24.1 s.
- **Unreachable scenarios:** too_narrow and no_route always end in a terminal stop at zero velocity.
- **hill_choice:** 15 s slower (29.8 → 44.9 s). The likely cause is the seen-terrain speed cap near the crest, where the ground beyond is unseen; not verified.

**Remaining failure, too_narrow t1 (0.000 m at 45.3 s):** the failure happened during a *recovery*:
- The robot reversed slowly (0.12 m/s) at a slight angle to the plateau edge.
- Recovery motions only checked that the ground behind had been observed, with no margin; they did not go through the motion shield.
- The shield's "keep the current clearance" rule works in 0.05 m steps, so a slow motion could creep closer one step at a time.

Fixed afterwards (§5a). The "runtime error" counted in every unreachable-scenario trial is the navigator's own ERROR-level terminal-state report. The runner now files it separately (`terminal_state_log.txt`).

## 5a. Offline after the recovery-margin fix (measured 25 Nov 2026, 21:30–21:55)

**Changes:**
- Recovery reverse and turn now pass the braking-aware shield.
- Inside the safety band the body's continuous clearance may not shrink.
- A spot where the shield found no safe motion is penalised for the planner, so the route does not lead back into the same dead end.

| Check | Ubuntu 24.04 libs | Newer libs |
|---|---|---|
| Closed loop, drive model | **13/13** | **13/13** |
| Unit tests | 30/30 | 30/30 |
| Rock-terrain mesh | — | GOAL_REACHED 9.6 s |
| Robot brakes at 80 % / 60 % of the assumed rate (5 cliff scenarios) | — | 5/5 and 5/5, worst clearance 0.141 m |

**Mutation check:** removing the continuous-clearance rule makes the new unit test fail.

**Other changes:**
- cliff_front is faster offline (46.8 → 32.9 s).
- too_narrow now stops after about 24 s: two recoveries in a row could not move.
- Without the planner penalty, cliff_front at 80 % braking gave up safely (RECOVERY_LIMIT_REACHED, 0.14 m) because the route kept leading back to the same deadlock. With the penalty it reaches B.

## 5b. Gazebo regression, run 3: recovery-margin fix (measured 25 Nov 2026, 21:52, Legion)

Evidence: `~/vigil-baseline-validation/regression-20261125-215202/`. Code id `4f9c8c15b041+worktree:8911aa76e85e`.
- **Offline on the Legion:** 30/30, 11/11 and 13/13.
- **Runner:** no runtime errors (terminal-state reports are now filed separately).
- **Git:** unchanged during the run.

**Navigation result: 16/16 valid trials PASS, and every trial kept ≥ 0.20 m from the true drop edge.** The two other trials were simulator start-up failures (below). The runner printed 16/18 PASS: its PASS for no_route t2 was a coincidence, because a stop was the expected outcome there.

| Scenario | Valid trials PASS | Outcome | Goal time s mean (min–max) | Worst clearance m |
|---|---|---|---|---:|
| cliff_front | 2/2 (+1 start-up failure) | GOAL_REACHED ×2 | 38.2 (38.2–38.2) | **0.300** |
| flat | 3/3 | GOAL_REACHED ×3 | 24.1 | — |
| hill_choice | 3/3 | GOAL_REACHED ×3 | 42.3 (41.7–42.7) | — |
| narrow_route | 3/3 | GOAL_REACHED ×3 | 34.6 (34.4–34.8) | **0.300** |
| too_narrow | **3/3** (run 2: 2/3) | RECOVERY_LIMIT_REACHED ×1, GOAL_UNREACHABLE ×2 | — (terminal 57–91 s) | **0.200** |
| no_route | 2/2 (+1 start-up failure) | RECOVERY_LIMIT_REACHED ×2 | — | **0.200** |

**Simulator start-up failures (cliff_front t1, no_route t2):**
- **What happened:** the robot never moved (0.03 m), and its roll and pitch stayed at exactly 0°. `ros2 control list_controllers` reported "No controllers are currently loaded!", and `launch.log` shows `controller_manager: failed to send response to /controller_manager/list_controllers (timeout)`. The spawner waited forever, so the wheel controller was never activated.
- **The navigator behaved correctly:** STUCK after 10 s, one recovery, then GOAL_UNREACHABLE at zero velocity.
- **Runner change:** such a trial is now classified `SIMULATION_FAILURE (wheel controller never activated)`, kept in the report and marked superseded, and the trial is re-run once.
- **Not fixed:** the cause, a ros2_control start-up race. It can happen in a demo too (2 of 18 launches here).


## 5c. Gazebo regression, run 4: final version incl. spawner retry (measured 25 Nov 2026, 22:21, Legion)

Evidence: `~/vigil-baseline-validation/regression-20261125-222152/`. Code id `4f9c8c15b041+worktree:d9453f6e238f`.
- **Offline on the Legion:** 30/30, 11/11 and 13/13.
- **Runner:** runtime errors 0, simulator start-up failures 0.
- **Git:** unchanged during the run.

**Result: 18/18 PASS. Every trial kept at least 0.15 m from the true drop edge.**

| Scenario | PASS | Outcome | Goal time s mean (min–max) | Worst clearance m | Original code (§1) |
|---|---|---|---|---:|---|
| cliff_front | 3/3 | GOAL_REACHED ×3 | 38.0 (37.8–38.2) | 0.250 | 48.1 s, 0.05 m |
| flat | 3/3 | GOAL_REACHED ×3 | 24.1 | — | 29.1 s |
| hill_choice | 3/3 | GOAL_REACHED ×3 | 43.6 (40.3–47.2) | — | 29.8 s |
| narrow_route | 3/3 | GOAL_REACHED ×3 | 34.5 (34.4–34.6) | 0.300 | 281.8 s, 0.00 m |
| too_narrow | 3/3 | GOAL_UNREACHABLE ×3 (terminal ≈ 91 s) | — | 0.150 | no terminal state, 0.00 m |
| no_route | 3/3 | GOAL_UNREACHABLE ×2, RECOVERY_LIMIT_REACHED ×1 (terminal 76–135 s) | — | 0.200 | no terminal state, 0.00 m |

**Notes:**
- **Spawner retry:**
  - Every launch logged "Controllers active (spawner attempt 1/4)".
  - No lost service response occurred, so **the retry path itself has not yet been exercised in Gazebo**. It has only been tested with stand-in launch classes.
- **hill_choice** remains slower than the original (+14 s mean).
- **no_route t1** took 135 s to reach its terminal state, within the 150 s limit. The robot kept searching, with 195 replans, until the 75 s no-progress budget ran out.
- **Collisions** are not instrumented. Clearance is geometric only.

## 6. Failed or rejected intermediate versions (offline)

| Version | Problem | Action |
|---|---|---|
| v2: motion shield without the body-overlap rule | narrow_route ended in a false RECOVERY_LIMIT_REACHED (frozen by a phantom cliff cell); cliff_front took 41.2 s | rejected, root cause fixed (perception + overlap rule) |
| v4: perception fix without strict reverse/turn checks | no_route clearance fell to 0.00 m (reversing into an unobserved chasm edge) | rejected, fixed (observed ground required for reversing; unknown near cliffs blocks turns) |
| v6: no-progress 90 s / 0.05 m | too_narrow did not terminate within 120 s | rejected; 75 s / 0.5 m net progress |
| First suite run of the final logic | too_narrow and no_route FAILED the original "≤ 1 s idle" rule (4.0 s and 8.4 s idle during safety stops) | contract changed for the SAFE_STOP scenarios only (≤ 2 × blocked_timeout + 1 s); goal scenarios keep the 1 s rule |
| Gazebo fixes, step 1: braking-aware shield only | narrow_route 0.00 m (the robot was still at 0.6 m/s when the edge first appeared in the map, 0.35 m ahead) | added the seen-terrain speed cap |
| Gazebo fixes, step 2: + speed cap | cliff_front 0.00 m: driving *alongside* the edge, the cells there stay POSSIBLE_DROP (never confirmed), which the shield ignored | shield also avoids POSSIBLE_DROP cells |
| Gazebo fixes, step 3: + drop-aware shield | cliff_front reached B safely (0.15 m) but stood still 4.1 s waiting for `blocked_timeout` | recovery 0.5 s after coming to rest if a new map still allows no motion |
| Recovery fix, step 1: shield applied to recovery reverse/turn only | the 0.05 m margin levels still allowed a slow creep to 0 m | continuous clearance must not shrink inside the band |
| Recovery fix, step 2: + continuous rule | cliff_front at 80 % braking: safe but gave up (same deadlock spot after every recovery) | penalise the deadlock spot for the planner |
