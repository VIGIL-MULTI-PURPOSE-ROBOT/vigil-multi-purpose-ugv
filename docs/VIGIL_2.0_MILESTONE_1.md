# VIGIL 2.0 — Milestone 1: ground-truth isolation and independent evaluation

**Status:** completed, 26 November 2026. **Branch:** `vigil-ai-gps-denied`.
**Code under test:** `f1d7158` (built on `0f44a87`, which builds on `a0a1543`). On GitHub the same two commits
appear as `3854b6b` and `51b415f`: they were re-applied unchanged on top of the README edits merged into
`vigil-ai-gps-denied` on 26 Nov 2026, which is why the hashes differ. The evidence files name `f1d7158`.
**Platform:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo 8.11, RTX 5070 laptop.

Milestone 1 shows that simulator truth can be cut off from the robot and still measured independently.
It does **not** show GPS-denied navigation: Rock Terrain has no pose estimator yet, so with truth
isolated the robot correctly refuses to move.

## A. Objective

Establish ground-truth isolation and independently evaluate the existing Rock Terrain navigation
system:

1. bridge the simulator truth only to an isolated evaluator;
2. detect any path by which truth could reach the robot;
3. measure localization against truth with metrics that do not hide drift;
4. record every trial reproducibly.

No navigation algorithm or safety parameter was changed.

## B. Test results

Evidence: [`docs/evidence/vigil2-milestone1/`](evidence/vigil2-milestone1/) (see section E).

| Check | Result |
|---|---|
| Offline tests (`tools/vigil_eval`) | **13/13 passed** |
| Guard self-test on a live ROS 2 graph | **3/3 passed**: rogue reader of `/eval/ground_truth` → VIOLATION; truth republished on `/sim/ground_truth` → VIOLATION; clean graph → ISOLATED |
| Source scan | 17 operational files can read a simulator-truth channel (report only) |
| E1 baseline flat, 1 trial | Goal reached in **24.1 s**, the same as every earlier run. Guard: **GROUND_TRUTH_ASSISTED** — `/cliff_navigator`, `/terrain_mapper` and `/ugv_dashboard` read `/sim/ground_truth`; `/terrain_mapper` reads `/camera/segmentation/labels_map` |
| E2 truth-isolated flat, 2 trials | Guard **ISOLATED** in both |
| E2 truth-isolated cliff_front, 1 trial | Guard **ISOLATED** |
| Guard findings | No violations and no warnings in any isolated trial; 35 checks (one per second) in flat trial 1 |
| Robot motion with truth isolated | Maximum ground-truth displacement **0.034 m** in all three trials: the robot settling after spawn, not driving |
| Navigator outcome with truth isolated | Safe stop with **LOCALIZATION_LOST** in all three trials |
| Evaluation reports | Written for all **four** trials (`eval_report.json`, `guard.json`, `eval_trajectory.csv`) |
| Helper shutdown | Evaluator and bag recorder stopped cleanly on SIGINT in all four trials; no forced stop logged |

In the regression runner's own verdicts the isolated trials appear as `SAFE_STOP [FAIL]`, because
that runner expects the goal to be reached. For milestone 1 the isolated trials passed the
**isolation checks** and showed **safe stopping**. They did not pass navigation, and were not
expected to.

## C. Interpretation

1. **The baseline still relies on simulator ground truth.** The guard names the three nodes that read
   `/sim/ground_truth`, and shows the mapper also uses Gazebo's segmentation camera (label 0 counts as
   drop evidence in `terrain_core.py` l.292–295).
2. **The isolated robot stops safely because Rock Terrain has no operational pose estimator.** The
   three subscriptions to `/sim/ground_truth` remain, but nothing publishes on it, so the navigator
   receives no pose and enters `LOCALIZATION_LOST`.
3. **Estimate availability was 0%** in all four trials. No TF `map→base_footprint` exists yet.
4. **Localization errors are unavailable, not zero.** ATE, RPE, drift and heading error are `null` in
   every report. The metric code never fills in 0 for a missing estimate (`loc_metrics.py`; tested).
5. **The guard detects ROS-graph-level leakage:**
   - subscribers to `/eval/*` outside the allowed evaluation nodes;
   - any publisher on `/sim/ground_truth` or `/sim/link_poses`;
   - a bridged segmentation oracle;
   - readers of wheel-contact topics (world-frame contact positions);
   - unexpected TF publishers (warning).
6. **Source scanning supplements the guard.** A node that opened Gazebo's own messaging directly would
   be invisible in the ROS graph. `truth_scan.py` searches the source for truth topics, `gz.transport`
   imports and world-file readers.

## D. Next milestone

**Milestone 2 — Rock Terrain localization: NOT STARTED.** Proposed scope, pending approval:

1. A configurable pose source for `terrain_mapper` and `cliff_navigator` (`ground_truth` by default,
   or `estimate` through a small adapter). The navigation algorithm stays unchanged.
2. Wheel odometry from the 8 wheel encoders.
3. IMU integration.
4. RGB-D visual odometry (RTAB-Map `rgbd_odometry`).
5. A 3D EKF (`robot_localization`), taking heading from turn rates, not from absolute IMU yaw.
6. RTAB-Map mapping and loop closure.
7. A textured flat world, with a test proving the terrain geometry is byte-identical.
8. The estimator running alongside the ground-truth baseline, to measure localization error at no risk.
9. GPS-denied navigation on the estimated pose (flat first, then narrow route, rough terrain, hill and
   cliff_front), under the guard.

## E. Evidence and reproduction

### Published evidence

`docs/evidence/vigil2-milestone1/`, about 320 kB:

| Path | Contents |
|---|---|
| `summary.txt` | Output of the milestone-1 validation script |
| `guard_selftest.txt` | Guard self-test output |
| `truth_scan.txt` | Source scan output |
| `E1_baseline_flat/` | Run notes, `SUMMARY.md`, `run.log`; per trial: command, code id, `result.json`, `eval_report.json`, `guard.json`, evaluator log, timeline, topic list, trajectory CSV, bag `metadata.yaml` |
| `E2_isolated_flat/` | Same, for 2 trials |
| `E2_isolated_cliff_front/` | Same, for 1 trial |

The trajectory CSVs were converted from CRLF to LF line endings when they were copied; their content
is otherwise unchanged. `evaluator.py` now writes LF directly.

### Not published

ROS bags (`bag_0.mcap`, 15.9–20.7 MB each, 4 trials), launch logs, dashboard images and Git
snapshots. The bags hold no evidence that the published files lack: the truth trajectory is in
`eval_trajectory.csv` and the message counts are in `bag_metadata.yaml`.

### Original evidence (kept on the test laptop)

- `~/vigil-baseline-validation/m1-20261126-062937/` (validation run)
- `~/vigil-baseline-validation/regression-20261126-063011/` (E1)
- `~/vigil-baseline-validation/regression-20261126-063105/` (E2 flat)
- `~/vigil-baseline-validation/regression-20261126-063308/` (E2 cliff_front)

An earlier attempt (`m1-20261126-062454`) is kept but superseded. In `0f44a87` the evaluator was never
stopped, so it wrote no report. `f1d7158` fixed this.

### Reproduce

From a checkout of `vigil-ai-gps-denied`, in a graphical terminal:

```bash
bash tools/vigil_eval/milestone1_validate.sh        # S1-S3 + E1 + E2, about 10-15 minutes
# or individually:
python3 -m pytest -q tools/vigil_eval
python3 tools/vigil_eval/truth_scan.py
source /opt/ros/jazzy/setup.bash && python3 tools/vigil_eval/guard_selftest.py
bash "Rock Terrain/tools/regression/vigil_regression.sh" --eval --no-offline --trials 1 --scenarios flat
bash "Rock Terrain/tools/regression/vigil_regression.sh" --truth-isolation --eval --no-offline --trials 2 --scenarios flat
```

### Files of milestone 1

| File | Role |
|---|---|
| `tools/vigil_eval/loc_metrics.py` | ATE (start-aligned and SE(2)), RPE, drift, heading error, availability |
| `tools/vigil_eval/guard_core.py` | Guard decision on a ROS-graph snapshot |
| `tools/vigil_eval/evaluator.py` | Isolated evaluator node; runs the guard every second; writes the reports |
| `tools/vigil_eval/guard_selftest.py` | Live-graph self-test of the guard |
| `tools/vigil_eval/truth_scan.py` | Static scan for truth channels |
| `tools/vigil_eval/test_vigil_eval.py` | 13 offline tests |
| `tools/vigil_eval/milestone1_validate.sh` | The validation script that produced this evidence |
| `Rock Terrain/src/vigil_rough_terrain/config/bridge_truth_isolated.yaml` | Bridge with truth only under `/eval/` |
| `Rock Terrain/src/vigil_rough_terrain/launch/{terrain_sim,vision_nav}.launch.py` | `truth_isolation` argument (default `false`) |
| `Rock Terrain/tools/regression/vigil_regression.sh`, `monitor.py` | `--truth-isolation`, `--eval`, `--no-bag`; configurable truth topic |
