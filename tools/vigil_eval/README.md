# VIGIL GPS-denied evaluation tools

Independent measurement of localization and the guard that keeps simulator truth away from the
robot. Nothing here steers the robot; nothing here may be imported by an operational node.

| File | Purpose | Needs ROS |
|---|---|---|
| `loc_metrics.py` | ATE, RPE, drift, heading error, availability from time-associated samples | No |
| `guard_core.py` | Decides from a ROS-graph snapshot whether simulator truth can reach the robot | No |
| `evaluator.py` | Isolated evaluator node: reads `/eval/ground_truth`, looks up the estimate in TF, runs the guard every second, writes the reports | Yes |
| `guard_selftest.py` | Proves the guard catches a rogue truth reader and republished truth | Yes (no Gazebo) |
| `truth_scan.py` | Static scan: which source files can read simulator truth | No |
| `test_vigil_eval.py` | Offline tests of the three ROS-free modules | No |

## Truth isolation

`vision_nav.launch.py truth_isolation:=true` (Rock Terrain) bridges `/sim/ground_truth` and
`/sim/link_poses` only as `/eval/ground_truth` and `/eval/link_poses`
(`config/bridge_truth_isolated.yaml`), and never bridges the segmentation camera, a simulator oracle.
Default `false` = unchanged behaviour.

The guard reports, per run (`guard.json`):

| Verdict | Meaning |
|---|---|
| `ISOLATED` | isolation mode; only allowed evaluation nodes read `/eval/*`; no truth on operational topics |
| `VIOLATION` | isolation mode and a truth path exists (listed) |
| `GROUND_TRUTH_ASSISTED` | baseline mode; lists the nodes that used simulator truth |

Allowed readers of `/eval/*`: `/vigil_evaluator`, `/vigil_regression_monitor`, `/rosbag2_recorder`,
`/_ros2cli_*` (a person typing `ros2 topic echo`). Wheel-contact topics carry world-frame contact
positions and are also watched. **Limit:** the guard sees the ROS graph only. A node that opened Gazebo
transport directly would be invisible to it; `truth_scan.py` covers that at source level
(`gz.transport` imports).

## Metric definitions

- **ATE (start-aligned):** RMSE of position error after aligning the estimate to the truth at the first
  sample only. This is the acceptance metric, because it keeps the drift the robot actually suffered.
- **ATE (SE(2)-aligned):** best-fit alignment of the whole trajectory, for comparison with the literature
  only. It hides drift, and a unit test demonstrates this.
- **RPE over 1, 2 and 5 m:** translational error of relative motion over ground-truth path segments.
- **Drift:** final start-aligned error divided by path length, in %.
- **Heading error:** after start alignment, wrapped; RMS and max.
- **Availability:** share of truth samples that had an estimate; longest gap in seconds.

A metric that cannot be computed is `null`, never 0.

## Commands

```bash
python3 -m pytest -q tools/vigil_eval                    # offline tests
python3 tools/vigil_eval/truth_scan.py                   # source-level truth users
source /opt/ros/jazzy/setup.bash
python3 tools/vigil_eval/guard_selftest.py               # guard against a live graph
# Gazebo, via the Rock Terrain runner (from the repository root):
bash "Rock Terrain/tools/regression/vigil_regression.sh" --eval --no-offline --trials 1 --scenarios flat
bash "Rock Terrain/tools/regression/vigil_regression.sh" --truth-isolation --eval --no-offline --trials 1 --scenarios flat
```

Per trial (with `--eval`), next to the runner's usual files: `eval_report.json`, `guard.json`,
`eval_trajectory.csv`, `evaluator.log`, and `bag/` (ROS 2 bag of truth, TF, clock, navigation status,
`cmd_vel`, IMU, encoders, joint states, camera info, cliff reports; no images).
