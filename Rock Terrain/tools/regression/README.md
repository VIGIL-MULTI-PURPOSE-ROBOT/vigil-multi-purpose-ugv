# Rock Terrain regression tools

Repeatable validation of `vigil_rough_terrain`: offline tests, build, and Gazebo scenarios × trials.
See [docs/REGRESSION_RESULTS.md](../../../docs/REGRESSION_RESULTS.md) for recorded results.

| File | Purpose |
|---|---|
| `vigil_regression.sh` | Runs everything and writes evidence to `/tmp/vigil-regression/<stamp>/`, copied to `~/vigil-baseline-validation/regression-<stamp>/` |
| `monitor.py` | Read-only ROS 2 observer for one run. Writes `result.json`, `timeline.jsonl` and `trajectory.csv` |
| `metrics.py` | Path length, progress, rates, real-time factor, roll/pitch and outcome classification (no ROS) |
| `test_metrics.py` | Unit tests for `metrics.py` |
| `summarize.py` | Builds `SUMMARY.md` (every run, per-scenario aggregate, baseline comparison, Git check) |

## Run

From a graphical terminal on the Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic machine:

```bash
cd ~/sih26126_ws/vigil-multi-purpose-ugv          # your clone
bash "Rock Terrain/tools/regression/vigil_regression.sh"                       # 6 scenarios x 3 trials
bash "Rock Terrain/tools/regression/vigil_regression.sh" --trials 1 --scenarios "cliff_front narrow_route"
```

Options: `--trials N`, `--scenarios "..."`, `--no-gui`, `--light-camera`, `--no-offline`,
`--baseline DIR`, `--branch NAME`. A full 6 × 3 run takes about 60–90 minutes of wall time.

By default the runner tests whatever branch is checked out and records the branch, commit and uncommitted
changes in the evidence. `--branch NAME` adds a hard check: if the checkout is not on `NAME`, the runner
stops with exit code 3 before building or launching anything.

## What it guarantees

- **Git:** no writing Git command is run. Branch, HEAD, `main`, stash, status and content hashes are recorded before and after, and any change is reported.
- **Process safety:** each launch gets its own process group and a unique `GZ_PARTITION` with `cleanup:=false`. Only those processes are stopped. The run refuses to start if port 8080 or `ROS_DOMAIN_ID=71` is already in use.
- **Clearance is a pass criterion:** in the cliff scenarios a run PASSes only if the footprint stayed at least 0.10 m from the true drop edge (`metrics.MIN_CLEARANCE`; 0.000 = the footprint overhung the edge). `summarize.py` recomputes every verdict with this rule and stars runs recorded under the older rule.
- **Honest outcomes:** a timeout is never a PASS. Every run is classified as `GOAL_REACHED`, `SAFE_STOP`, `NAVIGATION_FAILURE`, `TIMEOUT`, `SIMULATION_FAILURE` or `MONITOR_FAILURE`. `SAFE_STOP` additionally requires that `/cmd_vel` stays zero and the ground-truth speed stays ≤ 0.05 m/s after the terminal state.
- **Separated messages:** launch-log messages before the stop (runtime) are reported separately from those after it (shutdown).
- **No silent failures:** screenshot failures and dashboard collection failures are recorded per run.
- **No collision claim:** body collisions are not instrumented. "Minimum clearance" is geometric (ground-truth pose against the scenario's true drop edges).
