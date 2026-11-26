# VIGIL Rock Terrain regression run

- Code under test: `f1d715851bc5+worktree:e3b0c44298fc`
- Environment: Ubuntu 24.04.4 LTS | 7.0.0-43-generic | NVIDIA GeForce RTX 5070 Laptop GPU, 595.99.02 | session=wayland DISPLAY=:0 | /dev/nvme0n1p5  291G  241G   35G  88% / | ROS_DISTRO=jazzy | 8.11.0 | Python 3.12.3
- Offline tests (0 = pass): skipped
- Build: build exit code: 0
- Collision instrumentation: **none** (minimum clearance is geometric: ground-truth pose vs. scenario geometry). No collision-free claim is made.
- Verdict rule: the expected outcome AND, in the cliff scenarios, a footprint clearance of at least 0.10 m to the true drop edge (0.000 = the footprint overhung the edge). Verdicts are recomputed with this rule; runs recorded under the older rule are marked *.

## Every run

| Scenario | Trial | Category | Verdict | Final state | Failure reason | Goal time (sim s) | Sim s | Wall s | RTF | Path m | Replans | Hazard replans | Recoveries | Min clearance m | Max roll ° | Max pitch ° | Longest no-progress s | Depth Hz (sim) | cmd_vel Hz (sim) | Runtime errors | Shutdown msgs | Screenshots |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| flat | 1 | GOAL_REACHED | **PASS** | GOAL_REACHED |  | 24.1 | 27.2 | 30.0 | 0.971 | 10.20 | 1 | 0 | 0 | n/a | 2.8 | 0.8 | 2.5 | 10.0 | 10.0 | 0 | 8 | 0/2 ok |

## Per scenario (mean (min-max) over trials)

| Scenario | Trials | PASS | Categories | Goal time s | Path m | Replans | Hazard replans | Recoveries | Min clearance m (worst) | Max roll ° | Max pitch ° | RTF |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| flat | 1 | 1/1 | GOAL_REACHED | 24.1 | 10.20 | 1 | 0 | 0 | n/a | 2.8 | 0.8 | 0.971 |

## Comparison with the baseline (`/home/user/vigil-baseline-validation/20261125-170337`, original code, monitor v1, 1 trial)

| Scenario | Baseline outcome | Baseline goal s | Improved goal s (mean) | Baseline replans / hazard | Improved replans / hazard (mean) | Baseline min clearance m | Improved worst clearance m |
|---|---|---|---|---|---|---|---|
| cliff_front | GOAL_REACHED | 48.1 | n/a | 116 / 50 | n/a / n/a | 0.050 | n/a |
| flat | GOAL_REACHED | 29.1 | 24.1 | 3 / 0 | 1 / 0 | n/a | n/a |
| hill_choice | GOAL_REACHED | 29.8 | n/a | 3 / 1 | n/a / n/a | n/a | n/a |
| narrow_route | GOAL_REACHED | 281.8 | n/a | 641 / 135 | n/a / n/a | 0.000 | n/a |
| too_narrow | no terminal state (RECOVERY) | n/a | n/a | 367 / 34 | n/a / n/a | 0.000 | n/a |
| no_route | no terminal state (CLIFF_AVOIDANCE) | n/a | n/a | 339 / 16 | n/a / n/a | 0.000 | n/a |

Baseline path length is not shown: monitor v1 over-counted it (see tools/regression/metrics.py). Baseline recoveries were not recorded.

## Git verification

- branch.txt: unchanged during the run
- head.txt: unchanged during the run
- main.txt: unchanged during the run
- branches.txt: unchanged during the run
- stash.txt: unchanged during the run
- status_porcelain.txt: unchanged during the run
- diff_tracked.sha256: unchanged during the run
- diff_files.sha256: unchanged during the run
- diff_summary.txt: unchanged during the run
- untracked.sha256: unchanged during the run

```
vigil-ai-gps-denied
exit=0
```

## Notes

```
branch check: none requested; running on 'vigil-ai-gps-denied'
build exit code: 0
simulator start-up failures retried: 0
truth isolation: false, evaluator: true, bag: yes
```
