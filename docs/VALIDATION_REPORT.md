# VIGIL validation report: offline and static checks

Date: 25 November 2026 · Commit audited: `4f9c8c1` plus the working-tree changes listed in [the audit](PROJECT_AUDIT.md#7-issues-found)

[Project audit](PROJECT_AUDIT.md) · [Demo plan](DEMO_PLAN.md) · [Earlier runtime evidence](validation.md)

> **Later results:** this is the record of the 25 Nov 2026 audit run against the *original* code. The Rock Terrain navigator has since changed: `too_narrow` and `no_route` now end in a terminal safe stop instead of searching. See [REGRESSION_RESULTS.md](REGRESSION_RESULTS.md) for current results.

## Scope: read this first

**No ROS 2 or Gazebo run was performed for this report.** The audit machine had no ROS 2, no Gazebo and no GPU. Everything below is either a static check or an offline test that runs the project's own Python perception, planning and mission code against synthetic inputs, with **idealized sensing** (noise-free depth, perfect pose, kinematic motion without wheel slip or dynamics). These results show that the algorithms behave as designed. They are **not** evidence of robot behaviour in physics simulation or on hardware. The only live-Gazebo navigation evidence remains the single 27 September 2026 capture in [validation.md](validation.md).

## Environment

| Item | Value |
|---|---|
| OS | Ubuntu 24.04.5 LTS (container) |
| Python | 3.13 |
| Libraries | numpy 2.5.3, scipy 1.18.1, OpenCV 5.0.0, PyYAML, pytest 9.1.1 |
| Added for the audit only (not repository dependencies) | `xacro` and `pyflakes` from pip; a stub `ament_index_python` mapping package names to source folders, so xacro `$(find …)` resolves without ROS |
| ROS 2 / Gazebo / colcon | **not available** |

## Results

| # | Check | Command (from the repository root) | Outcome |
|---|---|---|---|
| 1 | Rough-terrain closed loop, 13 scenarios | `cd "Rock Terrain/src/vigil_rough_terrain" && python3 test/closed_loop_sim.py` | **13/13 PASS**, 2 m 58 s wall time |
| 2 | Same loop on the real rock-terrain mesh (default A → B) | `… python3 test/closed_loop_sim.py --terrain` | **PASS**: `GOAL_REACHED`, 4.9 s sim time, peak tilt 7.8°, 0 hazard replans |
| 3 | Agriculture unit tests | `cd Agriculture && python3 -m pytest -q tests` | **79 passed**, 20.5 s |
| 4 | SAR offline tests (baseline) | `cd "Military Search and Rescue" && python3 -m pytest -q -rs tests sar_ws/src/vigil_sar/test` | **107 passed, 2 skipped**, 6 m 20 s. Both skips: "vigil_rough_terrain … not found" (stale author path) |
| 5 | SAR offline tests (after the path fix) | same | **109 passed, 0 skipped**, 6 m 27 s. The two formerly skipped model-equivalence tests now run against `Rock Terrain/` and pass |
| 6 | New Mining navigation-core tests | `cd "New Mining" && python3 -m pytest -q src/vigil_new_mining/test_navigation.py` | **6 passed**, 0.2 s |
| 7 | Python syntax, all 126 files | `python3 -m py_compile <file>` for each | **0 errors** |
| 8 | pyflakes (undefined names, bad syntax use) | `python3 -m pyflakes <all .py>`; style-only warnings filtered out (unused imports/variables, redefinitions, f-strings without placeholders, star imports) | **0 remaining findings** |
| 9 | XML/SDF/xacro well-formedness, 62 files | `xmllint --noout` | 61 OK. 1 expected failure: `Agriculture/src/agri_ugv/config/gazebo_gui.config` (Gazebo GUI configs have several root elements by design) |
| 10 | YAML/JSON parse, 64 files | `yaml.safe_load` / `json.loads` | **0 errors** |
| 11 | Robot description expansion (4 workspaces, `simulation:=true`, `vision_sensors:=true`) | `xacro.process_file(…)` | **All expand.** One root (`base_footprint`), 0 broken joints, all mesh references resolve: Rock Terrain 33 links / 29 meshes, SAR 36 / 29, New Mining 33 / 29, Agriculture 26 / 21 |
| 12 | Launch-file executables vs. CMake install rules (6 packages) | audit script | Every `Node(executable=…)` of the package's own scripts is installed. Orphans: `Rock Terrain/.../suspension.py` (and its import `hydraulics.py`) |
| 13 | Relative Markdown links (all `.md` outside `.git`) | audit script | Before: **7 broken** (6 in `README.md`, 1 anchor in `vigil/README.md`). After fixes: **0** |
| 14 | Launcher menu, options 1–5 and an invalid option | `printf "N\n6\n" \| python3 vigil/master_launcher/vigil_launcher.py` | Before: option 4 said only "launch file not found"; option 5 went straight to the build check; build hints had unquoted paths with spaces. After: correct messages, and option 5 refuses to start without the licensed assets (same files as `New Mining/run.sh`) |
| 15 | Agriculture `tools/check_structure.py` | `python3 tools/check_structure.py` | **Not run:** needs a real ROS install (`ament_index_python`) |
| 16 | Rock Terrain `test/model_contract.py` | — | **Not run:** needs the `gz sdf` command from Gazebo |

### Per-scenario detail for check 1

Measured in the offline closed loop (0.1 s step, camera every 0.5 s, 0.60 m/s cruise). "Lateral range" is the robot's y-excursion; "over drop" counts steps with the body hanging over a drop (must be 0).

| Scenario | Expected | Outcome | Sim time (s) | Hazard replans | Lateral range y (m) | Peak tilt (°) | Over drop |
|---|---|---|---|---|---|---|---|
| flat | reach B | GOAL_REACHED | 16.7 | 0 | 0.00 … 0.09 | 0.0 | 0 |
| small_rocks | drive over | GOAL_REACHED | 16.8 | 0 | −0.05 … 0.22 | 4.4 | 0 |
| moderate_slope (15°) | climb | GOAL_REACHED | 16.7 | 0 | 0.00 … 0.09 | 15.1 | 0 |
| small_drop (12 cm) | drive off | GOAL_REACHED | 16.7 | 0 | 0.00 … 0.09 | 8.6 | 0 |
| cliff_front (1.5 m pit) | detour | GOAL_REACHED | 27.9 | 8 | −3.15 … 0.34 | 0.0 | 0 |
| cliff_left | keep course | GOAL_REACHED | 16.7 | 0 | 0.00 … 0.09 | 0.0 | 0 |
| cliff_right (notch) | shift left | GOAL_REACHED | 18.0 | 0 | 0.00 … 1.68 | 0.0 | 0 |
| narrow_route (2.0 m bridge) | find bridge | GOAL_REACHED | 81.7 | 113 | −2.71 … 2.90 | 0.0 | 0 |
| too_narrow (1.2 m bridge) | refuse, keep searching | NAVIGATING at 120 s cap | 120.1 | 61 | −2.73 … 2.84 | 0.0 | 0 |
| no_route | never fall, keep searching | CLIFF_AVOIDANCE at 120 s cap | 120.1 | 107 | −2.81 … 2.90 | 0.0 | 0 |
| steep_hill (28°) | climb slowly | GOAL_REACHED | 20.9 | 0 | 0.00 … 0.09 | 28.2 | 0 |
| hill_choice (38° face, 22° flanks) | climb a flank | GOAL_REACHED | 21.4 | 0 | 0.00 … 2.82 | 22.4 | 0 |
| big_rocks (15–21 cm) | treat as cost | GOAL_REACHED | 18.4 | 0 | 0.00 … 1.55 | 8.2 | 0 |

Each scenario ran once. The offline loop is deterministic for a fixed code version (seeded rocks, no noise), so repeating it adds no statistical information. Repeated trials matter in Gazebo, where they have not been done.

**What these results do and do not show:**
- They show that terrain costs and hazards change the plan: the detour in `cliff_front`, the flank in `hill_choice`, and rocks treated as cost in `big_rocks`.
- `too_narrow` and `no_route` "pass" by **never reaching a terminal failure state**. That is the intended "never give up" design, but it also shows that there is no bounded unreachable-goal report (see [audit §5](PROJECT_AUDIT.md#5-terrain-obstacles-and-planning-rock-terrain)).
- No collisions with moving obstacles, localization error or wheel slip are modelled in this test.

## Runtime scenarios not run (and why)

| Scenario | Command | Reason |
|---|---|---|
| Rock terrain A → B in Gazebo | `ros2 launch vigil_rough_terrain vision_nav.launch.py` | No ROS 2/Gazebo/GPU on the audit machine |
| Rough-terrain test worlds in Gazebo | `… vision_nav.launch.py scenario:=<name>` | Same |
| `colcon build` / `colcon test` for all packages | per workspace | Same |
| Agriculture row mission, Nav2 navigation, mapping | `cd Agriculture && ./run.sh …` | Same |
| SAR mission with walking people | `ros2 launch vigil_sar sar_mission.launch.py` | Same; it also needs the separately built people plugin |
| New Mining end-to-end | `cd "New Mining" && ./run.sh` | Same; it also needs licensed mine assets that are not in the repository |
| Localization accuracy (visual/EKF pose vs. ground truth) | — | No measurement tool exists yet |

## How to reproduce

The offline checks need only Python 3 with numpy, scipy, opencv-python and PyYAML (plus pytest for checks 3–6). Run them from a clone, using the commands in the table. The total run time is about 10 minutes, mostly the SAR suite (6 minutes) and the 13-scenario loop (3 minutes).

To record runtime evidence, run on Ubuntu 24.04 with ROS 2 Jazzy and Gazebo Harmonic. For each run, capture the world/scenario, commit, start/goal, pose source (currently `/sim/ground_truth`), final `/navigation/status`, time to `GOAL_REACHED`, path length, hazard replans and any intervention. Include failed runs.
