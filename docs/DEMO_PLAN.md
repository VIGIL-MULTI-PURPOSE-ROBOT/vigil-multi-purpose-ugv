# VIGIL 2–3 minute demo plan

[Project audit](PROJECT_AUDIT.md) · [Validation report](VALIDATION_REPORT.md) · [Setup](setup.md)

This plan uses the **Rock Terrain** workspace, which shows the clearest camera → terrain map → planner → wheels loop for SIH26126. Every command below matches the package's actual launch file, arguments and topics. **This exact sequence has not been rehearsed or recorded as part of the audit.** Do a full dry run on the demo machine first. The YouTube video linked in the README is a separate concept and simulation video, not a recording of this plan.

## Before the session (not part of the 3 minutes)

On Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Harmonic and a working GPU/display, follow [setup](setup.md), then:

```bash
cd "Rock Terrain"                                   # from the repository root
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_DOMAIN_ID=71
```

- Run the live demo command once beforehand, so Gazebo has cached the world and models, then stop it with Ctrl+C.
- Open a second terminal with the same `source` and `export` lines, for the topic commands.
- Open a browser tab at `http://localhost:8080` (it loads once the launch is running).
- Pre-generate the fallback images (see Fallback B), so they exist even if nothing else works.

## Live sequence

| Time | Action | What to say (accurate scope) |
|---|---|---|
| 0:00 | `ros2 launch vigil_rough_terrain vision_nav.launch.py scenario:=cliff_front` | "An eight-wheel rover with rocker suspension, on a 1.5 m plateau with a pit on the straight line to B." |
| 0:00–0:30 | Gazebo window opens; the rover spawns and settles (`start_delay` 6 s of simulation time) | "Wheel controllers start, then the navigator waits for the rover to settle." |
| 0:30 | Switch to the dashboard at `http://localhost:8080` | "Left: depth-camera overlay. Right: the terrain map built from depth, with slope, roughness, steps and drops checked against the rover's footprint and wheel size." |
| 0:45 | The rover drives; the pit appears in the overlay and the map; the state changes to `CLIFF AHEAD` / `CLIFF_AVOIDANCE`, then `SAFE PATH FOUND` | "The drop is larger than the 0.17 m wheel-radius limit, so it becomes a hard hazard and the A* path is replanned around it." |
| 1:30 | In the second terminal: `ros2 topic echo /navigation/status --once` and `ros2 topic echo /vision/cliff --once` | Shows the raw state, pose, replans and cliff summary behind the dashboard |
| 1:45 | The rover reaches B, and the dashboard shows `GOAL_REACHED` | "Goal reached. Position comes from the simulator's ground truth. Terrain perception is from the depth camera only. Visual localization in this loop is our next milestone." |
| 2:15 | (Optional) send a new goal: `ros2 topic pub --once /navigation/goal geometry_msgs/msg/PoseStamped "{header: {frame_id: world}, pose: {position: {x: 2.0, y: 2.5}}}"` | Shows that a goal can be set at runtime; `/goal_pose` from RViz also works |
| 2:45 | Ctrl+C | The launch stops the wheels on shutdown |

Say plainly that this is **simulation with simulator localization**, and that New Mining is where an RGB-D visual-odometry + RTAB-Map + EKF pose drives navigation. If someone asks about avoiding moving obstacles, the honest answer is that it's implemented and tested offline in the SAR workspace, not in this demo.

**Lighter display load:** if the machine struggles, add `camera_width:=960 camera_height:=720 camera_rate:=5` to the launch command.

**Default world instead of the cliff test:** `ros2 launch vigil_rough_terrain vision_nav.launch.py` (rock terrain, A = (−5.4, −3.6), B = (−3.024, −2.16)). This is the configuration that reported `GOAL_REACHED` in the 27 September capture ([validation](validation.md)). The route is short, so the replanning story is weaker.

## Fallback A: Gazebo will not start or render

Show the recorded evidence from the 27 September capture: `docs/media/cliff-dashboard.png`, `docs/media/rough-terrain-dashboard.png` and the raw status in `docs/validation/*.json`. Describe them as captures, not as a live run.

## Fallback B: no ROS at all (any machine with Python 3, numpy, scipy, opencv, pyyaml)

The same perception and planning code runs against a synthetic depth renderer:

```bash
cd "Rock Terrain/src/vigil_rough_terrain"
python3 test/closed_loop_sim.py cliff_front --png /tmp/vigil_demo     # ~10 s; prints JSON + PASS/FAIL
python3 test/closed_loop_sim.py hill_choice --png /tmp/vigil_demo     # terrain-aware hill choice
```

Show the printed outcome and the saved map images. Say what this is: an **offline test with idealized depth and perfect pose**, which checks the algorithm but is not physics simulation. Expected outcomes, measured on 25 Nov 2026, are in the [validation report](VALIDATION_REPORT.md).

## Do not claim in the demo

- GPS-denied visual SLAM in the rough-terrain loop (pose is ground truth)
- Learned/AI terrain segmentation (classification is geometric)
- Validated collision avoidance or repeated-trial success rates (only single runs are recorded)
- Mining gas sensing (New Mining's gas values are synthetic)
- Real hardware operation
