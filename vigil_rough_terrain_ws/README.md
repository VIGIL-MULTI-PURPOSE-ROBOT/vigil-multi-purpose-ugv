# VIGIL rough-terrain rover and physical comparison

Standalone package; no agriculture package is required or modified. The original
terrain is now 18 × 18 m with half the elevation: mesh scale `1.8 1.8 1.44` (originally 25 × 25 m at `2.5 2.5 4`;
length and width ×0.72, height ×0.36, so cliffs and hills are half as high as at 18 m full height). The mesh files are unchanged.

## Build

```bash
cd /home/user/Documents/robot/vigil_rough_terrain_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_DOMAIN_ID=71
```
## Run only the original eight-wheel rover

```bash
ros2 launch vigil_rough_terrain terrain_sim.launch.py
```

In another terminal, source ROS, set `ROS_DOMAIN_ID=71`, then:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p repeat_rate:=10.0 -p key_timeout:=0.5
```

## Suspension and validation

Eight native passive prismatic wheel suspensions supplement the two articulated
side rockers. The CAD axle components move with their wheel carriers; overlapping
sleeves retain visible attachment. Wheel drive joints, body reference, tire sizes,
sensor mounts and terrain geometry are preserved. Springs are integrated by
Gazebo Harmonic/DART; there are no pose-reset or animation commands.

- Wheel travel: **220 mm**, limits **−100 / +120 mm**, velocity limit 2 m/s.
- Wheel springs: **8,000 N/m**, damping **650 N·s/m**; individual preload references
  account for the original asymmetric geometry and mass distribution.
- Rockers: limits **±0.20 rad**, springs **3,500 N·m/rad**, damping **300 N·m·s/rad**.
- Tire friction: `mu=mu2=0.85`; terrain friction remains 1.0.
- Physics timestep: 1 ms. Original wheel and payload masses are retained; chassis
  inertia is located at the chassis box centre; guides add 4.8 kg in total.
- Measured settled chassis-box clearance on a separate flat calibration fixture:
  **0.33754 m**, without increasing the chassis reference height.

LiDAR, RGB, depth, IMU, ground-truth odometry and moving-joint TF were received in
live tests. The standalone package did not contain a Nav2 launch; a full Nav2 goal
was not tested. The comparison uses its own ground-truth waypoint driver.
Contact sensors and bridges are configured, but contact telemetry was not
successfully validated; zero recorded contact depth is **not** evidence of zero
penetration. ODE-specific `kp/kd/minDepth/maxVel` tags are retained compatibility
settings, not validated DART contact tuning.

The short comparison succeeded for the eight-wheel rover. Earlier longer routes
hit steep features and exceeded the test driver's 38-degree tilt threshold.
Universal traversal and rollover resistance are **not** established. See
`src/vigil_rough_terrain/docs/VALIDATION.md` for results and limits.

```bash
colcon test --packages-select vigil_rough_terrain --event-handlers console_direct+
colcon test-result --verbose
```

## Vision cliff detection + A-to-B navigation + dashboard (added)

Depth-based terrain/cliff detection, footprint-aware A* replanning and a web
dashboard. Everything above is unchanged; this only runs with the new launch file.
Full details, thresholds and test results: `src/vigil_rough_terrain/docs/VISION_NAVIGATION.md`.

```bash
cd /home/user/Documents/robot/vigil_rough_terrain_ws
chmod +x src/vigil_rough_terrain/scripts/*.py src/vigil_rough_terrain/test/*.py
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/local_setup.bash
export ROS_DOMAIN_ID=71

# rock terrain (18 x 18 m), A = (-5.4, -3.6), B = (-3.024, -2.16) from config/vision_nav.yaml, 0.40 m/s
ros2 launch vigil_rough_terrain vision_nav.launch.py
# open http://localhost:8080

# test worlds: flat small_rocks moderate_slope small_drop cliff_front cliff_left
#              cliff_right narrow_route too_narrow no_route
ros2 launch vigil_rough_terrain vision_nav.launch.py scenario:=cliff_front

# offline test of the detection + planning logic (no Gazebo, ~4 min)
python3 src/vigil_rough_terrain/test/closed_loop_sim.py
```
