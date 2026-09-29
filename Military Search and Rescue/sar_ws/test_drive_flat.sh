#!/usr/bin/env bash
# test_drive_flat.sh - can the rover drive AT ALL?
#
#   cd ~/Documents/military_world/sar_ws && bash test_drive_flat.sh
#
# Spawns the same rover, with the same controllers, on an empty flat plane: no military_world, no
# cameras. That runs near real time, so 10 s of driving is 10 s of waiting, and it separates the
# two possible worlds:
#
#   moves on the plane  -> rover, controllers and drive.py are fine; something in military_world
#                          holds it (ground contact, an object, the spawn spot)
#   does not move       -> it is the robot model / controller chain, and the joint states printed
#                          at the end say which joint is not doing its job
#
# Everything lands in sar_ws/diagnosis/test_drive_flat.log (send that file back). ~2 minutes.
set -u
cd "$(dirname "$0")" || exit 1
mkdir -p diagnosis generated
OUT="$PWD/diagnosis"; LOG="$OUT/test_drive_flat.log"
set +u                      # colcon's setup files read unset variables
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for d in /opt/ros/*/setup.bash; do [[ -f "$d" ]] && source "$d" && break; done
fi
[[ -f install/local_setup.bash ]] && source install/local_setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
export GZ_PARTITION=vigil_sar
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }

cat > "$OUT/flat_world.sdf" <<'SDF'
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="military_world">
    <physics name="default" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 50 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><direction>-0.5 0.3 -0.8</direction></light>
    <model name="flat_ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
        <surface><friction><ode><mu>0.85</mu><mu2>0.85</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry></visual>
    </link></model>
  </world>
</sdf>
SDF
say "=== test_drive_flat $(date -Is) ==="
say "world: $OUT/flat_world.sdf (flat plane, mu 0.85, 1 ms step, no cameras)"
say ""

bash stop_sar.sh >/dev/null 2>&1
( ros2 launch vigil_sar sim.launch.py gui:=false profile:=none cleanup:=true \
    world_file:="$OUT/flat_world.sdf" spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.35 \
    rgbd:=false lidar:=false segmentation:=false hd_camera:=false thermal:=false \
  ) > "$OUT/flat_launch.log" 2>&1 &
LAUNCH=$!
say "waiting for the rover and its controllers (up to 90 s) ..."
for i in $(seq 90); do
  grep -q "wheel_controller.*activate successful\|Successfully switched controllers" "$OUT/flat_launch.log" 2>/dev/null && break
  sleep 1
done
sleep 5
pose() { timeout 15 ros2 topic echo --once /sim/ground_truth 2>/dev/null \
         | grep -A4 "position:" | head -4 | tr -d ' \n'; }

say ""
say "--- before ---"
say "    pose: $(pose)"
say ""
say "--- driving forward at 0.5 m/s for 20 s ---"
timeout 20 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" >/dev/null 2>&1 &
sleep 6
say "    cmd_vel      : $(timeout 8 ros2 topic echo --once /cmd_vel 2>/dev/null | tr -d ' \n' | head -c 160)"
say "    wheel command: $(timeout 8 ros2 topic echo --once /wheel_controller/commands 2>/dev/null | tr -d ' \n' | head -c 200)"
say "    wheel speeds : $(timeout 8 ros2 topic echo --once /encoders 2>/dev/null | grep -A12 velocity: | head -12 | tr -d ' \n' | head -c 200)"
wait %2 2>/dev/null
say ""
say "--- after ---"
say "    pose: $(pose)"
say ""
say "--- every joint (suspension at its limit = the springs are not holding the rover) ---"
timeout 20 ros2 topic echo --once /joint_states 2>&1 | head -40 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
say ""
say "--- wheel contacts on the plane ---"
for wheel in L1 L2 L3 L4 R1 R2 R3 R4; do
  if timeout 6 ros2 topic echo --once "/suspension/contacts/$wheel" >/dev/null 2>&1; then
    say "    $wheel: touching"
  else
    say "    $wheel: NO contact"
  fi
done

kill -INT $LAUNCH 2>/dev/null; sleep 8; bash stop_sar.sh >/dev/null 2>&1
say ""
say "=== done. Send back $LOG (and $OUT/flat_launch.log if something looks odd) ==="
