#!/usr/bin/env bash
# measure_speed.sh - what makes military_world slow? Measures Gazebo's real-time factor (RTF,
# 100 % = as fast as real life) of the SAR world with one thing switched off at a time.
#   cd ~/Documents/military_world/sar_ws && bash measure_speed.sh        (~8 min, no window)
# Result: diagnosis/measure_speed.log  (send that file back)
set -u
cd "$(dirname "$0")" || exit 1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export ROS_DOMAIN_ID=72 GZ_PARTITION=vigil_sar
mkdir -p diagnosis; OUT="$PWD/diagnosis"; LOG="$OUT/measure_speed.log"; : > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
stop_all() { bash stop_sar.sh >/dev/null 2>&1; sleep 3; }
rtf() {  # average real_time_factor of 40 /stats messages
  timeout 90 gz topic -e -t /stats -n 40 2>/dev/null | awk '/real_time_factor/ {s+=$2; n++} END {if (n) printf "%5.1f %%", 100*s/n; else print "no data"}'
}
run() {  # run <label> <launch args...>
  local label="$1"; shift
  stop_all
  ( ros2 launch vigil_sar sim.launch.py gui:=false profile:=none cleanup:=true "$@" ) > "$OUT/measure_launch.log" 2>&1 &
  for i in $(seq 240); do grep -q "wheel_controller.*activate successful" "$OUT/measure_launch.log" 2>/dev/null && break; sleep 1; done
  sleep 15                                   # let loading settle
  say "$(printf '%-46s %s' "$label" "$(rtf)")"
}
say "=== measure_speed $(date -Is)  (rover standing still, no GUI)"
run "all sensors (current default)"
run "  + lidar ON (old default)"                 lidar:=true
run "  - 4K camera"                             hd_camera:=false
run "  - thermal camera"                        thermal:=false
run "  - segmentation camera"                   segmentation:=false
run "  - all cameras (depth only)"              hd_camera:=false thermal:=false segmentation:=false
run "  no sensors at all (physics only)"        hd_camera:=false thermal:=false segmentation:=false rgbd:=false
sed 's|<cast_shadows>true</cast_shadows>|<cast_shadows>false</cast_shadows>|' generated/military_sar.sdf > "$OUT/measure_noshadow.sdf"
run "  all sensors, sun shadows OFF"            world_file:="$OUT/measure_noshadow.sdf"
stop_all
say "=== done: $LOG"
