#!/usr/bin/env bash
# check_drive.sh - why is the rover not moving? Run it WHILE the mission is running
# (dashboard says NAVIGATING) in a second terminal:
#
#   cd ~/Documents/military_world/sar_ws && bash check_drive.sh
#
# Takes about a minute and follows the command from the navigator to the wheels:
#   /navigation/status      what the navigator thinks it is doing
#   /cmd_vel                what the navigator commands        <- 0 here = the navigator
#   /wheel_controller/commands   what drive.py asks the wheels to do
#   /encoders               what the wheels actually do        <- spinning but not moving = physics
#   /sim/ground_truth       whether the robot moves
# Everything is written to sar_ws/diagnosis/check_drive.log (send that file back).
set -u
cd "$(dirname "$0")" || exit 1
mkdir -p diagnosis
LOG="$PWD/diagnosis/check_drive.log"
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

say "=== check_drive $(date -Is) (ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
say ""
say "--- nodes running ---"
ros2 node list 2>&1 | sed 's/^/    /' | tee -a "$LOG" >/dev/null

one() {   # one <topic> [seconds]
  say ""
  say "--- $1 ---"
  timeout "${2:-12}" ros2 topic echo --once "$1" 2>&1 | head -30 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
  [[ ${PIPESTATUS[0]} -eq 124 ]] && say "    (nothing published within ${2:-12} s)"
}
rate() {  # rate <topic>
  local out
  out=$(timeout 12 ros2 topic hz "$1" 2>&1 | head -3 | tr '\n' ' ')
  say "    rate $1: ${out:-none}"
}

one /navigation/status
one /cmd_vel
one /wheel_controller/commands
one /encoders 20
one /sim/ground_truth
say ""
say "--- what the wheels are touching (contact sensors) ---"
for wheel in L1 L4 R1 R4; do
  say "  wheel $wheel:"
  timeout 15 ros2 topic echo --once "/suspension/contacts/$wheel" 2>&1 \
    | grep -aE "collision1|collision2|name:|depth|wrench|force|x:|y:|z:" | head -14 \
    | sed 's/^/      /' | tee -a "$LOG" >/dev/null
  [[ ${PIPESTATUS[0]} -eq 124 ]] && say "      (no contact message in 15 s - this wheel may be off the ground)"
done
say ""
say "--- rates over ~10 s (wall clock; the sim runs slower than real time) ---"
rate /cmd_vel
rate /wheel_controller/commands
rate /clock

say ""
say "--- controllers ---"
timeout 20 ros2 control list_controllers 2>&1 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
say ""
say "--- moved? two ground-truth samples 15 s apart ---"
p1=$(timeout 12 ros2 topic echo --once /sim/ground_truth 2>/dev/null | grep -A3 '^  pose:' | head -8 | tr '\n' ' ')
sleep 15
p2=$(timeout 12 ros2 topic echo --once /sim/ground_truth 2>/dev/null | grep -A3 '^  pose:' | head -8 | tr '\n' ' ')
say "    t0: $p1"
say "    t1: $p2"
say ""
say "=== done. Send back $LOG ==="
