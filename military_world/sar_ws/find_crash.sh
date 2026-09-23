#!/usr/bin/env bash
# find_crash.sh - ONE command that finds out why Gazebo crashes on this PC and leaves behind a
# render profile that works, which every vigil_sar launch then uses by itself.
#
#   cd ~/Documents/military_world/sar_ws && bash find_crash.sh
#
# Takes about 10-15 minutes, opens no window, needs nothing except this workspace.
# Optional, for the exact crash location:  sudo apt install gdb
#
#   bash find_crash.sh --seconds 40      shorter trials
#   bash find_crash.sh --skip-gdb        no backtrace run
#   bash find_crash.sh --skip-bisect     no world-content bisect
#
# Results:  sar_ws/diagnosis/find_crash.log   <- send this one back
#           sar_ws/diagnosis/find_crash.json
#           sar_ws/generated/render_profile.yaml
set -u
cd "$(dirname "$0")" || exit 1
WS="$PWD"
mkdir -p diagnosis generated

set +u                      # colcon's setup files read unset variables
if [[ -z "${ROS_DISTRO:-}" ]]; then
  for d in /opt/ros/*/setup.bash; do [[ -f "$d" ]] && source "$d" && break; done
fi
[[ -z "${ROS_DISTRO:-}" ]] && { echo "ROS 2 not found in /opt/ros - source it first."; exit 1; }

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-72}"
chmod +x src/vigil_sar/scripts/*.py stop_sar.sh 2>/dev/null

echo "[find_crash] building vigil_sar (log: diagnosis/build.log)"
colcon build --packages-select vigil_sar --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 > diagnosis/build.log 2>&1 || {
  echo "[find_crash] colcon build FAILED - last lines:"; tail -25 diagnosis/build.log; exit 1; }
source install/local_setup.bash
set -u

exec python3 "$WS/src/vigil_sar/scripts/find_crash.py" --ws "$WS" "$@"
