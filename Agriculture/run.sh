#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
mkdir -p log run
exec 9>run/simulation.lock
if ! flock -w 2 9; then
  echo 'This workspace already has a simulation running. Stop it before starting another.' >&2
  exit 1
fi
echo "$$" >run/launch.pid
colcon build --symlink-install >log/build_latest.txt 2>&1 || { cat log/build_latest.txt; exit 1; }
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export GZ_PARTITION="${GZ_PARTITION:-agri_ugv}"
export ROS_LOG_DIR="$PWD/log/runtime"
echo "Starting agricultural UGV: ROS_DOMAIN_ID=$ROS_DOMAIN_ID GZ_PARTITION=$GZ_PARTITION"
exec ros2 launch agri_ugv field.launch.py "$@"
