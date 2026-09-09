#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-91}"
export GZ_PARTITION="${GZ_PARTITION:-agri_ugv}"
export ROS_LOG_DIR="$PWD/log/demo"
if [ "$#" -gt 0 ]; then exec python3 tools/command.py "$1"; fi
python3 tools/presentation_demo.py
python3 tools/check_lighting.py
python3 tools/check_suspension.py
