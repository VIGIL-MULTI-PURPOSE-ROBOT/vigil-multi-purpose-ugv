#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$(readlink -f "$0")")"
for required in \
  src/vigil_new_mining/models/mine/meshes/mine_collision.stl \
  src/vigil_new_mining/models/mine/meshes/mine_u1_v1.obj \
  src/vigil_new_mining/models/mine/materials/textures/tex_u1_v1_diffuse.png; do
  if [[ ! -f "$required" ]]; then
    echo "Missing licensed mine asset: $required" >&2
    echo "See src/vigil_new_mining/models/mine/README.md before building." >&2
    exit 2
  fi
done
export ROS_DOMAIN_ID=74 GZ_PARTITION=vigil_new_mining ROS_LOG_DIR="$PWD/log/runtime"
source /opt/ros/jazzy/setup.bash
source install/setup.bash
exec ros2 launch vigil_new_mining mining.launch.py "$@"
