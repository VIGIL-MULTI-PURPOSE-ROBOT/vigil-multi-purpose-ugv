#!/usr/bin/env bash
# diagnose_sar.sh - find out WHICH part makes Gazebo crash on this machine.
#
#   cd ~/Documents/military_world/sar_ws && bash diagnose_sar.sh
#
# Takes about 5-10 minutes, needs no ROS, opens one Gazebo window for 40 s at the end.
# Everything is written to sar_ws/diagnosis/sar_diagnosis.log (Claude reads that file).
#
# Each test runs the Gazebo SERVER on its own for a fixed number of steps with a
# "sensor rig" (the rover's cameras on a fixed post at point A) so a crash can be
# pinned to: the world alone / depth camera / segmentation / 4K camera / thermal
# camera / all together / GPU selection.
set -u
cd "$(dirname "$0")" || exit 1
WS="$PWD"
MW="$(cd .. && pwd)"
OUT="$WS/diagnosis"
LOG="$OUT/sar_diagnosis.log"
mkdir -p "$OUT"
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
export GZ_PARTITION=vigil_sar_diag
export GZ_SIM_RESOURCE_PATH="$MW/gazebo_export${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MW/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
WORLD="$WS/generated/military_sar.sdf"

say "=== environment ==="
say "date            : $(date -Is)"
say "gz              : $(gz sim --versions 2>&1 | head -1)"
say "session         : ${XDG_SESSION_TYPE:-?}  WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-} DISPLAY=${DISPLAY:-}"
say "QT_QPA_PLATFORM : ${QT_QPA_PLATFORM:-unset}"
say "GPUs            :"; (lspci 2>/dev/null | grep -iE 'vga|3d|display' | sed 's/^/    /') | tee -a "$LOG"
command -v nvidia-smi >/dev/null && nvidia-smi --query-gpu=name,driver_version,memory.total,memory.used --format=csv,noheader 2>&1 | sed 's/^/    nvidia: /' | tee -a "$LOG"
say "GL renderer     : $(glxinfo -B 2>/dev/null | grep -E 'OpenGL renderer' | head -1 || echo 'glxinfo not installed (sudo apt install mesa-utils)')"
say "free RAM (MB)   : $(free -m | awk '/^Mem:/{print $7}')"
say "world           : $WORLD ($(stat -c %s "$WORLD" 2>/dev/null || echo MISSING) bytes)"
say ""

# ------------------------------------------------------------------ test worlds
python3 - "$WORLD" "$OUT" <<'PY'
import sys, re
from pathlib import Path
world, out = Path(sys.argv[1]), Path(sys.argv[2])
src = world.read_text()
tiny = '''<?xml version="1.0"?>
<sdf version="1.10"><world name="military_world">
  <physics name="default" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
  <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
  <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors"><render_engine>ogre2</render_engine></plugin>
  <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
  <atmosphere type="adiabatic"><temperature>293.15</temperature></atmosphere>
  <light type="directional" name="sun"><pose>0 0 50 0 0 0</pose><direction>-0.5 0.3 -0.8</direction></light>
  <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision>
    <visual name="v"><geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></visual></link></model>
  <model name="warm_person"><static>true</static><pose>0 -2 0.9 0 0 0</pose><link name="l">
    <visual name="v"><geometry><box><size>0.4 0.3 1.7</size></box></geometry>
      <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>305.15</temperature></plugin></visual></link></model>
</world></sdf>
'''
SENS = {
 'rgbd': '''<sensor name="rgbd" type="rgbd_camera"><topic>/diag/camera</topic><update_rate>10</update_rate><always_on>1</always_on>
   <camera><horizontal_fov>1.5184</horizontal_fov><image><width>320</width><height>240</height><format>R8G8B8</format></image><clip><near>0.2</near><far>25</far></clip><depth_camera><clip><near>0.2</near><far>15</far></clip></depth_camera></camera></sensor>''',
 'seg': '''<sensor name="segmentation" type="segmentation"><topic>/diag/segmentation</topic><update_rate>10</update_rate><always_on>1</always_on>
   <camera><segmentation_type>semantic</segmentation_type><horizontal_fov>1.5184</horizontal_fov><image><width>320</width><height>240</height></image><clip><near>0.2</near><far>25</far></clip></camera></sensor>''',
 'hd4k': '''<sensor name="hd_camera" type="camera"><topic>/diag/hd</topic><update_rate>5</update_rate><always_on>1</always_on>
   <camera><horizontal_fov>1.5184</horizontal_fov><image><width>3840</width><height>2160</height><format>R8G8B8</format></image><clip><near>0.2</near><far>60</far></clip></camera></sensor>''',
 'hd1080': '''<sensor name="hd_camera" type="camera"><topic>/diag/hd</topic><update_rate>10</update_rate><always_on>1</always_on>
   <camera><horizontal_fov>1.5184</horizontal_fov><image><width>1920</width><height>1080</height><format>R8G8B8</format></image><clip><near>0.2</near><far>60</far></clip></camera></sensor>''',
 'thermal': '''<sensor name="thermal_camera" type="thermal_camera"><topic>/diag/thermal</topic><update_rate>10</update_rate><always_on>1</always_on>
   <camera><horizontal_fov>1.0472</horizontal_fov><image><width>320</width><height>240</height><format>L16</format></image><clip><near>0.2</near><far>120</far></clip></camera>
   <plugin filename="gz-sim-thermal-sensor-system" name="gz::sim::systems::ThermalSensor"><min_temp>253.15</min_temp><max_temp>673.15</max_temp><resolution>0.01</resolution></plugin></sensor>''',
 'lidar': '''<sensor name="lidar" type="gpu_lidar"><topic>/diag/lidar</topic><update_rate>5</update_rate><always_on>1</always_on>
   <lidar><scan><horizontal><samples>512</samples><resolution>1</resolution><min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
   <vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.45</min_angle><max_angle>0.30</max_angle></vertical></scan><range><min>0.3</min><max>25</max></range></lidar></sensor>''',
}
def rig(names):
    s = ''.join(SENS[n] for n in names)
    return f'''<model name="diag_sensor_rig"><static>true</static><pose>0 -8 1.2 0 0.1745 1.5708</pose><link name="l">{s}</link></model>'''
def put(text, names, name):
    t = text.replace('</world>', (rig(names) if names else '') + '</world>', 1)
    (out / f'{name}.sdf').write_text(t)
cases = {
 'W0_world_no_sensors': (src, []),
 'W1_world_rgbd': (src, ['rgbd']),
 'W2_world_rgbd_lidar': (src, ['rgbd', 'lidar']),
 'W3_world_segmentation': (src, ['rgbd', 'seg']),
 'W4_world_hd1080': (src, ['rgbd', 'hd1080']),
 'W5_world_hd4k': (src, ['rgbd', 'hd4k']),
 'W6_world_thermal': (src, ['thermal']),
 'W7_world_all_sensors': (src, ['rgbd', 'lidar', 'seg', 'hd4k', 'thermal']),
 'T1_tiny_thermal': (tiny, ['thermal']),
 'T2_tiny_all_sensors': (tiny, ['rgbd', 'lidar', 'seg', 'hd4k', 'thermal']),
}
for name, (text, names) in cases.items():
    put(text, names, name)
print('test worlds written:', ', '.join(cases))
PY
say ""

run() {   # run <name> <world> [env...]
  local name="$1" w="$2"; shift 2
  say "--- $name ${*:+[$*]} ---"
  local T="$OUT/$name.out"
  ( env "$@" timeout 150 gz sim -v 3 -s -r --iterations 3000 "$w" ) > "$T" 2>&1
  local rc=$?
  local what="OK (3000 steps finished)"
  [[ $rc -eq 124 ]] && what="STILL RUNNING after 150 s (no crash, just slow)"
  [[ $rc -gt 128 ]] && what="CRASHED (signal $((rc-128)))"
  [[ $rc -ne 0 && $rc -ne 124 && $rc -lt 128 ]] && what="EXIT $rc"
  say "    result: $what"
  grep -aiE "error|segmentation|signal|stack trace|abort|exception|ogre|egl|gl_|vulkan|failed" "$T" \
     | sed 's/\x1b\[[0-9;]*m//g' | sort -u | head -15 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
  if [[ "$what" == CRASHED* ]]; then
    say "    backtrace (last 40 lines):"
    sed 's/\x1b\[[0-9;]*m//g' "$T" | tail -40 | sed 's/^/      /' | tee -a "$LOG" >/dev/null
  fi
}

say "=== server tests (default GPU selection) ==="
for n in T1_tiny_thermal T2_tiny_all_sensors W0_world_no_sensors W1_world_rgbd W2_world_rgbd_lidar \
         W3_world_segmentation W4_world_hd1080 W5_world_hd4k W6_world_thermal W7_world_all_sensors; do
  run "$n" "$OUT/$n.sdf"
done
say ""
say "=== all sensors, alternative GPU paths ==="
say "--- W7 with --headless-rendering ---"
( timeout 150 gz sim -v 3 -s -r --headless-rendering --iterations 3000 "$OUT/W7_world_all_sensors.sdf" ) > "$OUT/W7_headless.out" 2>&1
rc=$?; say "    exit code $rc (0 ok, 124 slow-no-crash, 139 crash)"
if command -v nvidia-smi >/dev/null; then
  run W7_nvidia_offload "$OUT/W7_world_all_sensors.sdf" __NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia \
      __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
fi
say ""
say "=== GUI test: generated world, 40 s, QT_QPA_PLATFORM=xcb (a window opens, just wait) ==="
( QT_QPA_PLATFORM=xcb timeout 40 gz sim -v 3 "$OUT/W0_world_no_sensors.sdf" ) > "$OUT/gui_xcb.out" 2>&1
rc=$?; say "    exit code $rc (124 = still running after 40 s = GUI WORKED; 139 = crash)"
grep -aiE "error|segmentation|signal|stack trace|egl|ogre" "$OUT/gui_xcb.out" | sed 's/\x1b\[[0-9;]*m//g' | sort -u | head -12 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
[[ $rc -ne 124 ]] && { say "    last 30 lines:"; sed 's/\x1b\[[0-9;]*m//g' "$OUT/gui_xcb.out" | tail -30 | sed 's/^/      /' | tee -a "$LOG" >/dev/null; }
say ""
say "=== done. Tell Claude: diagnosis finished ($LOG) ==="
