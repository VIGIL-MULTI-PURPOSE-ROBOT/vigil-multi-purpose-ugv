#!/usr/bin/env bash
# measure_physics.sh - measure_speed.sh showed the cameras are NOT the problem (no sensors at all:
# 5.3 %, all sensors: 6.5 %), so the physics of military_world is. This finds which part: it builds
# variants of the SAR world, starts each without GUI and without sensors, and reads Gazebo's
# real-time factor.            cd ~/Documents/military_world/sar_ws && bash measure_physics.sh   (~8 min)
# Result: diagnosis/measure_physics.log (send it back). Nothing in the project is changed.
set -u
cd "$(dirname "$0")" || exit 1
set +u; source /opt/ros/jazzy/setup.bash; source install/local_setup.bash; set -u
export ROS_DOMAIN_ID=72 GZ_PARTITION=vigil_sar
mkdir -p diagnosis/physics; OUT="$PWD/diagnosis"; V="$OUT/physics"; LOG="$OUT/measure_physics.log"; : > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }

say "=== measure_physics $(date -Is)  (no GUI, no sensors, rover standing still)"
python3 - "$V" <<'PY' 2>&1 | tee -a "$LOG"
import copy, sys, re, xml.etree.ElementTree as ET
from pathlib import Path
sys.path.insert(0, 'src/vigil_sar/scripts')
from sar_paths import find_military_world, load_config
import build_sar_world as b
out = Path(sys.argv[1]); cfg0 = load_config('src/vigil_sar/config/sar_mission.yaml')
import yaml
eng = yaml.safe_load(open('src/vigil_sar/config/physics.yaml'))['/**']['ros__parameters']['physics']['engine']
cfg0['world'].update(physics_step=eng['max_step_size'], gravity=eng['gravity'], collision_detector=eng['collision_detector'],
                     physics_solver=eng['solver'], ground_friction=eng['ground_friction'])
mw = find_military_world(cfg0['world'].get('military_world_dir', ''))
def build(name, **over):
    c = copy.deepcopy(cfg0); c['world'].update(over); b.build(c, mw, out / f'{name}.sdf'); return out / f'{name}.sdf'
build('A_current')
build('B_people_frozen', moving_people=False)
build('F_step_2ms', physics_step=0.002)
# C: every non-ground model keeps its look but loses its collision shapes
# D: only the ground (terrain tiles, roads, apron, bridge) - all other models removed
for name, keep_models in (('C_no_object_collisions', True), ('D_ground_only', False)):
    tree = ET.parse(out / 'A_current.sdf'); w = tree.getroot().find('world')
    for m in list(w.findall('model')):
        ground = re.match(r'SAR_(Terrain|Road|Street|Track|Base_Apron|Bridge_001_Deck)', m.get('name', ''))
        if ground:
            continue
        if keep_models:
            for link in m.iter('link'):
                for col in list(link.findall('collision')):
                    link.remove(col)
        else:
            w.remove(m)
    tree.write(out / f'{name}.sdf')
# E: one terrain tile's worth of mesh work: ground models replaced by a flat plane (everything else kept)
tree = ET.parse(out / 'A_current.sdf'); w = tree.getroot().find('world')
for m in list(w.findall('model')):
    if re.match(r'SAR_(Terrain|Road|Street|Track|Base_Apron|Bridge_001_Deck)', m.get('name', '')):
        for link in m.iter('link'):
            for col in list(link.findall('collision')):
                link.remove(col)
plane = ET.fromstring('<model name="flat_ground"><static>true</static><link name="l"><collision name="c"><geometry>'
                      '<plane><normal>0 0 1</normal><size>400 400</size></plane></geometry></collision></link></model>')
w.append(plane); tree.write(out / 'E_flat_plane_ground.sdf')
print('variants written to', out)
PY
stop_all() { bash stop_sar.sh >/dev/null 2>&1; sleep 3; }
rtf() { timeout 90 gz topic -e -t /stats -n 40 2>/dev/null | awk '/real_time_factor/ {s+=$2; n++} END {if (n) printf "%5.1f %%", 100*s/n; else print "no data"}'; }
for f in "$V"/A_current.sdf "$V"/B_people_frozen.sdf "$V"/C_no_object_collisions.sdf "$V"/D_ground_only.sdf \
         "$V"/E_flat_plane_ground.sdf "$V"/F_step_2ms.sdf; do
  stop_all
  ( ros2 launch vigil_sar sim.launch.py gui:=false profile:=none cleanup:=true world_file:="$f" \
      rgbd:=false lidar:=false segmentation:=false hd_camera:=false thermal:=false ) > "$OUT/measure_launch.log" 2>&1 &
  for i in $(seq 240); do grep -q "wheel_controller.*activate successful" "$OUT/measure_launch.log" 2>/dev/null && break; sleep 1; done
  sleep 15
  say "$(printf '%-34s %s' "$(basename "$f" .sdf)" "$(rtf)")"
done
stop_all
say "=== done: $LOG"
