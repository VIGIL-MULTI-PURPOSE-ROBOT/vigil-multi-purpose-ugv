#!/usr/bin/env bash
# Gazebo Harmonic: scripted moving people are enabled by default.
#
#   --kinematic  people are placed on their route every step (DEFAULT).
#                They never fall over and never fail, but they walk through
#                walls and through the robot.
#   --contact    people are dynamic bodies pushed along their route, so they
#                collide with terrain, props, walls and the robot and stop
#                when blocked. Heavier, and it leans on your physics build.
#   --demo       two people near the origin, for a close look.
#   --static     freeze the routes entirely.
#   --check      run the automated motion checks instead of opening the GUI.
#   --software   CPU rendering, for a broken GL driver.
#   --server     server only, no window (for ROS 2 work).
#   --reexport   force the Blender export to run again.
#
# --actors is kept as an alias for the default moving configuration.
set -euo pipefail
cd "$(dirname "$0")"

SOFTWARE=0
SERVER_ONLY=0
DEMO=0
STATIC=0
CHECK=0
REEXPORT=0
CONTROLLER=2
for arg in "$@"; do
  case "$arg" in
    --actors) STATIC=0 ;;
    --static|--no-actors) STATIC=1 ;;
    --kinematic) CONTROLLER=1 ;;
    --contact) CONTROLLER=2 ;;
    --demo) DEMO=1 ;;
    --software) SOFTWARE=1 ;;
    --server) SERVER_ONLY=1 ;;
    --check) CHECK=1 ;;
    --reexport) REEXPORT=1 ;;
    --help|-h)
      sed -n '2,17p' "$0" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done
command -v gz >/dev/null || { echo 'Gazebo Harmonic (gz sim) is required.' >&2; exit 1; }
EXPORT_DIR="$PWD/gazebo_export"
BUILD_DIR="$PWD/build"
SDF="$EXPORT_DIR/military_world.sdf"
export GZ_SIM_RESOURCE_PATH="$EXPORT_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$BUILD_DIR${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
# Preserve Gazebo's default / caller-supplied partition so ROS bridges and
# commands launched in another terminal can connect to this world.

if [[ ! -f "$SDF" ]] || ! grep -q 'sar::WaypointSystem' "$SDF" || \
   [[ export_gazebo.py -nt "$SDF" || sar_motion.py -nt "$SDF" || military_world.blend -nt "$SDF" || "$REEXPORT" == 1 ]]; then
  command -v blender >/dev/null || { echo 'Blender is required to update the export.' >&2; exit 1; }
  echo '[run_gazebo] Exporting visible models and their motion routes...'
  mkdir -p "$EXPORT_DIR"
  blender --background military_world.blend --python-exit-code 1 --python export_gazebo.py -- \
    --out "$EXPORT_DIR" --no-thermal > "$EXPORT_DIR/export.log" 2>&1 || {
      tail -40 "$EXPORT_DIR/export.log"; exit 1;
    }
  tail -10 "$EXPORT_DIR/export.log"
fi

# Controller / physics changes only rewrite SDF; meshes can be reused. The
# grep is for the controller actually asked for, so switching between
# --kinematic and --contact reconfigures instead of silently reusing the
# other one's world.
if [[ sar_physics.py -nt "$SDF" ]] || \
   ! grep -q "<controller_version>$CONTROLLER</controller_version>" "$SDF"; then
  python3 sar_physics.py --controller "$CONTROLLER" "$EXPORT_DIR"/*.sdf
fi

if [[ ! -f "$BUILD_DIR/Makefile" || CMakeLists.txt -nt "$BUILD_DIR/Makefile" ]]; then
  cmake -S . -B "$BUILD_DIR" -DCMAKE_BUILD_TYPE=Release > "$EXPORT_DIR/build.log" 2>&1 || {
    cat "$EXPORT_DIR/build.log"
    echo 'Build requirements: cmake, g++, libgz-sim8-dev.' >&2
    exit 1
  }
fi
cmake --build "$BUILD_DIR" --target sar-waypoint-system -j 2

if [[ "$DEMO" == 1 ]]; then SDF="$EXPORT_DIR/dynamic_demo.sdf"; fi
if [[ "$STATIC" == 1 ]]; then
  python3 - "$SDF" <<'PY'
import sys
from pathlib import Path
import xml.etree.ElementTree as ET
source = Path(sys.argv[1])
tree = ET.parse(source)
for model in tree.findall('.//model'):
    for plugin in model.findall("plugin[@name='sar::WaypointSystem']"):
        model.find('static').text = 'true'
        model.remove(plugin)
tree.write(source.with_stem(source.stem + '_static'), encoding='utf-8', xml_declaration=True)
PY
  SDF="${SDF%.sdf}_static.sdf"
fi

python3 - "$SDF" "$CONTROLLER" <<'PY'
import sys
import xml.etree.ElementTree as ET
world = ET.parse(sys.argv[1]).getroot().find('world')
mode = 'contact-aware' if sys.argv[2] == '2' else 'kinematic'
movers = [m for m in world.findall('model')
          if m.find("plugin[@name='sar::WaypointSystem']") is not None]
print(f'[run_gazebo] {len(movers)} {mode} moving people in {sys.argv[1]}')
for m in movers:
    pose = (m.findtext('pose') or '0 0 0').split()[:3]
    print(f"  {m.get('name')}: initial XYZ {pose}")
print("[run_gazebo] In the GUI: find one of those names in the Entity Tree,")
print("[run_gazebo] right-click it and choose 'Move to' to fly the camera there.")
PY

if [[ "$CHECK" == 1 ]]; then
  [[ "$STATIC" == 0 ]] || { echo '--check requires moving people; omit --static.' >&2; exit 1; }
  python3 -m unittest discover -s tests
  python3 tests/check_export.py "$EXPORT_DIR"
  gz sdf -k "$SDF"
  cmake --build "$BUILD_DIR" --target check_motion -j 2
  EXPECTED=20
  [[ "$DEMO" == 0 ]] || EXPECTED=2
  "$BUILD_DIR/check_motion" "$SDF" "$EXPECTED"
  # Contact behaviour is only meaningful for the contact controller, and its
  # fixtures are built from the demo export.
  if [[ "$CONTROLLER" == 2 && -f "$EXPORT_DIR/dynamic_demo.sdf" ]]; then
    mkdir -p validation
    python3 tests/make_contact_worlds.py
    cmake --build "$BUILD_DIR" --target check_contacts -j 2
    for mode in free wall people; do
      "$BUILD_DIR/check_contacts" "validation/contact_${mode}.sdf" "$mode"
    done
  fi
  exit 0
fi

export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
if [[ "$SOFTWARE" == 1 ]]; then
  unset __NV_PRIME_RENDER_OFFLOAD __GLX_VENDOR_LIBRARY_NAME
  export LIBGL_ALWAYS_SOFTWARE=1
  export GALLIUM_DRIVER=llvmpipe
fi
if [[ "$SERVER_ONLY" == 1 ]]; then exec gz sim -v 3 -s -r "$SDF"; fi

gz sim -v 3 -s -r "$SDF" &
SERVER_PID=$!
cleanup() { kill "$SERVER_PID" 2>/dev/null || true; wait "$SERVER_PID" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
# The GUI connects as the server loads; there is no fixed startup sleep.
echo '[run_gazebo] Motion starts when simulation time advances. Use --demo for a close view.'
gz sim -v 2 -g
