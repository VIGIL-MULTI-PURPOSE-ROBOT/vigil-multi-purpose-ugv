#!/usr/bin/env bash
# diagnose_gazebo.sh -- one run, one log, tells us exactly what Gazebo objects to.
#
# Runs each world SERVER-ONLY and HEADLESS for a fixed number of steps, so
# nothing depends on the GUI, the GPU or your window manager, and each test
# ends by itself instead of needing Ctrl-C. Then it tries the GUI once.
#
#   bash diagnose_gazebo.sh
#
# Paste the LAST 60 LINES of gazebo_diagnosis.log back.

cd "$(dirname "$0")" || exit 1
EXPORT_DIR="$PWD/gazebo_export"
LOG="$PWD/gazebo_diagnosis.log"
export GZ_SIM_RESOURCE_PATH="$EXPORT_DIR${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"

: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }

say "=== environment ==="
say "date              : $(date -Is)"
say "gz version        : $(gz sim --versions 2>&1 | head -1)"
say "gz sdf available  : $(command -v gz >/dev/null && echo yes || echo no)"
say "GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"
say "export dir exists : $([ -d "$EXPORT_DIR" ] && echo yes || echo NO)"
say "mesh count        : $(ls "$EXPORT_DIR/meshes" 2>/dev/null | wc -l)"
say "free RAM (MB)     : $(free -m 2>/dev/null | awk '/^Mem:/{print $7}')"
say "renderer          : ${XDG_SESSION_TYPE:-unknown} session"
say ""

# ---- 1. static SDF validation (no simulation, no GPU) -------------------
say "=== sdf syntax check ==="
for f in stage1_one_mesh stage2_terrain stage3_structures stage4_scatter \
         stage5_movers dynamic_demo military_world; do
  W="$EXPORT_DIR/$f.sdf"
  [ -f "$W" ] || { say "  $f.sdf  MISSING"; continue; }
  OUT=$(gz sdf -k "$W" 2>&1)
  RC=$?
  if [ "$RC" -eq 0 ]; then
    say "  $f.sdf  OK"
  else
    say "  $f.sdf  REJECTED:"
    echo "$OUT" | head -15 | sed 's/^/      /' | tee -a "$LOG" >/dev/null
    echo "$OUT" | head -15 | sed 's/^/      /'
  fi
done
say ""

# ---- 2. headless server run, finite steps -------------------------------
say "=== headless server runs (200 steps each, 120 s cap) ==="
for f in stage1_one_mesh stage2_terrain stage3_structures stage4_scatter \
         stage5_movers dynamic_demo military_world; do
  W="$EXPORT_DIR/$f.sdf"
  [ -f "$W" ] || continue
  say "--- $f ---"
  TMP=$(mktemp)
  timeout 120 gz sim -v 4 -s -r --iterations 200 "$W" > "$TMP" 2>&1
  RC=$?
  say "    exit code: $RC  $( [ $RC -eq 124 ] && echo '(TIMED OUT)' )"
  # the interesting lines only
  grep -iE "error|severe|unable|cannot|fail|missing|abort|segmentation|exception|terminate" \
       "$TMP" | sort -u | head -12 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
  grep -iE "error|severe|unable|cannot|fail|missing|abort|segmentation|exception|terminate" \
       "$TMP" | sort -u | head -12 | sed 's/^/    /'
  say "    last 5 lines:"
  tail -5 "$TMP" | sed 's/^/      /' | tee -a "$LOG" >/dev/null
  tail -5 "$TMP" | sed 's/^/      /'
  rm -f "$TMP"
done
say ""

# ---- 3. one GUI attempt, smallest world ---------------------------------
say "=== GUI attempt: stage1_one_mesh (15 s, then killed) ==="
TMP=$(mktemp)
timeout 15 gz sim -v 4 "$EXPORT_DIR/stage1_one_mesh.sdf" > "$TMP" 2>&1
say "    exit code: $?  (124 = still running after 15 s, which means IT WORKED)"
grep -iE "error|severe|unable|cannot|fail|ogre|render|gl " "$TMP" | sort -u | head -12 \
     | sed 's/^/    /' | tee -a "$LOG" >/dev/null
grep -iE "error|severe|unable|cannot|fail|ogre|render|gl " "$TMP" | sort -u | head -12 | sed 's/^/    /'
rm -f "$TMP"

say ""
say "=== done: full log at $LOG ==="
echo
echo "Paste the last 60 lines back:"
echo "    tail -60 $LOG"
