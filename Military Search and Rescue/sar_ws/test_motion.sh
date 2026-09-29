#!/usr/bin/env bash
# test_motion.sh - Gazebo acceptance tests, judged ONLY on what the physics engine reports.
#
#   cd ~/Documents/military_world/sar_ws && bash test_motion.sh
#
# Phase 1 (flat plane, no navigator - the test owns /cmd_vel, ~4 min):
#   GRAVITY at rest (IMU 9.81, level, six loaded wheels touching)
#   TEST 1 forward  TEST 2 turn left  TEST 3 turn right  TEST 4 stop  + reverse, arcs,
#   wheel rotation directions, and that drive.py really loaded physics.yaml
#   WHEELIE: full-speed step from rest -> smooth to exactly 3.0 m/s with the front wheels down,
#   speed cap, BRAKING from 3 m/s
# Scenarios (--scenarios): flat_road, moderate_slope, steep_hill, obstacle, cliff_front - each its
#   own small world with the navigator driving A->B (reach B, no wheelie, <= 3.0 m/s, <= 1 m/s2,
#   no standing stop, clearance speed profile, never into the wall / pit)
#   + people: human_block (the test drives INTO a standing person: the contact must stop the rover),
#   human_standing, human_crossing (navigator: never touch the person, keep distance, reach B)
#   bash test_motion.sh --scenario human_block --scenario human_standing --scenario human_crossing
# Phase 2 (military_world, full mission, headless - slow: the big world runs below real time):
#   TEST 7 SAR at A  TEST 5 reach B  TEST 8 SAR while driving  TEST 9 SAR near B
#   TEST 6 no overshoot  TEST 10 SAR after B  TEST 11 SAR again after STOP
#
#   bash test_motion.sh --motion-only          only phase 1
#   bash test_motion.sh --mission-only         only phase 2
#   bash test_motion.sh --goal -3.0 -2.0       B for phase 2 (default -3 -2, 6.7 m from A)
#   bash test_motion.sh --humans 20            phase 2 also runs SAR for up to 20 simulated
#                                              minutes and waits for a thermal human (H1)
#   bash test_motion.sh --scenarios            phases 1 + 2 + all five scenarios
#   bash test_motion.sh --scenarios-only       only the five scenarios
#   bash test_motion.sh --scenario steep_hill  only that scenario (repeatable)
#   bash test_motion.sh --urban                phase 2 with B in the building area (70, 40), 85 m from
#                                              A, and a 15-minute SAR search there (building scans)
#   bash test_motion.sh --validate             everything: scale check, phase 1, all scenarios, --urban
# Every run starts with the scale check (scripts/scale_check.py: world vs rover, 1 unit = 1 m).
#
# Everything lands in sar_ws/diagnosis/test_motion.log  (send that file back)
set -u
cd "$(dirname "$0")" || exit 1
mkdir -p diagnosis generated
OUT="$PWD/diagnosis"; LOG="$OUT/test_motion.log"
MOTION=1; MISSION=1; SCEN=0; GOAL=(-3.0 -2.0); HUMANS=0; ONLY=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --motion-only)  MISSION=0 ;;
    --mission-only) MOTION=0 ;;
    --goal)         GOAL=("$2" "$3"); shift 2 ;;
    --humans)       HUMANS="$2"; shift ;;
    --scenarios)    SCEN=1 ;;
    --scenarios-only) SCEN=1; MOTION=0; MISSION=0 ;;
    --scenario)     SCEN=1; MOTION=0; MISSION=0; ONLY+=("$2"); shift ;;
    --urban)        GOAL=(70.0 40.0); [[ $HUMANS == 0 ]] && HUMANS=15 ;;
    --validate)     MOTION=1; SCEN=1; MISSION=1; GOAL=(70.0 40.0); [[ $HUMANS == 0 ]] && HUMANS=15 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
  shift
done
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
PREFIX="$(ros2 pkg prefix vigil_sar 2>/dev/null)"
PROBE="$PREFIX/lib/vigil_sar/motion_test.py"
[[ -x "$PROBE" ]] || PROBE="python3 $PWD/src/vigil_sar/scripts/motion_test.py"
WORLDS="$PREFIX/lib/vigil_sar/scenario_worlds.py"
[[ -x "$WORLDS" ]] || WORLDS="python3 $PWD/src/vigil_sar/scripts/scenario_worlds.py"
CONFIG="$PREFIX/share/vigil_sar/config/sar_mission.yaml"
[[ -f "$CONFIG" ]] || CONFIG="$PWD/src/vigil_sar/config/sar_mission.yaml"

wait_ready() {      # wait_ready <launch log> <seconds>
  for i in $(seq "$2"); do
    grep -q "wheel_controller.*activate successful" "$1" 2>/dev/null && return 0
    grep -q "Gazebo server CRASHED" "$1" 2>/dev/null && return 1
    sleep 1
  done
  return 1
}
stop_all() { bash stop_sar.sh >/dev/null 2>&1; sleep 2; }

say "=== test_motion $(date -Is) ==="
RESULT=0

SCALE="$PREFIX/lib/vigil_sar/scale_check.py"
[[ -x "$SCALE" ]] || SCALE="python3 $PWD/src/vigil_sar/scripts/scale_check.py"
say ""
say "##### SCALE CHECK (no simulation)"
$SCALE 2>&1 | tee -a "$LOG"
[[ ${PIPESTATUS[0]} -eq 0 ]] || RESULT=1

if [[ $MOTION -eq 1 ]]; then
  cat > "$OUT/flat_world.sdf" <<'SDF'
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="military_world">
    <physics name="default" type="dart"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <gravity>0 0 -9.81</gravity>
    <light type="directional" name="sun"><cast_shadows>false</cast_shadows><pose>0 0 50 0 0 0</pose>
      <diffuse>1 1 1 1</diffuse><direction>-0.5 0.3 -0.8</direction></light>
    <model name="flat_ground"><static>true</static><link name="link">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>
        <surface><friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction></surface></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry></visual>
    </link></model>
  </world>
</sdf>
SDF
  say ""
  say "##### PHASE 1: flat plane, no navigator (the test drives /cmd_vel itself)"
  stop_all
  ( ros2 launch vigil_sar sim.launch.py gui:=false profile:=none cleanup:=true \
      world_file:="$OUT/flat_world.sdf" spawn_x:=0.0 spawn_y:=0.0 spawn_z:=0.35 \
      rgbd:=false lidar:=false segmentation:=false hd_camera:=false thermal:=false \
    ) > "$OUT/motion_phase1_launch.log" 2>&1 &
  if wait_ready "$OUT/motion_phase1_launch.log" 120; then
    grep -a "\[vigil_sar\] rover:" "$OUT/motion_phase1_launch.log" | tail -1 | sed 's/^/    /' | tee -a "$LOG"
    $PROBE --phase motion 2>&1 | tee -a "$LOG"
    [[ ${PIPESTATUS[0]} -eq 0 ]] || RESULT=1
  else
    say "FAIL  simulation did not come up - last lines of the launch log:"
    tail -25 "$OUT/motion_phase1_launch.log" | tee -a "$LOG"; RESULT=1
  fi
  stop_all
fi

if [[ $SCEN -eq 1 ]]; then
  SDIR="$OUT/scenarios"; $WORLDS "$SDIR" | tee -a "$LOG"
  NAMES=("${ONLY[@]}"); [[ ${#NAMES[@]} -eq 0 ]] && read -r -a NAMES <<< "$($WORLDS --list)"
  for NAME in "${NAMES[@]}"; do
    if [[ ! -f "$SDIR/$NAME.sdf" ]]; then say "FAIL  unknown scenario $NAME"; RESULT=1; continue; fi
    read -r SX SY SZ < <(python3 -c "import json;s=json.load(open('$SDIR/scenarios.json'))['$NAME']['spawn'];print(*s)")
    say ""
    say "##### SCENARIO $NAME (navigator drives; depth + segmentation cameras on)"
    stop_all
    ( ros2 launch vigil_sar sim.launch.py gui:=false profile:=none cleanup:=true \
        world_file:="$SDIR/$NAME.sdf" spawn_x:="$SX" spawn_y:="$SY" spawn_z:="$SZ" \
        rgbd:=true segmentation:=true lidar:=false hd_camera:=false thermal:=false \
      ) > "$OUT/scenario_${NAME}_launch.log" 2>&1 &
    DRIVE=$(python3 -c "import json;print(json.load(open('$SDIR/scenarios.json'))['$NAME'].get('drive','navigator'))")
    if wait_ready "$OUT/scenario_${NAME}_launch.log" 180; then
      # human_block: the probe itself drives into the person (no navigator, no avoidance)
      [[ "$DRIVE" == "probe" ]] || ( ros2 launch vigil_sar navigation.launch.py ) > "$OUT/scenario_${NAME}_nav.log" 2>&1 &
      $PROBE --phase scenario --scenario "$NAME" --meta "$SDIR/scenarios.json" --config "$CONFIG" 2>&1 | tee -a "$LOG"
      [[ ${PIPESTATUS[0]} -eq 0 ]] || RESULT=1
      grep -a "cliff_navigator.*state " "$OUT/scenario_${NAME}_nav.log" | tail -12 | sed 's/^/    nav: /' | tee -a "$LOG"
    else
      say "FAIL  simulation did not come up - last lines of the launch log:"
      tail -25 "$OUT/scenario_${NAME}_launch.log" | tee -a "$LOG"; RESULT=1
    fi
    stop_all
  done
fi

if [[ $MISSION -eq 1 ]]; then
  say ""
  say "##### PHASE 2: military_world, full mission, headless (B = ${GOAL[*]})"
  say "      slow on purpose: the 300 m world runs far below real time"
  stop_all
  ( ros2 launch vigil_sar sar_mission.launch.py gui:=false dashboard:=false hd_camera:=false \
      lidar:=false fast:=true ) > "$OUT/motion_phase2_launch.log" 2>&1 &
  if wait_ready "$OUT/motion_phase2_launch.log" 300; then
    $PROBE --phase mission --goal "${GOAL[0]}" "${GOAL[1]}" --humans "$HUMANS" 2>&1 | tee -a "$LOG"
    [[ ${PIPESTATUS[0]} -eq 0 ]] || RESULT=1
  else
    say "FAIL  simulation did not come up - last lines of the launch log:"
    tail -25 "$OUT/motion_phase2_launch.log" | tee -a "$LOG"; RESULT=1
  fi
  stop_all
fi

say ""
say "=== done: $([[ $RESULT -eq 0 ]] && echo 'ALL PASSED' || echo 'FAILURES above') - full log: $LOG ==="
exit $RESULT
