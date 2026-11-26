#!/usr/bin/env bash
# VIGIL Rock Terrain regression run (SIH26126): offline tests + build + Gazebo scenarios x trials.
#
# Read-only with respect to git: never commits, pushes, stashes, resets, cleans or switches branches,
# and edits no source file. Builds only into "Rock Terrain/build|install|log" (git-ignored). Stops
# only the processes it started (own process group + own unique GZ_PARTITION).
#
# Usage (from anywhere; needs a graphical session for the Gazebo window):
#   bash "Rock Terrain/tools/regression/vigil_regression.sh" [options]
#     --scenarios "a b c"     default: "cliff_front flat hill_choice narrow_route too_narrow no_route"
#     --trials N              trials per scenario (default 3)
#     --no-gui                gui:=false (sensors still render)
#     --light-camera          camera_width:=960 camera_height:=720 camera_rate:=5
#     --no-offline            skip the offline pytest / closed-loop suites
#     --baseline DIR          baseline evidence dir (default: newest ~/vigil-baseline-validation/2026*)
#     --branch NAME           stop unless the checkout is on branch NAME (default: no branch
#                             requirement; the current branch is recorded in the evidence)
#     --truth-isolation       VIGIL 2.0: simulator truth bridged only as /eval/* and no segmentation
#                             oracle (launch arg truth_isolation:=true); the monitor reads /eval/ground_truth
#     --eval                  VIGIL 2.0: per trial, run the isolated evaluator + GPS-denied guard
#                             (tools/vigil_eval/evaluator.py) and record a ROS bag (see --no-bag)
#     --no-bag                with --eval: do not record the ROS bag
# Evidence: /tmp/vigil-regression/<stamp>/  copied to ~/vigil-baseline-validation/regression-<stamp>/
set -uo pipefail

SELF_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RT="$(cd "$SELF_DIR/../.." && pwd)"                  # "Rock Terrain"
REPO="$(cd "$RT/.." && pwd)"
PKG="$RT/src/vigil_rough_terrain"
EVAL_DIR="$REPO/tools/vigil_eval"
EXPECTED_BRANCH=""                                   # empty = run on whatever branch is checked out
SCENARIOS="cliff_front flat hill_choice narrow_route too_narrow no_route"
TRIALS=3; GUI=true; LIGHT=false; OFFLINE=true; BASELINE=""
TRUTH_ISO=false; EVAL=false; BAG=true
while [ $# -gt 0 ]; do
  case "$1" in
    --scenarios) SCENARIOS="$2"; shift 2 ;;
    --trials) TRIALS="$2"; shift 2 ;;
    --no-gui) GUI=false; shift ;;
    --light-camera) LIGHT=true; shift ;;
    --no-offline) OFFLINE=false; shift ;;
    --baseline) BASELINE="$2"; shift 2 ;;
    --branch) [ -n "${2:-}" ] || { echo "--branch needs a branch name"; exit 2; }
              EXPECTED_BRANCH="$2"; shift 2 ;;
    --truth-isolation) TRUTH_ISO=true; shift ;;
    --eval) EVAL=true; shift ;;
    --no-bag) BAG=false; shift ;;
    -h|--help) sed -n 2,23p "$0"; exit 0 ;;
    *) echo "unknown option $1"; exit 2 ;;
  esac
done
[ -z "$BASELINE" ] && BASELINE="$(ls -d "$HOME"/vigil-baseline-validation/2026* 2>/dev/null | sort | tail -1)"

STAMP="$(date +%Y%m%d-%H%M%S)"
OUT="/tmp/vigil-regression/$STAMP"
KEEP="$HOME/vigil-baseline-validation/regression-$STAMP"
mkdir -p "$OUT/runs"
exec > >(tee -a "$OUT/run.log") 2>&1
say()  { printf '\n=== %s ===\n' "$*"; }
note() { printf '%s\n' "$*" | tee -a "$OUT/notes.txt"; }
STARTED_PGIDS=(); CURRENT_PARTITION=""

CLEAN_ENV=(env -i HOME="$HOME" USER="${USER:-$(id -un)}" LANG="${LANG:-C.UTF-8}" TERM="${TERM:-xterm}"
    PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
    DISPLAY="${DISPLAY:-}" WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-}" XAUTHORITY="${XAUTHORITY:-}"
    XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-}" XDG_SESSION_TYPE="${XDG_SESSION_TYPE:-}"
    DBUS_SESSION_BUS_ADDRESS="${DBUS_SESSION_BUS_ADDRESS:-}"
    __NV_PRIME_RENDER_OFFLOAD="${__NV_PRIME_RENDER_OFFLOAD:-}" __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-}")
clean() { "${CLEAN_ENV[@]}" "$@"; }
MYPGID="$(ps -o pgid= -p $$ | tr -d ' ')"
ros() {
  clean bash --noprofile --norc -c "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; \
    [ -f \"$RT/install/local_setup.bash\" ] && source \"$RT/install/local_setup.bash\"; export ROS_DOMAIN_ID=71; $*"
}
group_alive() { ps -o stat= -g "$1" 2>/dev/null | grep -qv '^Z'; }   # any non-zombie process in group $1
bg_ros() {
  # bg_ros LOGFILE "COMMAND": start COMMAND (ROS sourced, domain 71) in a new process group; print its pgid
  ( exec setsid "${CLEAN_ENV[@]}" PYTHONUNBUFFERED=1 bash --noprofile --norc -c "source /opt/ros/jazzy/setup.bash >/dev/null 2>&1; \
      [ -f \"$RT/install/local_setup.bash\" ] && source \"$RT/install/local_setup.bash\"; export ROS_DOMAIN_ID=71; $2" ) > "$1" 2>&1 &
  local pid=$!; sleep 0.5
  echo "$pid"            # setsid made this process a group leader: pid == pgid; callers signal -pid
}
sweep_partition() {
  # NOTE: every variable here is local. Bash locals are dynamically scoped, so a non-local loop
  # variable named like a caller's local (it used to be "d") silently overwrote run_one's output
  # directory with the last /proc/<pid> visited -> logs/result.json were read from /proc/<pid>/.
  local part="$1" pids="" proc env
  for proc in /proc/[0-9]*; do
    [ -O "$proc" ] || continue                       # only this user's processes
    # the redirection itself must be silenced: an unreadable environ fails in the shell, before tr
    env="$( { tr '\0' '\n' < "$proc/environ"; } 2>/dev/null )" || continue
    printf '%s\n' "$env" | grep -qx "GZ_PARTITION=$part" && pids="$pids ${proc#/proc/}"
  done
  pids="$(echo $pids | tr ' ' '\n' | grep -vx "$$" | tr '\n' ' ')"
  if [ -n "${pids// /}" ]; then
    echo "Stopping leftover processes of partition $part:$pids"
    kill -INT $pids 2>/dev/null; sleep 4; kill -TERM $pids 2>/dev/null; sleep 2; kill -KILL $pids 2>/dev/null
  fi
}
cleanup_on_exit() {
  local pg
  for pg in "${STARTED_PGIDS[@]:-}"; do
    [ -n "$pg" ] && [ "$pg" != "$MYPGID" ] && kill -0 -"$pg" 2>/dev/null && { kill -INT -"$pg"; sleep 5; kill -KILL -"$pg" 2>/dev/null; }
  done
  [ -n "$CURRENT_PARTITION" ] && sweep_partition "$CURRENT_PARTITION"
  mkdir -p "$KEEP" && cp -a "$OUT/." "$KEEP/" && echo "Evidence copied to: $KEEP"
}
trap cleanup_on_exit EXIT
trap 'echo Interrupted; exit 130' INT TERM

git_snapshot() {
  local f="$OUT/git_$1"; mkdir -p "$f"
  ( cd "$REPO" || exit 1
    git branch --show-current > "$f/branch.txt"; git status --short > "$f/status.txt"
    git status --porcelain=v1 -uall > "$f/status_porcelain.txt"; git log -10 --oneline > "$f/log.txt"
    git rev-parse HEAD > "$f/head.txt"; git rev-parse main > "$f/main.txt" 2>&1; git branch --list > "$f/branches.txt"
    git stash list > "$f/stash.txt"
    git diff --check > "$f/diff_check.txt" 2>&1; echo "exit=$?" >> "$f/diff_check.txt"
    git diff --binary | sha256sum > "$f/diff_tracked.sha256"
    # per-file content hashes of the modified tracked files, so a CHANGED diff hash can be traced to
    # the file (25 Nov 18:51 run: the diff hash changed although no tracked file's mtime did)
    git diff --name-only -z | xargs -0 -r sha256sum > "$f/diff_files.sha256" 2>&1
    git diff --summary > "$f/diff_summary.txt" 2>&1
    git ls-files --others --exclude-standard -z | xargs -0 -r sha256sum > "$f/untracked.sha256" )
}

# ============================================================ 1. git state
say "1. Git state ($REPO)"
git_snapshot before
cat "$OUT/git_before/branch.txt" "$OUT/git_before/status.txt"
BR="$(cat "$OUT/git_before/branch.txt")"
if [ -n "$EXPECTED_BRANCH" ]; then
  [ "$BR" = "$EXPECTED_BRANCH" ] || { note "STOP: branch is '$BR', expected '$EXPECTED_BRANCH'. Nothing was run."; exit 3; }
  note "branch check: on '$BR' as required by --branch"
else
  note "branch check: none requested; running on '${BR:-detached HEAD}'"
fi
CODE_ID="$(cat "$OUT/git_before/head.txt" | cut -c1-12)+worktree:$(cut -c1-12 "$OUT/git_before/diff_tracked.sha256")"
echo "$CODE_ID" > "$OUT/code_id.txt"; echo "code under test: $CODE_ID"

# ============================================================ 2. environment
say "2. Environment"
{ lsb_release -ds 2>&1; uname -r; nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1 | head -2
  echo "session=${XDG_SESSION_TYPE:-} DISPLAY=${DISPLAY:-}"; df -h "$REPO" | tail -1; } > "$OUT/environment.txt"
ros 'echo "ROS_DISTRO=$ROS_DISTRO"; gz sim --versions 2>&1 | head -1; python3 --version' >> "$OUT/environment.txt" 2>&1
cat "$OUT/environment.txt"

# ============================================================ 3. offline tests (no Gazebo)
OFFLINE_RC=skipped
if $OFFLINE; then
  say "3. Offline tests"
  ( cd "$PKG" && clean bash --noprofile --norc -c "python3 -m pytest -q -p no:cacheprovider test/test_navigation_safety.py" ) \
      > "$OUT/offline_unit.log" 2>&1; U=$?
  ( cd "$SELF_DIR" && clean bash --noprofile --norc -c "python3 -m pytest -q -p no:cacheprovider test_metrics.py" ) \
      > "$OUT/offline_metrics.log" 2>&1; M=$?
  ( cd "$PKG" && clean bash --noprofile --norc -c "python3 test/closed_loop_sim.py" ) > "$OUT/offline_closed_loop.log" 2>&1; C=$?
  find "$PKG" "$SELF_DIR" -name __pycache__ -prune -exec rm -rf {} + 2>/dev/null
  OFFLINE_RC="unit=$U metrics=$M closed_loop=$C"
  tail -1 "$OUT/offline_unit.log"; tail -1 "$OUT/offline_metrics.log"; tail -15 "$OUT/offline_closed_loop.log"
  note "offline tests: $OFFLINE_RC (0 = pass)"
fi

# ============================================================ 4. build + install check
say "4. Build"
BUILD_CMD='colcon build --symlink-install --packages-select vigil_rough_terrain --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
( cd "$RT" && clean bash --noprofile --norc -c "source /opt/ros/jazzy/setup.bash && $BUILD_CMD" ) > "$OUT/build.log" 2>&1
BUILD_RC=$?; tail -4 "$OUT/build.log"; note "build exit code: $BUILD_RC"
[ $BUILD_RC -ne 0 ] && { note "FAIL: build failed - no simulation launched."; exit 5; }
{
  for f in terrain_mapper.py cliff_navigator.py ugv_dashboard.py drive.py planner_core.py terrain_core.py; do
    p="$RT/install/vigil_rough_terrain/lib/vigil_rough_terrain/$f"
    if [ -e "$p" ] && cmp -s "$p" "$PKG/scripts/$f"; then echo "OK      lib/$f (identical to source)"; else echo "PROBLEM lib/$f"; fi
  done
  for f in launch/vision_nav.launch.py config/vision_nav.yaml dashboard/index.html test/test_navigation_safety.py; do
    p="$RT/install/vigil_rough_terrain/share/vigil_rough_terrain/$f"
    if [ -e "$p" ] && cmp -s "$p" "$PKG/$f"; then echo "OK      share/$f"; else echo "PROBLEM share/$f"; fi
  done
  ros 'ros2 pkg executables vigil_rough_terrain'
} > "$OUT/install_check.txt" 2>&1
cat "$OUT/install_check.txt"
grep -q PROBLEM "$OUT/install_check.txt" && note "WARNING: installed files differ from source (see install_check.txt)"

# ============================================================ 5. pre-run safety checks
say "5. Pre-run checks"
port_busy() { ss -ltnH "sport = :8080" 2>/dev/null | grep -q .; }
port_busy && { note "STOP: port 8080 busy."; exit 6; }
NODES="$(ros 'timeout 15 ros2 node list --no-daemon 2>/dev/null' || true)"
[ -n "$NODES" ] && { note "STOP: ROS_DOMAIN_ID=71 already has nodes: $NODES"; exit 6; }
DAEMON_BEFORE="$(ros 'ros2 daemon status' 2>&1 | head -1)"

sim_limit() { case "$1" in narrow_route) echo 300;; too_narrow|no_route) echo 150;; *) echo 150;; esac; }
shot() {
  local f="$1" how=""
  if command -v gnome-screenshot >/dev/null && timeout 10 gnome-screenshot -f "$f" >/dev/null 2>&1 && [ -s "$f" ]; then how=gnome-screenshot
  elif command -v import >/dev/null && [ -n "${DISPLAY:-}" ] && timeout 10 import -window root "$f" >/dev/null 2>&1 && [ -s "$f" ]; then how=import
  fi
  if [ -n "$how" ]; then echo "OK $f ($how)"; else echo "SCREENSHOT FAILED $f (gnome-screenshot/import unavailable or refused on ${XDG_SESSION_TYPE:-?})"; fi
}
grab_dashboard() {
  local dir="$1" tag="$2" s="" u code
  for u in state.json map.png camera.jpg; do
    code=$(curl -s -m 5 -o "$dir/dash_${tag}_$u" -w "%{http_code}" "http://localhost:8080/$u" 2>/dev/null) || code="${code:-000}"
    s="$s $u=${code:-000}"
  done
  echo "$tag:$s" >> "$dir/dashboard_collection.txt"
}

run_one() {
  local sc="$1" n="$2" lim; lim="$(sim_limit "$sc")"
  # unique, persistent per-trial output directory (never reused, never a /proc path)
  local rundir="$OUT/runs/${sc}_t${n}"
  [ -e "$rundir" ] && rundir="$rundir.$(date +%s%N)"
  mkdir -p "$rundir" || { note "$sc t$n: cannot create $rundir"; return; }
  LAST_RUNDIR="$rundir"
  echo "$CODE_ID" > "$rundir/code_id.txt"
  CURRENT_PARTITION="vigilreg_${sc}_${n}_$$"
  local args="scenario:=$sc cleanup:=false partition:=$CURRENT_PARTITION gui:=$GUI"
  $LIGHT && args="$args camera_width:=960 camera_height:=720 camera_rate:=5"
  $TRUTH_ISO && args="$args truth_isolation:=true"
  local truth_topic=/sim/ground_truth eval_mode=baseline
  $TRUTH_ISO && { truth_topic=/eval/ground_truth; eval_mode=isolation; }
  say "Scenario $sc trial $n/$TRIALS (sim limit ${lim}s)"
  echo "ros2 launch vigil_rough_terrain vision_nav.launch.py $args" | tee "$rundir/command.txt"
  for _ in $(seq 1 15); do port_busy || break; sleep 1; done
  port_busy && { note "$sc t$n: port 8080 busy - not run"; echo '{"end_condition":"NOT_RUN_PORT_BUSY","category":"SIMULATION_FAILURE","verdict":"FAIL"}' > "$rundir/result.json"; return; }
  ( cd "$RT" && exec setsid "${CLEAN_ENV[@]}" bash --noprofile --norc -c "source /opt/ros/jazzy/setup.bash && \
      source install/local_setup.bash && export ROS_DOMAIN_ID=71 && exec ros2 launch vigil_rough_terrain vision_nav.launch.py $args" ) \
      > "$rundir/launch.log" 2>&1 &
  local lpid=$!; sleep 1
  local pgid; pgid="$(ps -o pgid= -p "$lpid" 2>/dev/null | tr -d ' ')"
  if [ -z "$pgid" ] || [ "$pgid" = "$MYPGID" ] || [ "$pgid" != "$lpid" ]; then pgid=""; note "$sc t$n: own process group not isolated; stopping by partition"; else STARTED_PGIDS+=("$pgid"); fi
  echo "pid=$lpid pgid=${pgid:-none} partition=$CURRENT_PARTITION" > "$rundir/pids.txt"
  ( for _ in $(seq 1 300); do [ -f "$rundir/nav_seen" ] && break; [ -f "$rundir/result.json" ] && break; sleep 1; done
    [ -f "$rundir/nav_seen" ] || { echo "probes: navigation never started"; echo "not attempted (navigation never started)" >> "$rundir/screenshot_status.txt"; exit 0; }
    sleep 8
    for cmd in "ros2 topic list -t --no-daemon:topic_list" "timeout 20 ros2 control list_controllers:controllers" \
               "timeout 12 ros2 topic hz /camera/depth_image:hz_depth" "timeout 12 ros2 topic hz /cmd_vel:hz_cmd_vel" \
               "timeout 10 ros2 topic echo --once --full-length /vision/cliff:echo_cliff"; do
      ros "${cmd%%:*}" > "$rundir/${cmd##*:}.txt" 2>&1; echo "${cmd##*:} exit=$?" >> "$rundir/probe_status.txt"
    done
    grab_dashboard "$rundir" running; shot "$rundir/screen_running.png" >> "$rundir/screenshot_status.txt"
  ) > "$rundir/probes.log" 2>&1 &
  local probe=$!
  local evpid="" bagpid=""
  if $EVAL; then
    # isolated evaluator: the only operational-graph reader of simulator truth; writes guard.json,
    # eval_report.json and eval_trajectory.csv when stopped (SIGINT). Each helper runs in its OWN
    # process group (setsid, like the launch) so SIGINT reaches the process itself, and the exit trap
    # can stop it. (0f44a87 signalled a subshell instead; the evaluator never stopped.)
    evpid="$(bg_ros "$rundir/evaluator.log" "exec python3 -u '$EVAL_DIR/evaluator.py' --out '$rundir' --mode $eval_mode --truth-topic $truth_topic")"
    if $BAG; then
      bagpid="$(bg_ros "$rundir/bag.log" "exec ros2 bag record -o '$rundir/bag' --topics $truth_topic /tf /tf_static /clock /navigation/status /cmd_vel /imu/data /encoders /joint_states /camera/camera_info /vision/cliff")"
    fi
    for p in $evpid $bagpid; do STARTED_PGIDS+=("$p"); done
    echo "evaluator pgid=${evpid:-none} bag pgid=${bagpid:-none} mode=$eval_mode truth=$truth_topic" >> "$rundir/pids.txt"
  fi
  ros "python3 '$SELF_DIR/monitor.py' --scenario $sc --trial $n --out '$rundir' --scenarios '$PKG/scripts/scenarios.py' \
       --config '$PKG/config/vision_nav.yaml' --sim-limit $lim --wall-limit $(( lim * 6 + 300 )) --startup-timeout 300 \
       --truth-topic $truth_topic" \
       > "$rundir/monitor.log" 2>&1
  local p
  for p in $evpid $bagpid; do kill -INT -"$p" 2>/dev/null; done
  for p in $evpid $bagpid; do
    for _ in $(seq 1 25); do group_alive "$p" || break; sleep 1; done
    group_alive "$p" && { note "$sc t$n: helper group $p did not stop on SIGINT - killed"; kill -KILL -"$p" 2>/dev/null; }
  done
  [ -f "$rundir/result.json" ] || { note "$sc t$n: MONITOR FAILURE - no result.json (see monitor.log)"; \
      echo '{"end_condition":"MONITOR_ERROR","category":"MONITOR_FAILURE","verdict":"FAIL"}' > "$rundir/result.json"; }
  grab_dashboard "$rundir" final; ros 'timeout 10 ros2 topic echo --once --full-length /navigation/status' > "$rundir/echo_status_final.txt" 2>&1
  shot "$rundir/screen_final.png" >> "$rundir/screenshot_status.txt"
  wait "$probe" 2>/dev/null
  local stop_line=0
  [ -f "$rundir/launch.log" ] && stop_line=$(wc -l < "$rundir/launch.log")
  echo "$stop_line" > "$rundir/launch_stop_line.txt"
  if [ -n "$pgid" ]; then
    kill -INT -"$pgid" 2>/dev/null
    for _ in $(seq 1 20); do kill -0 -"$pgid" 2>/dev/null || break; sleep 1; done
    kill -TERM -"$pgid" 2>/dev/null; sleep 3; kill -KILL -"$pgid" 2>/dev/null
    STARTED_PGIDS=("${STARTED_PGIDS[@]/$pgid}")
  fi
  sweep_partition "$CURRENT_PARTITION"; CURRENT_PARTITION=""
  case "$rundir" in
    "$OUT/runs/"*) ;;
    *) note "INTERNAL ERROR: trial directory changed to '$rundir' - post-processing skipped"; return ;;
  esac
  # log analysis: runtime messages (before the stop) vs. shutdown messages (after)
  if [ -f "$rundir/launch.log" ]; then
    # the navigator's own terminal-state report (logged at ERROR on purpose) is the expected outcome
    # of the unreachable scenarios, not a runtime error: kept separately in terminal_state_log.txt
    head -n "$stop_line" "$rundir/launch.log" | grep -nE "\[ERROR\]|Traceback|\[FATAL\]|process has died|Exception" \
      | grep -vE "\[cliff_navigator\]: (GOAL_UNREACHABLE|RECOVERY_LIMIT_REACHED|SENSOR_TIMEOUT|LOCALIZATION_LOST): " \
      > "$rundir/runtime_errors.txt" || true
    head -n "$stop_line" "$rundir/launch.log" \
      | grep -nE "\[cliff_navigator\]: (GOAL_UNREACHABLE|RECOVERY_LIMIT_REACHED|SENSOR_TIMEOUT|LOCALIZATION_LOST): " \
      > "$rundir/terminal_state_log.txt" || true
    head -n "$stop_line" "$rundir/launch.log" | grep -nE "\[WARN\]" > "$rundir/runtime_warnings.txt" || true
    tail -n +"$((stop_line + 1))" "$rundir/launch.log" | grep -nE "\[ERROR\]|Traceback|process has died|\[FATAL\]" > "$rundir/shutdown_messages.txt" || true
  else
    note "$sc t$n: launch.log missing - launch output not captured"
  fi
  python3 - "$rundir" "$sc" "$n" <<'PY'
import json, sys, pathlib
d, scen, trial = pathlib.Path(sys.argv[1]), sys.argv[2], sys.argv[3]
if not d.is_dir() or str(d).startswith('/proc'):
    print(f'{scen} t{trial}: POST-PROCESSING SKIPPED - bad trial directory {d}'); sys.exit(0)
missing = []
res = d / 'result.json'
try:
    r = json.loads(res.read_text())
except FileNotFoundError:
    missing.append('result.json')
    r = dict(scenario=scen, trial=trial, end_condition='MONITOR_ERROR', category='MONITOR_FAILURE', verdict='FAIL',
             note='result.json was never written (see monitor.log)')
except ValueError as e:
    (d / 'result.json.corrupt').write_bytes(res.read_bytes())
    r = dict(scenario=scen, trial=trial, end_condition='MONITOR_ERROR', category='MONITOR_FAILURE', verdict='FAIL',
             note=f'result.json unreadable ({e}); original kept as result.json.corrupt')
def lines(name):
    p = d / name
    if not p.exists():
        missing.append(name); return None
    try:
        return sum(1 for _ in open(p, errors='replace'))
    except OSError:
        missing.append(name); return None
def text(name, default):
    p = d / name
    try:
        return p.read_text(errors='replace').strip().splitlines()
    except OSError:
        missing.append(name); return default
# Simulator start-up failure: the ros2_control wheel controller was never activated (seen on the
# Legion 25 Nov 21:52, cliff_front t1 / no_route t2: the controller manager's service response to
# the spawner timed out, the wheels could not turn). Such a trial says nothing about navigation.
ctrl_ok = False
try:
    if 'Configured and activated wheel_controller' in (d / 'launch.log').read_text(errors='replace'):
        ctrl_ok = True
except OSError:
    pass
try:
    ctl = (d / 'controllers.txt').read_text(errors='replace')
    if any('wheel_controller' in ln and 'active' in ln.split() for ln in ctl.splitlines()):
        ctrl_ok = True
except OSError:
    pass
r['wheel_controller_active'] = ctrl_ok
if not ctrl_ok and r.get('category') != 'GOAL_REACHED':
    r['navigation_category_before_override'] = r.get('category')
    r['navigation_verdict_before_override'] = r.get('verdict')
    r['category'] = 'SIMULATION_FAILURE (wheel controller never activated)'
    r['verdict'] = 'FAIL'
    (d / 'sim_failure').write_text('wheel controller never activated\n')
r['runtime_error_lines'] = lines('runtime_errors.txt')
r['runtime_warning_lines'] = lines('runtime_warnings.txt')
r['shutdown_message_lines'] = lines('shutdown_messages.txt')
r['screenshots'] = text('screenshot_status.txt', ['not attempted'])
r['dashboard_collection'] = text('dashboard_collection.txt', [])
r['diagnostics_missing'] = missing      # optional diagnostics that were not collected (never hidden)
r.setdefault('scenario', scen); r.setdefault('trial', trial)
res.write_text(json.dumps(r, indent=2))
print(f"{r.get('scenario')} t{r.get('trial')}: {r.get('category')} [{r.get('verdict')}] goal={r.get('goal_time_sim_s')} "
      f"terminal={r.get('terminal_failure')} replans={r.get('replans')}/{r.get('hazard_replans')} "
      f"clearance={r.get('min_footprint_clearance_m')} runtime_errors={r['runtime_error_lines']}"
      + (f" missing={missing}" if missing else ''))
PY
  if $EVAL; then
    python3 - "$rundir" <<'PY'
import json, sys
from pathlib import Path
d = Path(sys.argv[1])
try:
    e = json.loads((d / 'eval_report.json').read_text()); m = e['metrics']
except (OSError, ValueError, KeyError) as x:
    print(f"  eval: NO REPORT ({x}) - see evaluator.log"); sys.exit(0)
ate = (m.get('ate_start') or {}).get('rmse')
r2 = ((m.get('rpe') or {}).get('2m') or {}).get('p95')
f = lambda v: 'n/a' if v is None else f'{v:.3f}'
print(f"  eval: mode={e['mode']} guard={e['guard_verdict']} truth_msgs={e['truth_messages']} samples={m['samples']} "
      f"availability={f(m['availability'])} ATE_start_rmse={f(ate)} RPE2m_p95={f(r2)} "
      f"drift_pct={f(m.get('drift_pct'))} gt_max_displacement={f(m.get('gt_max_displacement_m'))}")
PY
  fi
  sleep 3
}

# ============================================================ 6. scenarios x trials
say "6. Gazebo scenarios"
SIM_RETRIES=0
for sc in $SCENARIOS; do for n in $(seq 1 "$TRIALS"); do
  LAST_RUNDIR=""
  run_one "$sc" "$n"
  if [ -n "$LAST_RUNDIR" ] && [ -f "$LAST_RUNDIR/sim_failure" ]; then
    # keep the failed attempt (it is reported), mark it superseded, run the trial once more
    echo "retried" > "$LAST_RUNDIR/superseded_by_retry"
    SIM_RETRIES=$((SIM_RETRIES + 1))
    note "$sc t$n: simulator start-up failure (wheel controller never activated) - retrying this trial once"
    run_one "$sc" "$n"
  fi
done; done
echo "simulator start-up failures retried: $SIM_RETRIES" >> "$OUT/notes.txt"
echo "truth isolation: $TRUTH_ISO, evaluator: $EVAL, bag: $($EVAL && $BAG && echo yes || echo no)" >> "$OUT/notes.txt"

# ============================================================ 7. summary + git verification
say "7. Summary"
DAEMON_AFTER="$(ros 'ros2 daemon status' 2>&1 | head -1)"
echo "$DAEMON_BEFORE" | grep -q "not running" && ! echo "$DAEMON_AFTER" | grep -q "not running" && ros 'ros2 daemon stop' >/dev/null 2>&1
git_snapshot after
python3 "$SELF_DIR/summarize.py" "$OUT" "$BASELINE" "$OFFLINE_RC" > "$OUT/SUMMARY.md"
cat "$OUT/SUMMARY.md"
echo; echo "No git command that writes (commit/push/stash/reset/clean/checkout) was executed."
