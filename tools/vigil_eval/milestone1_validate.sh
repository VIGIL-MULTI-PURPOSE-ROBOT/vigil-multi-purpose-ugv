#!/usr/bin/env bash
# VIGIL 2.0 milestone 1 validation: truth isolation + independent evaluation (Rock Terrain).
# Run from a checkout of vigil-ai-gps-denied, in a graphical terminal (Gazebo opens a window):
#   bash tools/vigil_eval/milestone1_validate.sh
# Read-only for Git. About 10-15 minutes. Summary: ~/vigil-baseline-validation/m1-<stamp>/summary.txt
#
#   S1 offline tests of the metric, guard and scan code
#   S2 source scan of truth users (report only)
#   S3 guard self-test on a live ROS graph (no Gazebo)
#   E1 baseline flat x1 with evaluator      expect guard GROUND_TRUTH_ASSISTED naming the truth users
#   E2 truth-isolated flat x2 + cliff_front x1, evaluator on
#        expect guard ISOLATED, navigator ends LOCALIZATION_LOST (or SENSOR_TIMEOUT), robot does not
#        move (ground-truth displacement < 0.10 m): the robot has no pose source once truth is isolated
set -u
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LOG="$HOME/vigil-baseline-validation/m1-$(date +%Y%m%d-%H%M%S)"; mkdir -p "$LOG"; S="$LOG/summary.txt"
R="$REPO/Rock Terrain/tools/regression/vigil_regression.sh"
say() { printf '\n=== %s ===\n' "$*" | tee -a "$S"; }
cd "$REPO" || exit 1
say "0. Checkout"
echo "repo: $REPO  branch: $(git branch --show-current)  HEAD: $(git rev-parse --short HEAD)  dirty files: $(git status --porcelain | wc -l)" | tee -a "$S"

say "0b. Leftover evaluation helpers from an earlier run"
# 0f44a87 could leave the evaluator and bag recorder running after a trial; they block the runner's
# "ROS_DOMAIN_ID=71 already has nodes" check. Stop only these two helpers, gently (SIGINT finalises bags).
left="$(pgrep -f 'tools/vigil_eval/evaluator.py|ros2 bag record -o /tmp/vigil-regression' || true)"
if [ -n "$left" ]; then
  echo "stopping leftover helpers: $(echo $left)" | tee -a "$S"; kill -INT $left 2>/dev/null; sleep 8
  left="$(pgrep -f 'tools/vigil_eval/evaluator.py|ros2 bag record -o /tmp/vigil-regression' || true)"
  [ -n "$left" ] && { kill -KILL $left 2>/dev/null; echo "force-stopped: $(echo $left)" | tee -a "$S"; }
else echo "none" | tee -a "$S"; fi

say "S1 offline tests (tools/vigil_eval)"
python3 -m pytest -q -p no:cacheprovider tools/vigil_eval 2>&1 | tail -2 | tee -a "$S"

say "S2 source scan (report only)"
python3 tools/vigil_eval/truth_scan.py --json "$LOG/truth_scan.json" > "$LOG/truth_scan.txt" 2>&1
tail -1 "$LOG/truth_scan.txt" | tee -a "$S"

say "S3 guard self-test"
( set +u; source /opt/ros/jazzy/setup.bash && python3 tools/vigil_eval/guard_selftest.py ) > "$LOG/guard_selftest.txt" 2>&1
grep -E "^(PASS|FAIL) |GUARD SELF-TEST" "$LOG/guard_selftest.txt" | tee -a "$S"

say "E1 baseline (ground-truth-assisted) flat x1 + evaluator"
bash "$R" --eval --no-offline --trials 1 --scenarios flat > "$LOG/e1.log" 2>&1
grep -E "^flat t[0-9]+:|^  eval:|Evidence copied to" "$LOG/e1.log" | tee -a "$S"

say "E2 truth-isolated flat x2 + cliff_front x1 + evaluator"
bash "$R" --truth-isolation --eval --no-offline --trials 2 --scenarios flat > "$LOG/e2a.log" 2>&1
grep -E "^flat t[0-9]+:|^  eval:|Evidence copied to" "$LOG/e2a.log" | tee -a "$S"
bash "$R" --truth-isolation --eval --no-offline --trials 1 --scenarios cliff_front > "$LOG/e2b.log" 2>&1
grep -E "^cliff_front t[0-9]+:|^  eval:|Evidence copied to" "$LOG/e2b.log" | tee -a "$S"

say "Guard details (truth users / violations / dormant subscribers)"
for f in $(grep -ho "Evidence copied to: .*" "$LOG"/e1.log "$LOG"/e2a.log "$LOG"/e2b.log | sed 's/Evidence copied to: //'); do
  for g in "$f"/runs/*/guard.json; do
    [ -f "$g" ] || continue
    echo "-- ${g#$HOME/}" | tee -a "$S"
    python3 -c "import json,sys; g=json.load(open(sys.argv[1])); [print('   ',k+':',x) for k in ('verdict','truth_users','violations','dormant','warnings') for x in ([g.get(k)] if k=='verdict' else g.get(k,[]))]" "$g" | tee -a "$S"
  done
done
echo; echo "Nothing was committed or pushed. Paste: $S"
