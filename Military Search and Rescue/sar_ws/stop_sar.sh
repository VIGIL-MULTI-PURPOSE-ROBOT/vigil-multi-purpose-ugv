#!/usr/bin/env bash
# Stop every process of the vigil_sar simulation (Gazebo server/GUI, bridge, nodes, dashboard).
# Only processes started by the vigil_sar launch files are touched (GZ_PARTITION=vigil_sar
# or a vigil_sar executable); the original vigil_rough_terrain workspace is never affected.
set -u
pids=""
for p in /proc/[0-9]*; do
  pid=${p#/proc/}
  [[ "$pid" == "$$" ]] && continue
  if tr '\0' '\n' < "$p/environ" 2>/dev/null | grep -qx 'GZ_PARTITION=vigil_sar'; then pids="$pids $pid"; continue; fi
  if tr '\0' ' ' < "$p/cmdline" 2>/dev/null | grep -q 'lib/vigil_sar/'; then pids="$pids $pid"; fi
done
if [[ -z "${pids// }" ]]; then echo "[stop_sar] nothing running"; exit 0; fi
echo "[stop_sar] stopping:$pids"
kill -INT $pids 2>/dev/null; sleep 3
kill -TERM $pids 2>/dev/null; sleep 2
kill -KILL $pids 2>/dev/null
echo "[stop_sar] done"
