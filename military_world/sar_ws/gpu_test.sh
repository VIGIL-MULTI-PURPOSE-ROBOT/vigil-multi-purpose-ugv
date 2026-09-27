#!/usr/bin/env bash
# gpu_test.sh - 4 short Gazebo server runs (about 4 min): military_world + cameras on the
# Intel iGPU vs the NVIDIA GPU. Needs the worlds made by diagnose_sar.sh (sar_ws/diagnosis/*.sdf).
# Result: sar_ws/diagnosis/gpu_test.log
set -u
cd "$(dirname "$0")" || exit 1
MW="$(cd .. && pwd)"; OUT="$PWD/diagnosis"; LOG="$OUT/gpu_test.log"
[[ -f "$OUT/W1_world_rgbd.sdf" ]] || { echo "run diagnose_sar.sh first (it creates the test worlds)"; exit 1; }
export GZ_PARTITION=vigil_sar_gpu
export GZ_SIM_RESOURCE_PATH="$MW/gazebo_export${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$MW/build${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
NV=(__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia __VK_LAYER_NV_optimus=NVIDIA_only)
[[ -f /usr/share/glvnd/egl_vendor.d/10_nvidia.json ]] && NV+=(__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json)
: > "$LOG"
say() { echo "$@" | tee -a "$LOG"; }
say "nvidia GL check: $(env "${NV[@]}" glxinfo -B 2>/dev/null | grep 'OpenGL renderer' || echo 'glxinfo missing: sudo apt install mesa-utils')"
t() {  # t <label> <world> <gpu: intel|nvidia>
  local e=(); [[ "$3" == nvidia ]] && e=("${NV[@]}")
  ( env "${e[@]}" timeout 60 gz sim -v 3 -s -r --iterations 2000 "$OUT/$2.sdf" ) > "$OUT/gpu_$1.out" 2>&1
  local rc=$? r="OK"
  [[ $rc -eq 124 ]] && r="no crash in 60 s (slow)"; [[ $rc -gt 128 ]] && r="CRASHED (signal $((rc-128)))"
  [[ $rc -ne 0 && $rc -ne 124 && $rc -le 128 ]] && r="exit $rc"
  say "$1: $r"
  [[ $rc -gt 128 ]] && sed 's/\x1b\[[0-9;]*m//g' "$OUT/gpu_$1.out" | grep -aiE "stack|#[0-9]+ |signal|error" | tail -25 | sed 's/^/    /' | tee -a "$LOG" >/dev/null
}
t world_rgbd_intel   W1_world_rgbd       intel
t world_rgbd_nvidia  W1_world_rgbd       nvidia
t world_all_intel    W7_world_all_sensors intel
t world_all_nvidia   W7_world_all_sensors nvidia
say "GPU test finished. Review results: $LOG"
