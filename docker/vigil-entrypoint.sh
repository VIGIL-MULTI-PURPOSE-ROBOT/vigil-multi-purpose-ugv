#!/usr/bin/env bash
set -euo pipefail

# ROS setup scripts probe optional variables that may legitimately be unset.
set +u
source /opt/ros/jazzy/setup.bash
source /opt/vigil_ws/install/setup.bash
set -u

if [[ "${VIGIL_RENDER_MODE:-software}" == hardware ]]; then
    unset LIBGL_ALWAYS_SOFTWARE
    renderer="$(glxinfo -B)" || { echo 'Cannot access the host GPU/display.' >&2; exit 1; }
    echo "$renderer"
    if grep -Eqi 'llvmpipe|softpipe|software rasterizer' <<< "$renderer"; then
        echo 'Software rendering detected. Fix GPU/display access before using hardware mode.' >&2
        exit 1
    fi
    if [[ -n "${VIGIL_SCENARIO:-}" ]]; then
        set -- "$@" "scenario:=${VIGIL_SCENARIO}"
    fi
    exec "$@"
fi

# Xvfb gives Gazebo an OS-independent display. x11vnc and noVNC make that
# display available in the host's browser without exposing an unauthenticated
# VNC port. Compose binds the browser port to the local machine only.
export DISPLAY="${DISPLAY:-:99}"
desktop_size="${VIGIL_DESKTOP_SIZE:-1600x900x24}"
children=()

Xvfb "$DISPLAY" -screen 0 "$desktop_size" -nolisten tcp &
xvfb_pid=$!
children+=("$xvfb_pid")

display_number="${DISPLAY#:}"
display_number="${display_number%%.*}"
for _ in {1..50}; do
    [[ -S "/tmp/.X11-unix/X${display_number}" ]] && break
    kill -0 "$xvfb_pid" 2>/dev/null || {
        echo "VIGIL: virtual display failed to start" >&2
        exit 1
    }
    sleep 0.1
done

if [[ ! -S "/tmp/.X11-unix/X${display_number}" ]]; then
    echo "VIGIL: timed out waiting for virtual display" >&2
    exit 1
fi

cleanup() {
    trap - EXIT INT TERM
    for pid in "${children[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
}
trap cleanup EXIT INT TERM

openbox >/tmp/vigil-openbox.log 2>&1 &
children+=("$!")

x11vnc -display "$DISPLAY" -forever -shared -nopw -localhost -noxdamage \
    -rfbport 5900 >/tmp/vigil-x11vnc.log 2>&1 &
children+=("$!")

websockify --web=/usr/share/novnc 6080 localhost:5900 \
    >/tmp/vigil-novnc.log 2>&1 &
children+=("$!")

if [[ -n "${VIGIL_SCENARIO:-}" ]]; then
    set -- "$@" "scenario:=${VIGIL_SCENARIO}"
fi

"$@" &
app_pid=$!
children+=("$app_pid")

set +e
wait "$app_pid"
status=$?
set -e
exit "$status"
