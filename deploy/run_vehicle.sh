#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
[ -f "$WS_DIR/install/setup.bash" ] && source "$WS_DIR/install/setup.bash"
[ -f "$WS_DIR/.env" ] && set -a && source "$WS_DIR/.env" && set +a

# Check serial devices when hardware mode is on
if [ "${USE_HARDWARE:-false}" = "true" ]; then
    MISSING=""
    for dev in /dev/earendil_h7 /dev/earendil_rtk /dev/earendil_rscp /dev/earendil_lidar; do
        [ -e "$dev" ] || MISSING="$MISSING $dev"
    done
    if [ -n "$MISSING" ]; then
        echo "WARNING: Missing serial devices:$MISSING"
        echo "Check udev rules: deploy/99-earendil.rules"
        echo "Continuing anyway (nodes will retry)..."
    fi
fi

echo "Starting Earendil Vehicle..."
echo "Web UI: http://$(hostname -I | awk '{print $1}'):8080"
ros2 launch earendil_bringup vehicle.launch.py \
    use_gps:="${USE_GPS:-true}" \
    use_lidar:="${USE_LIDAR:-true}" \
    use_web:="${USE_WEB:-true}" \
    use_navigation:="${USE_NAVIGATION:-true}" \
    use_hardware:="${USE_HARDWARE:-false}"
