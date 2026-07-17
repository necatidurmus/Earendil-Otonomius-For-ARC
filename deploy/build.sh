#!/bin/bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(dirname "$SCRIPT_DIR")"
cd "$WS_DIR"
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y 2>/dev/null || true

# Build interfaces first (other packages depend on them)
if ! colcon build --packages-select earendil_interfaces --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    echo "ERROR: earendil_interfaces build failed" >&2
    exit 1
fi

# Build all packages
if ! colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release; then
    echo "ERROR: Full build failed" >&2
    exit 1
fi

echo "Build complete! Source: source $WS_DIR/install/setup.bash"
