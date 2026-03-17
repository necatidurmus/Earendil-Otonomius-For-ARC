#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

if [ -f /home/ros/ws/install/setup.bash ]; then
    source /home/ros/ws/install/setup.bash
fi

exec "$@"
