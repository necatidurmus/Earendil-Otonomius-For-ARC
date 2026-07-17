#!/bin/bash
# ===========================================================================
# Earendil RPi5 Vehicle — Ubuntu 24.04 + ROS 2 Jazzy Install Script
# ===========================================================================
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

echo "============================================="
echo " Earendil RPi5 Vehicle — System Setup"
echo " Ubuntu 24.04 + ROS 2 Jazzy"
echo "============================================="

# 1. System update
echo "[1/7] Updating system..."
sudo apt update && sudo apt upgrade -y

# 2. ROS 2 Jazzy
echo "[2/7] Installing ROS 2 Jazzy..."
sudo apt install -y software-properties-common curl gnupg lsb-release
if [ ! -f /etc/apt/sources.list.d/ros2.list ]; then
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
        sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    sudo apt update
fi
sudo apt install -y \
    ros-jazzy-ros-base ros-jazzy-robot-state-publisher ros-jazzy-xacro \
    ros-jazzy-nav2-bringup ros-jazzy-nav2-bt-navigator ros-jazzy-nav2-controller \
    ros-jazzy-nav2-planner ros-jazzy-nav2-recoveries ros-jazzy-nav2-costmap-2d \
    ros-jazzy-dwb-controller ros-jazzy-slam-toolbox ros-jazzy-robot-localization \
    ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-diagnostic-updater \
    python3-colcon-common-extensions python3-rosdep python3-vcstool

# 3. Python deps
echo "[3/7] Installing Python dependencies..."
pip3 install --break-system-packages aiohttp pyserial pyyaml gpiod

# 4. rosdep
echo "[4/7] Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

# 5. udev rules
echo "[5/7] Installing udev rules..."
sudo cp "$(dirname "$0")/99-earendil.rules" /etc/udev/rules.d/ 2>/dev/null || echo "WARNING: udev rules not found"
sudo udevadm control --reload-rules && sudo udevadm trigger

# 6. GPIO/serial permissions
echo "[6/7] Setting permissions..."
sudo usermod -aG gpio,dialout,tty $USER 2>/dev/null || true

# 7. Environment
echo "[7/7] Setting up environment..."
if ! grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc; then
    echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
fi

echo ""
echo "Done! Reboot, then: bash deploy/build.sh && bash deploy/run_vehicle.sh"
