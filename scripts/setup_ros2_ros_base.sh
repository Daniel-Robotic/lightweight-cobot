#!/bin/bash
# Install ROS2 Jazzy (ros-base) on Ubuntu 24.04.
# Emits PROGRESS:<pct>:<label> lines so the Python caller can update its progress bar.
# Устанавливает ROS2 Jazzy (ros-base) на Ubuntu 24.04.
# Выводит строки PROGRESS:<pct>:<метка> для обновления прогресс-бара в Python.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

PROGRESS() { echo "PROGRESS:$1:$2"; }

PROGRESS 0 "Checking dpkg state..."
echo "Checking dpkg state..."
sudo dpkg --configure -a

PROGRESS 5 "Setting up locale..."
echo "Setting up locale..."
sudo apt-get update -q
sudo apt-get install -y -q locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

PROGRESS 12 "Adding universe repository..."
echo "Adding universe repository..."
sudo apt-get install -y -q software-properties-common
sudo add-apt-repository -y universe

PROGRESS 20 "Adding ROS2 GPG key..."
echo "Adding ROS2 GPG key..."
sudo apt-get update -q
sudo apt-get install -y -q curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

PROGRESS 26 "Configuring ROS2 apt repository..."
echo "Configuring ROS2 apt repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

PROGRESS 32 "Updating package lists..."
echo "Updating package lists..."
sudo apt-get update -q
sudo apt-get upgrade -y -q

PROGRESS 38 "Installing ros-jazzy-ros-base (this may take a while)..."
echo "Installing ros-jazzy-ros-base..."
sudo apt-get install -y ros-jazzy-ros-base

PROGRESS 78 "Installing ROS2 dev tools..."
echo "Installing ROS2 dev tools..."
sudo apt-get install -y ros-dev-tools

PROGRESS 85 "Installing Python build tools..."
echo "Installing Python build tools..."
sudo apt-get install -y python3-pip python3-venv python3-dev

PROGRESS 90 "Initializing rosdep..."
echo "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
sudo sed -i '/ruby\.yaml/d; /fuerte\.yaml/d' /etc/ros/rosdep/sources.list.d/20-default.list
rosdep update

PROGRESS 95 "Configuring pip for system installs..."
echo "Configuring pip for system-wide installs (PEP 668 override)..."
sudo mkdir -p /root/.config/pip
if ! sudo grep -qs 'break-system-packages' /root/.config/pip/pip.conf 2>/dev/null; then
    printf '[global]\nbreak-system-packages = true\n' | sudo tee -a /root/.config/pip/pip.conf > /dev/null
fi
# rosdep calls `pip install -U <pkg>` which upgrades every transitive dependency,
# including packages installed by apt that have no pip RECORD file, causing an
# uninstall failure. Pre-installing these packages with --ignore-installed creates
# pip RECORD files for all their transitive deps so the later rosdep upgrade succeeds.
# fastapi>=0.100.0 + starlette>=0.27.0 are pinned together to avoid the
# "Router.__init__() got an unexpected keyword argument 'on_startup'" error that
# occurs when the apt-installed fastapi (old) is mixed with a newer pip starlette.
sudo pip3 install --break-system-packages --ignore-installed \
    "fastapi>=0.100.0" "starlette>=0.27.0" fastmcp

PROGRESS 100 "Done"
echo "ROS2 Jazzy (ros-base) installed successfully."
