#!/bin/bash
# Install ROS2 Jazzy (ros-base) on Ubuntu 24.04.
# Emits PROGRESS:<pct>:<label> lines so the Python caller can update its progress bar.
# Устанавливает ROS2 Jazzy (ros-base) на Ubuntu 24.04.
# Выводит строки PROGRESS:<pct>:<метка> для обновления прогресс-бара в Python.
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive

PROGRESS() { echo "PROGRESS:$1:$2"; }

PROGRESS 0 "Setting up locale..."
echo "Setting up locale..."
sudo apt-get update -q
sudo apt-get install -y -q locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

PROGRESS 8 "Adding universe repository..."
echo "Adding universe repository..."
sudo apt-get install -y -q software-properties-common
sudo add-apt-repository -y universe

PROGRESS 16 "Adding ROS2 GPG key..."
echo "Adding ROS2 GPG key..."
sudo apt-get update -q
sudo apt-get install -y -q curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

PROGRESS 22 "Configuring ROS2 apt repository..."
echo "Configuring ROS2 apt repository..."
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

PROGRESS 28 "Updating package lists..."
echo "Updating package lists..."
sudo apt-get update -q
sudo apt-get upgrade -y -q

PROGRESS 35 "Installing ros-jazzy-ros-base (this may take a while)..."
echo "Installing ros-jazzy-ros-base..."
sudo apt-get install -y ros-jazzy-ros-base

PROGRESS 75 "Installing ROS2 dev tools..."
echo "Installing ROS2 dev tools..."
sudo apt-get install -y ros-dev-tools

PROGRESS 88 "Initializing rosdep..."
echo "Initializing rosdep..."
sudo rosdep init 2>/dev/null || true
rosdep update

PROGRESS 100 "Done"
echo "ROS2 Jazzy (ros-base) installed successfully."
