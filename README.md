Подмена файла по пути обязательна: `/opt/ros/rolling/lib/webots_ros2_driver/ros2_supervisor.py`

> необходимо `warn` заменить на `warning` в логере

```bash

sudo apt install -y ros-${ROS_DISTRO}-webots-ros2 \
                    ros-${ROS_DISTRO}-ros2-control \ 
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-moveit-* \
```

Установка moveit2 (внимательно проверяй)
```bash
sudo apt install -y build-essential \
                    cmake \
                    git \
                    python3-colcon-common-extensions \
                    python3-flake8 \
                    python3-rosdep \
                    python3-setuptools \
                    python3-vcstool \
                    wget

git clone https://github.com/moveit/moveit2.git
vcs import --recursive < moveit2/moveit2.repos
sudo apt remove ros-$ROS_DISTRO-moveit*
rosdep install -r --from-paths ./src/ --ignore-src --rosdistro $ROS_DISTRO --os=ubuntu:noble -y

colcon build --mixin release
```