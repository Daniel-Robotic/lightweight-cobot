Подмена файла по пути обязательна: `/opt/ros/rolling/lib/webots_ros2_driver/ros2_supervisor.py`

> необходимо `warn` заменить на `warning` в логере

```bash

sudo apt install -y ros-${ROS_DISTRO}-webots-ros2 \
                    ros-${ROS_DISTRO}-ros2-control \ 
                    ros-${ROS_DISTRO}-ros2-controllers \
                    ros-${ROS_DISTRO}-moveit-* \
```