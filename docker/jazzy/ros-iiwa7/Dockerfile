ARG IMAGE=evilfisru/ros:base-jazzy
FROM ${IMAGE}

ARG BUILD_TYPE=release

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros2-control \ 
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-py \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src/iiwa_bringup src/iiwa_bringup
COPY src/iiwa_config src/iiwa_config
COPY src/iiwa_controller src/iiwa_controller
COPY src/iiwa_description src/iiwa_description
COPY src/iiwa_msgs src/iiwa_msgs
COPY src/iiwa_planning src/iiwa_planning
COPY src/iiwa_utils src/iiwa_utils
COPY src/iiwa_web src/iiwa_web
COPY rosdep.yaml rosdep.yaml

RUN echo "yaml file:///ros2_ws/rosdep.yaml" \
        > /etc/ros/rosdep/sources.list.d/50-kuka-local.list && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths src -i -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin release

RUN if [ "$BUILD_TYPE" = "release" ]; then rm -rf src/ build/ log/; fi

RUN echo ". /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
