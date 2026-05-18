FROM evilfisru/ros:jazzy-base-noble
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

COPY src/iiwa_* src/

RUN apt-get update && \
    rosdep install --from-paths src -i -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin release

RUN if [ "$BUILD_TYPE" = "release" ]; then rm -rf src/ build/ log/; fi

RUN echo ". /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
