ARG IMAGE=evilfisru/ros:base-jazzy
FROM ${IMAGE}

ARG BUILD_TYPE=release

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV WEBOTS_HOME=/usr/local/webots
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV GALLIUM_DRIVER=llvmpipe

# X11 utilities and Mesa software renderer
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    libx11-6 \
    x11-utils \
    libglu1-mesa \
    libgl1-mesa-dri \
    mesa-utils \
    libegl-mesa0 \
    && rm -rf /var/lib/apt/lists/*

# Install Webots R2025a (compatible with ros-jazzy-webots-ros2)
RUN wget -q https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb \
        -O /tmp/webots.deb && \
    apt-get update && \
    apt-get install -y /tmp/webots.deb && \
    rm /tmp/webots.deb && \
    rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-webots-ros2 \
    ros-${ROS_DISTRO}-ros2-control \ 
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-moveit-py \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-rosbag2-storage-mcap \
    python3-pil \
    && rm -rf /var/lib/apt/lists/* \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src/iiwa_bringup src/iiwa_bringup
COPY src/iiwa_config src/iiwa_config
COPY src/iiwa_controller src/iiwa_controller
COPY src/iiwa_description src/iiwa_description
COPY src/iiwa_msgs src/iiwa_msgs
COPY src/iiwa_planning src/iiwa_planning
COPY src/iiwa_utils src/iiwa_utils

RUN apt-get update && \
    rosdep install --from-paths src -i -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --mixin release

RUN if [ "${BUILD_TYPE}" = "release" ]; then rm -rf src/ build/ log/; fi

RUN echo ". /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
