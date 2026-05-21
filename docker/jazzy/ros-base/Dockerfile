ARG IMAGE=evilfisru/ros:core-jazzy
FROM ${IMAGE}

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init && \
    rosdep update --rosdistro $ROS_DISTRO || true

RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-base \
    && rm -rf /var/lib/apt/lists/*
