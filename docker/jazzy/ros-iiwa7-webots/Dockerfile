FROM evilfisru/ros:jazzy-base-noble

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
    ros-jazzy-rmw-cyclonedds-cpp \
    # ros-jazzy-webots-ros2-control \
    python3-pil \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws

COPY src/ src/

RUN apt-get update && \
    rosdep install --from-paths src -i -r -y && \
    rm -rf /var/lib/apt/lists/*

RUN . /opt/ros/$ROS_DISTRO/setup.sh && \
    colcon build --mixin release

RUN echo ". /ros2_ws/install/setup.bash" >> /root/.bashrc

CMD ["bash"]
