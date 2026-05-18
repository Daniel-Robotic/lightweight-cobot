FROM evilfisru/ros:jazzy-base-noble

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-rmw-cyclonedds-cpp \
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
