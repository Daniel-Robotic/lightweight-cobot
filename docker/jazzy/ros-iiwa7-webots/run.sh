#!/bin/bash
# Run the Webots container with X11 forwarding.
# Usage:
#   ./run.sh                          — interactive shell
#   ./run.sh ros2 launch iiwa_bringup webots_spawn.launch.py ...

IMAGE="${WEBOTS_IMAGE:-evilfisru/lwc:webots-jazzy}"

# Allow local Docker processes to connect to the X server
xhost +local:docker > /dev/null

docker run -it --rm \
    --name ros-webots \
    --network host \
    -e USER=root \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    -e LIBGL_ALWAYS_SOFTWARE=1 \
    -e GALLIUM_DRIVER=llvmpipe \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    "${IMAGE}" \
    "$@"
