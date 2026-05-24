#!/bin/bash
# Run the Webots container with X11 forwarding.
#
# Usage:
#   ./run.sh [--gpu nvidia|mesa|software] [CMD...]
#
# Examples:
#   ./run.sh                                  — software rendering (llvmpipe), по умолчанию
#   ./run.sh --gpu mesa                       — Intel/AMD GPU через DRI (ноутбук без NVIDIA)
#   ./run.sh --gpu nvidia                     — NVIDIA GPU rendering
#   ./run.sh --gpu nvidia ros2 launch ...     — NVIDIA GPU rendering + команда

set -e

IMAGE="${WEBOTS_IMAGE:-evilfisru/lwc:webots-jazzy-dev}"
GPU_MODE="software"

# Resolve the project root (two levels above this script: docker/jazzy/ros-iiwa7-webots → project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
CONFIG_FILE="${PROJECT_ROOT}/cobot-setting.yaml"

# Parse --gpu flag
if [[ "${1}" == "--gpu" ]]; then
    GPU_MODE="${2:?--gpu requires an argument: nvidia|mesa|software}"
    shift 2
fi

# Allow local Docker processes to connect to the X server
xhost +local:docker > /dev/null

case "${GPU_MODE}" in
    nvidia)
        RENDER_FLAGS=(
            --gpus all
            -e NVIDIA_VISIBLE_DEVICES=all
            -e NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
        )
        ;;
    mesa)
        # Intel/AMD integrated/discrete — Mesa через DRI, без NVIDIA toolkit
        RENDER_FLAGS=(
            --device /dev/dri
        )
        ;;
    software)
        RENDER_FLAGS=(
            -e LIBGL_ALWAYS_SOFTWARE=1
            -e GALLIUM_DRIVER=llvmpipe
        )
        ;;
    *)
        echo "Unknown --gpu value: '${GPU_MODE}'. Use 'nvidia', 'mesa' or 'software'." >&2
        exit 1
        ;;
esac

docker run -it --rm \
    --name ros-webots \
    --network host \
    -e USER=root \
    -e DISPLAY="${DISPLAY}" \
    -e QT_X11_NO_MITSHM=1 \
    "${RENDER_FLAGS[@]}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    ${CONFIG_FILE:+-v "${CONFIG_FILE}:/ros2_ws/cobot-setting.yaml:ro"} \
    "${IMAGE}" \
    "$@"
