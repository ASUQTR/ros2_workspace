#!/bin/bash
SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
ENV_FILE="$SCRIPT_DIR"

if ! docker images --format json | grep -q "ros2-jetson"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/build.sh
fi

docker rm -f ros2-desktop 2>/dev/null || true

xhost +local:docker > /dev/null 2>&1 || true

docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  --group-add dialout \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e XAUTHORITY=/root/.Xauthority \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /home/asuqtr/.Xauthority:/root/.Xauthority \
  -v $REPO_ROOT/workspace:/workspace \
  --name ros2-desktop \
  ros2-jetson
