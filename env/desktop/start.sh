#!/bin/bash
ENV_FILE="$(pwd)/env/desktop"

if ! docker images --format json | grep -q "ros2-humble"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

echo -e "[\033[0;36m INFO \033[0m] Starting ROS2 Humble desktop container..."
echo -e "[\033[0;36m INFO \033[0m] All host devices are accessible (IMU, DVL, etc. can be plugged after start)"

# Allow the container to open GUI windows on the host display
xhost +local:docker > /dev/null 2>&1 || true

touch ~/.docker_ros2_history

docker run -it --rm \
  --privileged \
  -v /dev:/dev \
  --group-add dialout \
  --net=host \
  -e DISPLAY=$DISPLAY \
  -e HISTFILE=/root/.bash_history \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd)/workspace:/workspace \
  -v ~/.docker_ros2_history:/root/.bash_history \
  --name ros2-desktop \
  ros2-humble
