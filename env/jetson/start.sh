#!/bin/bash
ENV_FILE="$(pwd)/env/jetson"
CONTAINER_NAME="${ASUQTR_CONTAINER_NAME:-asuqtr_ros2_manual_assisted}"

if ! docker images --format json | grep -q "asuqtr_ros2"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

docker run -it --rm \
  --privileged \
  --group-add dialout \
  --net=host \
  -v /dev:/dev \
  -v $(pwd)/workspace:/workspace \
  --name "$CONTAINER_NAME" \
  asuqtr_ros2
