#!/bin/bash
ENV_FILE="$(pwd)/env/jetson"
CONTAINER_NAME="${ASUQTR_CONTAINER_NAME:-asuqtr_ros2}"

if ! docker images --format json | grep -q "asuqtr_ros2"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

docker run -it --rm \
  --privileged \
  --group-add dialout \
  --net=host \
  --shm-size=1gb \
  -v /dev:/dev \
  -v $(pwd)/workspace:/workspace \
  -v /tmp/argus_socket:/tmp/argus_socket \
  -v /mnt/nvme:/mnt/nvme \
  --name "$CONTAINER_NAME" \
  asuqtr_ros2
