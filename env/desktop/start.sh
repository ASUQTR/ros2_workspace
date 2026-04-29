#!/bin/bash
ENV_FILE="$(pwd)/env/desktop"

if ! docker images --format json | grep -q "ros2-humble"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

docker run -it --rm \
  --device=/dev/ttyUSB0 \
  --group-add dialout \
  --net=host \
  -v $(pwd)/workspace:/workspace \
  ros2-humble
