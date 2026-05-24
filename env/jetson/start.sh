#!/bin/bash
ENV_FILE="$(pwd)/env/jetson"

if ! docker images --format json | grep -q "ros2-jetson"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

docker run -it --rm \
  --group-add dialout \
  --net=host \
  -v $(pwd)/workspace:/workspace \
  --name ros2-desktop \
  ros2-jetson
