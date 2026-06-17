#!/bin/bash
ENV_FILE="$(pwd)/env/jetson"

if ! docker images --format json | grep -q "asuqtr_ros2"; then
  echo -e "[\033[0;31m ERROR \033[0m] Missing docker image.."
  bash $ENV_FILE/setup.sh
fi

docker compose -f $ENV_FILE/docker-compose.yaml up
