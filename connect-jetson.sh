#!/bin/bash
set -euo pipefail

CONTAINER_NAME="${ASUQTR_CONTAINER_NAME:-asuqtr_ros2_manual_assisted}"

docker exec -it "$CONTAINER_NAME" /bin/bash
