#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
ENV="$REPO_ROOT/env/desktop"

cp "$REPO_ROOT/requirements_xavier.txt" "$ENV"
cp "$REPO_ROOT/underlay.repos" "$ENV"

docker build -t ros2-humble -f "$ENV/Dockerfile" "$ENV"
