#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
REPO_ROOT="$(dirname "$(dirname "$SCRIPT_DIR")")"
ENV="$SCRIPT_DIR"

cleanup() {
  rm -rf "$ENV/packages"
  rm -f "$ENV/requirements_xavier.txt"
}
trap cleanup EXIT

echo "[INFO] Preparation du contexte de build..."
rm -rf "$ENV/packages"
cp "$REPO_ROOT/requirements_xavier.txt" "$ENV/"
cp -r "$REPO_ROOT/workspace/packages" "$ENV/packages"

echo "[INFO] Build de l'image ros2-jetson..."
docker build -t ros2-jetson -f "$ENV/Dockerfile" "$ENV"

echo "[INFO] Build termine avec succes."
docker images | grep ros2-jetson
