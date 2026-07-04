#!/usr/bin/env bash
set -euo pipefail

SOURCE_DIR="${1:-/workspace/recordings/playback}"
DEST_DIR="${2:-${HOME}/playback_recordings_backup}"

if [[ ! -d "${SOURCE_DIR}" ]]; then
  echo "Recording source not found: ${SOURCE_DIR}" >&2
  exit 1
fi

mkdir -p "${DEST_DIR}"

shopt -s nullglob
files=("${SOURCE_DIR}"/*.json)
if (( ${#files[@]} == 0 )); then
  echo "No JSON recordings found in ${SOURCE_DIR}" >&2
  exit 1
fi

cp -a "${files[@]}" "${DEST_DIR}/"
echo "Copied ${#files[@]} recording(s) from ${SOURCE_DIR} to ${DEST_DIR}"
