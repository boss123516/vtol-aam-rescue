#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash

INITIAL_TARGET="${INITIAL_TARGET:-disabled}"
IMAGE_TOPIC="${IMAGE_TOPIC:-/image_raw}"
YOLO_DEVICE="${YOLO_DEVICE:-cpu}"
YOLO_THRESHOLD="${YOLO_THRESHOLD:-0.5}"

echo "[run_vision] initial_target=${INITIAL_TARGET} image_topic=${IMAGE_TOPIC} device=${YOLO_DEVICE} threshold=${YOLO_THRESHOLD}"

ros2 launch krac_vision yolo_detector.launch.py \
  initial_target:="${INITIAL_TARGET}" \
  image_topic:="${IMAGE_TOPIC}" \
  device:="${YOLO_DEVICE}" \
  threshold:="${YOLO_THRESHOLD}"
