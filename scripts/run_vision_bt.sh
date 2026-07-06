#!/usr/bin/env bash
set -eo pipefail

# BT vision entry point: starts yolo_node, vision_tracker, and precision_lander
# together. Run scripts/run_gz_image_republisher.sh separately to publish the
# shared camera topic from Gazebo. Use scripts/run_vision.sh for the legacy
# yolo-only flow.

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

YOLO_INITIAL_TARGET="${YOLO_INITIAL_TARGET:-disabled}"
VISION_OBB_CONFIDENCE="${VISION_OBB_CONFIDENCE:-0.20}"
VISION_TRACKING_HOLD_SEC="${VISION_TRACKING_HOLD_SEC:-75.0}"
VISION_INFERENCE_INTERVAL_SEC="${VISION_INFERENCE_INTERVAL_SEC:-0.25}"
echo "[vision_bt] image_topic=${ROS_IMAGE_TOPIC:-/image_raw} initial_target=${YOLO_INITIAL_TARGET} obb_confidence=${VISION_OBB_CONFIDENCE} tracking_hold_sec=${VISION_TRACKING_HOLD_SEC} inference_interval_sec=${VISION_INFERENCE_INTERVAL_SEC}"

ros2 launch krac_vision vision_bt.launch.py \
  image_topic:="${ROS_IMAGE_TOPIC:-/image_raw}" \
  initial_target:="${YOLO_INITIAL_TARGET}" \
  obb_confidence:="${VISION_OBB_CONFIDENCE}" \
  tracking_hold_sec:="${VISION_TRACKING_HOLD_SEC}" \
  inference_interval_sec:="${VISION_INFERENCE_INTERVAL_SEC}"
