#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run krac_vision gz_image_republisher --ros-args \
  -p gz_topic:="${GZ_CAMERA_TOPIC:-/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image}" \
  -p ros_topic:="${ROS_IMAGE_TOPIC:-/image_raw}" \
  -p rate_hz:="${IMAGE_RATE_HZ:-5.0}"