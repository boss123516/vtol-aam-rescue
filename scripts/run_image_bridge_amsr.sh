#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

GZ_TOPIC="${GZ_CAMERA_TOPIC:-/world/default/model/standard_vtol_0/link/camera_link/sensor/camera/image}"
ROS_TOPIC="${ROS_IMAGE_TOPIC:-/image_raw}"

ros2 run ros_gz_image image_bridge "${GZ_TOPIC}" \
  --ros-args -r "${GZ_TOPIC}:=${ROS_TOPIC}"
