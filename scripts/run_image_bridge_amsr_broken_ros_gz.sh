#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

# This bridge form may work on some ros_gz versions. If it publishes no data,
# use scripts/run_gz_image_republisher.sh instead.
GZ_TOPIC="${GZ_CAMERA_TOPIC:-/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image}"
ROS_TOPIC="${ROS_IMAGE_TOPIC:-/image_raw}"

ros2 run ros_gz_bridge parameter_bridge \
  "${GZ_TOPIC}@sensor_msgs/msg/Image@gz.msgs.Image" \
  --ros-args -r "${GZ_TOPIC}:=${ROS_TOPIC}"