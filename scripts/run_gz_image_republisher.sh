#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

GZ_CAMERA_TOPIC="${GZ_CAMERA_TOPIC:-/world/default/model/standard_vtol_0/link/camera_link/sensor/camera/image}"
ROS_IMAGE_TOPIC="${ROS_IMAGE_TOPIC:-/image_raw}"

# IMAGE_BRIDGE_IMPL:
#   ros_gz  (default) - native C++ ros_gz_image image_bridge. The Harmonic-matched
#                       ros-humble-ros-gzharmonic-image package IS installed and
#                       parameter_bridge links libgz-transport13, so this bridges
#                       the camera at full rate with one persistent subscription
#                       (no per-frame process churn). Replaces the old python
#                       `gz topic -e` republisher, which pegged 94% CPU, could not
#                       drain 1280x960@10Hz (froze rqt / starved the vision PID),
#                       and left the gz server buffering frames.
#   legacy            - the old python gz_image_republisher fallback.
IMAGE_BRIDGE_IMPL="${IMAGE_BRIDGE_IMPL:-ros_gz}"

if [[ "$IMAGE_BRIDGE_IMPL" == "ros_gz" ]]; then
  echo "[image] ros_gz_image image_bridge: ${GZ_CAMERA_TOPIC} -> ${ROS_IMAGE_TOPIC}"
  # image_bridge publishes the bridged image on a ROS topic named after the gz
  # topic; remap that to the topic the vision stack expects.
  exec ros2 run ros_gz_image image_bridge "${GZ_CAMERA_TOPIC}" \
    --ros-args -r "${GZ_CAMERA_TOPIC}:=${ROS_IMAGE_TOPIC}"
else
  echo "[image] legacy python gz_image_republisher: ${GZ_CAMERA_TOPIC} -> ${ROS_IMAGE_TOPIC}"
  exec ros2 run krac_vision gz_image_republisher --ros-args \
    -p gz_topic:="${GZ_CAMERA_TOPIC}" \
    -p ros_topic:="${ROS_IMAGE_TOPIC}" \
    -p rate_hz:="${IMAGE_RATE_HZ:-5.0}"
fi
