#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/vtol-aam-rescue/ros2_ws/install/setup.bash"

ros2 run ros_gz_bridge parameter_bridge \
  "/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image" \
  --ros-args \
  -r /world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image:=/image_raw