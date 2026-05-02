#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/vtol-aam-rescue/ros2_ws/install/setup.bash"

ros2 run krac_vision gz_image_republisher
