#!/usr/bin/env bash
set -eo pipefail
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch krac_mission sitl_vtol.launch.py
