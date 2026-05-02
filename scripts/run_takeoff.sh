#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source "$HOME/vtol-aam-rescue/ros2_ws/install/setup.bash"

echo "[1/4] Waiting for MAVROS state..."
ros2 topic echo /mavros/state --once

echo "[2/4] Arming vehicle..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"

sleep 1

echo "[3/4] Setting AUTO.TAKEOFF mode..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.TAKEOFF'}"

sleep 1

echo "[4/4] Current state:"
ros2 topic echo /mavros/state --once
