#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

START_SEQ="${START_SEQ:-1}"

echo "[1/6] Current MAVROS state:"
ros2 topic echo /mavros/state --once || true

echo "[2/6] Current local position:"
ros2 topic echo /mavros/local_position/pose --once || true

echo "[3/6] Set current mission item to ${START_SEQ}..."
ros2 service call /mavros/mission/set_current mavros_msgs/srv/WaypointSetCurrent "{wp_seq: ${START_SEQ}}" || true

sleep 1

echo "[4/6] Set AUTO.MISSION..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.MISSION'}" || true

sleep 2

echo "[5/6] State after AUTO.MISSION request:"
ros2 topic echo /mavros/state --once || true

echo "[6/6] Watch progress:"
echo "  ros2 topic echo /mavros/mission/reached"
echo "  ros2 topic echo /mavros/local_position/pose"
