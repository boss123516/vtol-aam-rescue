#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[1/7] Checking MAVROS state..."
ros2 topic echo /mavros/state --once || true

echo "[2/7] Checking uploaded mission..."
ros2 topic echo /mavros/mission/waypoints --once || true

echo "[3/7] Arming vehicle..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" || true

echo "[4/7] Waiting for armed=true..."
for i in {1..20}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  echo "$STATE" | grep -q "armed: true" && {
    echo "$STATE"
    break
  }
  sleep 0.5
done

echo "[5/7] Setting AUTO.MISSION mode..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.MISSION'}" || true

echo "[6/7] Current state:"
ros2 topic echo /mavros/state --once || true

echo "[7/7] Watch mission progress with:"
echo "  ros2 topic echo /mavros/mission/reached"
echo "  ros2 topic echo /mavros/local_position/pose"
