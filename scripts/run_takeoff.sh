#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[1/6] Waiting for MAVROS connection..."
for i in {1..30}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  echo "$STATE" | grep -q "connected: true" && break
  sleep 0.5
done

echo "[STATE]"
ros2 topic echo /mavros/state --once || true

echo "[2/6] Arming vehicle..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" || true

echo "[3/6] Waiting until armed=true..."
for i in {1..20}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  echo "$STATE" | grep -q "armed: true" && {
    echo "$STATE"
    break
  }
  sleep 0.5
done

echo "[4/6] Requesting AUTO.TAKEOFF..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.TAKEOFF'}" || true

echo "[5/6] Waiting and checking altitude..."
sleep 3

echo "[STATE]"
ros2 topic echo /mavros/state --once || true

echo "[LOCAL POSITION]"
ros2 topic echo /mavros/local_position/pose --once || true

echo "[6/6] If z is increasing or PX4 says 'Takeoff detected', takeoff command worked."
echo "      If armed is still false and z stays 0, use PX4 shell for debug:"
echo "      commander arm -f"
echo "      commander takeoff"
