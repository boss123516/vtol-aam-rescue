#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "[1/6] Waiting for MAVROS connection..."
CONNECTED=0
for i in {1..30}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if echo "$STATE" | grep -q "connected: true"; then
    CONNECTED=1
    break
  fi
  sleep 0.5
done

if [ "$CONNECTED" -ne 1 ]; then
  echo "[ERROR] MAVROS is not connected. Is PX4 SITL running?"
  exit 1
fi

echo "[STATE]"
ros2 topic echo /mavros/state --once || true

echo "[2/6] Arming vehicle..."
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}" || true

echo "[3/6] Waiting until armed=true..."
ARMED=0
for i in {1..20}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  echo "$STATE" | grep -q "armed: true" && {
    echo "$STATE"
    ARMED=1
    break
  }
  sleep 0.5
done

if [ "$ARMED" -ne 1 ]; then
  echo "[ERROR] Vehicle did not arm. Check QGC/PX4 preflight messages."
  exit 1
fi

echo "[4/6] Requesting AUTO.TAKEOFF..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.TAKEOFF'}" || true

echo "[4.5/6] Waiting for AUTO.TAKEOFF mode..."
TAKEOFF_MODE=0
for i in {1..20}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if echo "$STATE" | grep -q "mode: AUTO.TAKEOFF"; then
    TAKEOFF_MODE=1
    break
  fi
  sleep 0.5
done

if [ "$TAKEOFF_MODE" -ne 1 ]; then
  echo "[WARN] AUTO.TAKEOFF was not observed yet. Continuing to print diagnostics."
fi

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
