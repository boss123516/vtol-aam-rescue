#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash

START_SEQ="${START_SEQ:-1}"
STOP_LEGACY_FSM="${STOP_LEGACY_FSM:-true}"
PLAN_FILE="${PLAN_FILE:-$REPO_DIR/ros2_ws/src/krac_control/src/way3.plan}"

echo "[0/7] Waiting for MAVROS connection..."
CONNECTED=0
for i in {1..60}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if echo "$STATE" | grep -q "connected: true"; then
    CONNECTED=1
    break
  fi
  sleep 0.5
done

if [ "$CONNECTED" -ne 1 ]; then
  echo "[ERROR] MAVROS is not connected. Is ./scripts/run_sitl.sh still running?"
  exit 1
fi

if [ "$STOP_LEGACY_FSM" = "true" ]; then
  echo "[1/7] Stopping legacy vtol_fsm_P if it is running..."
  pkill -f vtol_fsm_P 2>/dev/null || true
else
  echo "[1/7] Leaving legacy vtol_fsm_P untouched (STOP_LEGACY_FSM=false)."
fi

echo "[2/7] Current MAVROS state:"
ros2 topic echo /mavros/state --once || true

echo "[3/7] Current local position:"
ros2 topic echo /mavros/local_position/pose --once || true

echo "[4/7] Checking uploaded mission..."
MISSION_COUNT=0
for i in {1..30}; do
  WPS="$(ros2 topic echo /mavros/mission/waypoints --once 2>/dev/null || true)"
  MISSION_COUNT="$(echo "$WPS" | grep -c "frame:" || true)"
  if [ "$MISSION_COUNT" -gt 0 ]; then
    echo "[OK] Mission waypoints detected: ${MISSION_COUNT}"
    break
  fi
  sleep 0.5
done

if [ "$MISSION_COUNT" -le "$START_SEQ" ]; then
  echo "[WARN] Mission has ${MISSION_COUNT} waypoint(s). Re-running mission_loader..."
  ros2 run krac_control mission_loader.py --ros-args -p plan_file:="${PLAN_FILE}" || true

  sleep 1
  WPS="$(ros2 topic echo /mavros/mission/waypoints --once 2>/dev/null || true)"
  MISSION_COUNT="$(echo "$WPS" | grep -c "frame:" || true)"

  if [ "$MISSION_COUNT" -le "$START_SEQ" ]; then
    echo "[ERROR] Mission has ${MISSION_COUNT} waypoint(s), cannot set START_SEQ=${START_SEQ}."
    echo "        Check mission_loader output in Terminal 1."
    exit 1
  fi
fi

echo "[5/7] Set current mission item to ${START_SEQ}..."
ros2 service call /mavros/mission/set_current mavros_msgs/srv/WaypointSetCurrent "{wp_seq: ${START_SEQ}}" || true

sleep 1

echo "[6/7] Set AUTO.MISSION..."
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.MISSION'}" || true

MISSION_MODE=0
for i in {1..20}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if echo "$STATE" | grep -q "mode: AUTO.MISSION"; then
    MISSION_MODE=1
    break
  fi
  sleep 0.5
done

if [ "$MISSION_MODE" -ne 1 ]; then
  echo "[WARN] AUTO.MISSION was not observed yet. Check PX4 commander messages."
fi

echo "[7/7] State after AUTO.MISSION request:"
ros2 topic echo /mavros/state --once || true

echo "Watch progress:"
echo "  ros2 topic echo /mavros/mission/reached"
echo "  ros2 topic echo /mavros/local_position/pose"
