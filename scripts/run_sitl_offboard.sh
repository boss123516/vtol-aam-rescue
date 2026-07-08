#!/usr/bin/env bash
set -eo pipefail

# krac24's latest "offboard_test" flow (commit 9f525f45, offboard_test_added):
# PX4/MAVROS + vtol_offboard (local OFFBOARD position control, hardcoded
# rescue target at x=-22.48 y=-31.59) + precision_lander/vision + gripper.
# This is NOT the full WP1-4 patrol BT mission (see run_sitl_bt.sh for that) -
# it only exercises takeoff -> fly to rescue point -> precision land -> grab
# (manual trigger via gripper_test.py / cmd/mission_proceed) -> return -> land.
# No .plan file / mission_loader is used; vtol_offboard hardcodes its target.

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
ROS_LOG_BASE="${ROS_LOG_DIR:-/tmp/krac_ros_logs}"
mkdir -p "$ROS_LOG_BASE"
RUN_LOG_DIR="${RUN_LOG_DIR:-$ROS_LOG_BASE/offboard_run_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_LOG_DIR"
export ROS_LOG_DIR="$RUN_LOG_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"

VEHICLE_MODEL="${VEHICLE_MODEL:-gz_amsr_vtol}"
GZ_MODEL_NAME="${GZ_MODEL_NAME:-amsr_vtol_0}"
WORLD_NAME="${WORLD_NAME:-default}"
APPLY_PX4_ASSETS="${APPLY_PX4_ASSETS:-true}"
START_AUTO_SPAWN="${START_AUTO_SPAWN:-true}"
START_IMAGE_REPUBLISHER="${START_IMAGE_REPUBLISHER:-true}"
START_VISION_BT="${START_VISION_BT:-true}"
AUTO_SPAWN_MODE="${AUTO_SPAWN_MODE:-after_world}"
AUTO_SPAWN_DELAY_SEC="${AUTO_SPAWN_DELAY_SEC:-2}"
export GZ_CAMERA_TOPIC="${GZ_CAMERA_TOPIC:-/world/${WORLD_NAME}/model/${GZ_MODEL_NAME}/link/camera_link/sensor/camera/image}"

if [[ "${APPLY_PX4_ASSETS}" == "true" ]]; then
  echo "[OFFBOARD] Applying PX4/Gazebo assets (camera, gripper, mission spawn models)..."
  (
    cd "$REPO_DIR"
    ./scripts/apply_px4_assets.sh
  ) >"$RUN_LOG_DIR/apply_px4_assets.log" 2>&1
fi

echo "[OFFBOARD] Logs: ${RUN_LOG_DIR}"
echo "[OFFBOARD] Starting gripper servo bridge for ${GZ_MODEL_NAME}..."
ros2 run ros_gz_bridge parameter_bridge \
  "/model/${GZ_MODEL_NAME}/servo_4@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/${GZ_MODEL_NAME}/servo_5@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/${GZ_MODEL_NAME}/servo_6@std_msgs/msg/Float64]gz.msgs.Double" \
  >"$RUN_LOG_DIR/gripper_bridge.log" 2>&1 &
GRIPPER_BRIDGE_PID=$!

ros2 launch krac_mission sitl_vtol.launch.py \
  vehicle_model:="${VEHICLE_MODEL}" \
  start_px4:=true \
  start_mavros:=true \
  start_fsm:=false \
  start_mission_loader:=false \
  start_logger:=false \
  >"$RUN_LOG_DIR/sitl_vtol.launch.log" 2>&1 &
LAUNCH_PID=$!

SUPPORT_PIDS=()

start_support_process() {
  local name="$1"
  shift
  echo "[OFFBOARD] Starting ${name}..."
  "$@" >"$RUN_LOG_DIR/${name// /_}.log" 2>&1 &
  SUPPORT_PIDS+=("$!")
}

if [[ "${START_AUTO_SPAWN}" == "true" ]]; then
  (
    echo "[OFFBOARD] Auto spawn waits for Gazebo create service in world '${WORLD_NAME}'..."
    for i in {1..90}; do
      if gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create"; then
        echo "[OFFBOARD] Gazebo create service is ready; spawning objects after ${AUTO_SPAWN_DELAY_SEC}s."
        sleep "${AUTO_SPAWN_DELAY_SEC}"
        break
      fi
      sleep 1
    done
    exec "$REPO_DIR/scripts/auto_spawn2.sh"
  ) >"$RUN_LOG_DIR/auto_spawn2.log" 2>&1 &
  SUPPORT_PIDS+=("$!")
fi

if [[ "${START_IMAGE_REPUBLISHER}" == "true" ]]; then
  start_support_process "Gazebo image republisher" "$REPO_DIR/scripts/run_gz_image_republisher.sh"
fi

if [[ "${START_VISION_BT}" == "true" ]]; then
  start_support_process "vision + precision_lander stack" "$REPO_DIR/scripts/run_vision_bt.sh"
fi

cleanup() {
  echo ""
  echo "[CLEANUP] Stopping PX4/MAVROS launch..."
  kill "$LAUNCH_PID" 2>/dev/null || true
  for pid in "${SUPPORT_PIDS[@]}"; do
    kill "$pid" 2>/dev/null || true
  done
  echo "[CLEANUP] Stopping gripper servo bridge..."
  kill "$GRIPPER_BRIDGE_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[OFFBOARD] Waiting for MAVROS connection before starting vtol_offboard..."
CONNECTED=false
for i in {1..60}; do
  STATE="$(ros2 topic echo /mavros/state --once 2>/dev/null || true)"
  if echo "$STATE" | grep -q "connected: true"; then
    echo "[OK] MAVROS connected."
    CONNECTED=true
    break
  fi
  sleep 1
done

if [[ "$CONNECTED" != "true" ]]; then
  echo "[ERROR] MAVROS did not connect within 60 seconds."
  exit 3
fi

echo "[OFFBOARD] Starting vtol_offboard node..."
echo "[OFFBOARD] To trigger grab-release after landing, run in another shell:"
echo "  ros2 service call /cmd/mission_proceed std_srvs/srv/Trigger"
echo "[OFFBOARD] For manual gripper testing, run in another shell:"
echo "  ros2 run krac_control gripper_test.py"
set +e
ros2 run krac_control vtol_offboard >"$RUN_LOG_DIR/vtol_offboard.log" 2>&1
BT_EXIT=$?
set -e

echo "[OFFBOARD] vtol_offboard exited with code ${BT_EXIT}"
exit "$BT_EXIT"
