#!/usr/bin/env bash
set -uo pipefail

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_LOG_BASE="${ROS_LOG_DIR:-/tmp/krac_ros_logs}"
RUN_LOG_DIR="${RUN_LOG_DIR:-$ROS_LOG_BASE/bt_run_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_LOG_DIR"
STATUS=130

export RUN_LOG_DIR
export PRINT_BT_TRANSITIONS="${PRINT_BT_TRANSITIONS:-true}"
export VEHICLE_MODEL="${VEHICLE_MODEL:-gz_standard_vtol}"
export GZ_MODEL_NAME="${GZ_MODEL_NAME:-standard_vtol_0}"
export START_AUTO_SPAWN="${START_AUTO_SPAWN:-true}"
export START_IMAGE_REPUBLISHER="${START_IMAGE_REPUBLISHER:-true}"
export START_VISION_BT="${START_VISION_BT:-true}"
export APPLY_PX4_ASSETS="${APPLY_PX4_ASSETS:-true}"
export AUTO_SPAWN_MODE="${AUTO_SPAWN_MODE:-after_world}"
export YOLO_INITIAL_TARGET="${YOLO_INITIAL_TARGET:-disabled}"
export START_RQT_GRAPH="${START_RQT_GRAPH:-true}"
export START_VISION_VIEW="${START_VISION_VIEW:-true}"

sample_topics() {
  set +u
  source /opt/ros/humble/setup.bash
  source "$REPO_DIR/ros2_ws/install/setup.bash"
  set -u
  export ROS_LOG_DIR="$RUN_LOG_DIR/sampler_ros_logs"
  mkdir -p "$ROS_LOG_DIR"
  ros2 daemon stop >/dev/null 2>&1 || true

  while true; do
    echo "===== $(date '+%Y-%m-%d %H:%M:%S %Z') ====="
    echo "--- nodes ---"
    timeout 4 ros2 node list --no-daemon 2>&1 || true
    echo "--- /mavros/state ---"
    timeout 4 ros2 topic echo /mavros/state --once 2>&1 || true
    echo "--- /mavros/extended_state ---"
    timeout 4 ros2 topic echo /mavros/extended_state --once 2>&1 || true
    echo "--- /mavros/global_position/rel_alt ---"
    timeout 4 ros2 topic echo /mavros/global_position/rel_alt --once 2>&1 || true
    echo "--- /mavros/mission/reached ---"
    timeout 4 ros2 topic echo /mavros/mission/reached --once 2>&1 || true
    echo "--- /vision/target_error ---"
    timeout 4 ros2 topic echo /vision/target_error --once 2>&1 || true
    sleep "${BT_DEBUG_SAMPLE_PERIOD_SEC:-10}"
  done
}

sample_topics >"$RUN_LOG_DIR/topic_samples.log" 2>&1 &
SAMPLER_PID=$!

cleanup() {
  kill "$SAMPLER_PID" 2>/dev/null || true
}
trap cleanup EXIT

echo "[BT-DEBUG] run_log_dir=${RUN_LOG_DIR}"
"$REPO_DIR/scripts/run_sitl_bt.sh"
STATUS=$?

cleanup
"$REPO_DIR/scripts/collect_bt_debug_logs.sh" "$RUN_LOG_DIR" || true

echo "[BT-DEBUG] exit_code=${STATUS}"
exit "$STATUS"
