#!/usr/bin/env bash
set -eo pipefail

# BT-only SITL entry point: PX4/MAVROS + krac_bt_runner.
# Legacy FSM (vtol_fsm_P) and its launch-time mission_loader node are always
# disabled here so they never run alongside the BT — mission_loader is instead
# invoked internally by krac_bt_runner (MissionContext::runExternalMissionLoader).
# Use scripts/run_sitl.sh for the legacy FSM flow.

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_DIR/ros2_ws"

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 daemon stop >/dev/null 2>&1 || true
ROS_LOG_BASE="${ROS_LOG_DIR:-/tmp/krac_ros_logs}"
mkdir -p "$ROS_LOG_BASE"
RUN_LOG_DIR="${RUN_LOG_DIR:-$ROS_LOG_BASE/bt_run_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_LOG_DIR"
export ROS_LOG_DIR="$RUN_LOG_DIR/ros_logs"
mkdir -p "$ROS_LOG_DIR"

VEHICLE_MODEL="${VEHICLE_MODEL:-gz_standard_vtol}"
GZ_MODEL_NAME="${GZ_MODEL_NAME:-standard_vtol_0}"
WORLD_NAME="${WORLD_NAME:-default}"
APPLY_PX4_ASSETS="${APPLY_PX4_ASSETS:-true}"
START_AUTO_SPAWN="${START_AUTO_SPAWN:-true}"
START_IMAGE_REPUBLISHER="${START_IMAGE_REPUBLISHER:-true}"
START_VISION_BT="${START_VISION_BT:-true}"
START_RQT_GRAPH="${START_RQT_GRAPH:-false}"
START_VISION_VIEW="${START_VISION_VIEW:-false}"
VISION_DEBUG_TOPIC="${VISION_DEBUG_TOPIC:-/vision/dbg_image}"
AUTO_SPAWN_MODE="${AUTO_SPAWN_MODE:-after_world}"
AUTO_SPAWN_MIN_ALT_M="${AUTO_SPAWN_MIN_ALT_M:-2.0}"
AUTO_SPAWN_DELAY_SEC="${AUTO_SPAWN_DELAY_SEC:-2}"

# auto_spawn.sh needs the REP's local (east, north) offset from home to place
# props correctly; it defaults to way3.plan's REP. When BT_XML_PATH selects the
# krac24 BT variant, switch those defaults to krac24.plan's REP offset instead
# (computed from krac24.plan's own plannedHomePosition) — override with
# AUTO_SPAWN_REP_E/AUTO_SPAWN_REP_N explicitly if a plan's REP location changes.
if [[ "${BT_XML_PATH:-}" == *krac24* ]]; then
  export AUTO_SPAWN_REP_E="${AUTO_SPAWN_REP_E:--26.03}"
  export AUTO_SPAWN_REP_N="${AUTO_SPAWN_REP_N:--31.41}"
fi

if [[ "${APPLY_PX4_ASSETS}" == "true" ]]; then
  echo "[BT] Applying PX4/Gazebo assets (camera, gripper, mission spawn models)..."
  (
    cd "$REPO_DIR"
    ./scripts/apply_px4_assets.sh
  ) >"$RUN_LOG_DIR/apply_px4_assets.log" 2>&1
fi

echo "[BT] Logs: ${RUN_LOG_DIR}"
echo "[BT] Starting gripper servo bridge for ${GZ_MODEL_NAME}..."
ros2 run ros_gz_bridge parameter_bridge \
  "/model/${GZ_MODEL_NAME}/servo_4@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/${GZ_MODEL_NAME}/servo_5@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/${GZ_MODEL_NAME}/servo_6@std_msgs/msg/Float64]gz.msgs.Double" \
  "/survivor_tray_rep/attach@std_msgs/msg/Empty]gz.msgs.Empty" \
  "/survivor_tray_rep/detach@std_msgs/msg/Empty]gz.msgs.Empty" \
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
  echo "[BT] Starting ${name}..."
  "$@" >"$RUN_LOG_DIR/${name// /_}.log" 2>&1 &
  SUPPORT_PIDS+=("$!")
}

start_optional_ros_gui() {
  local name="$1"
  shift
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "[BT] Skipping ${name}: DISPLAY is not set."
    return
  fi
  start_support_process "$name" "$@"
}

if [[ "${START_AUTO_SPAWN}" == "true" ]]; then
  (
    read_topic_value_once() {
      local topic="$1"
      local field="$2"
      local value=""
      set +e
      value="$(
        timeout 2 ros2 topic echo "$topic" --once 2>/dev/null \
          | awk -v field="${field}:" '$1 == field {print $2; exit}'
      )"
      local rc=$?
      set -e
      if [[ $rc -eq 0 && -n "$value" ]]; then
        printf '%s\n' "$value"
      fi
    }

    if [[ "${AUTO_SPAWN_MODE}" == "after_takeoff" ]]; then
      echo "[BT] Auto spawn waits until rel_alt >= ${AUTO_SPAWN_MIN_ALT_M}m..."
      for i in {1..180}; do
        ALT="$(read_topic_value_once /mavros/global_position/rel_alt data)"
        REACHED_SEQ="$(read_topic_value_once /mavros/mission/reached wp_seq)"
        if [[ -n "${ALT}" ]] && awk -v alt="${ALT}" -v min_alt="${AUTO_SPAWN_MIN_ALT_M}" 'BEGIN { exit !(alt >= min_alt) }'; then
          echo "[BT] rel_alt=${ALT}m; spawning mission objects."
          break
        fi
        if [[ -n "${REACHED_SEQ}" ]] && awk -v seq="${REACHED_SEQ}" 'BEGIN { exit !(seq >= 0) }'; then
          echo "[BT] mission reached seq=${REACHED_SEQ}; spawning mission objects."
          break
        fi
        if (( i % 10 == 0 )); then
          echo "[BT] Auto spawn still waiting... try=${i} rel_alt=${ALT:-n/a} reached_seq=${REACHED_SEQ:-n/a}"
        fi
        sleep 1
      done
    elif [[ "${AUTO_SPAWN_MODE}" == "after_world" ]]; then
      echo "[BT] Auto spawn waits for Gazebo create service in world '${WORLD_NAME}'..."
      for i in {1..90}; do
        if gz service -l 2>/dev/null | grep -q "/world/${WORLD_NAME}/create"; then
          echo "[BT] Gazebo create service is ready; spawning mission objects after ${AUTO_SPAWN_DELAY_SEC}s."
          sleep "${AUTO_SPAWN_DELAY_SEC}"
          break
        fi
        sleep 1
      done
    else
      echo "[BT] Auto spawn delay mode: sleeping ${AUTO_SPAWN_DELAY_SEC}s."
      sleep "${AUTO_SPAWN_DELAY_SEC}"
    fi
    exec "$REPO_DIR/scripts/auto_spawn.sh"
  ) >"$RUN_LOG_DIR/auto_spawn.log" 2>&1 &
  SUPPORT_PIDS+=("$!")
fi

if [[ "${START_IMAGE_REPUBLISHER}" == "true" ]]; then
  start_support_process "Gazebo image republisher" "$REPO_DIR/scripts/run_gz_image_republisher.sh"
fi

if [[ "${START_VISION_BT}" == "true" ]]; then
  start_support_process "BT vision stack" "$REPO_DIR/scripts/run_vision_bt.sh"
fi

if [[ "${START_RQT_GRAPH}" == "true" ]]; then
  start_optional_ros_gui "rqt graph" ros2 run rqt_graph rqt_graph
fi

if [[ "${START_VISION_VIEW}" == "true" ]]; then
  start_optional_ros_gui "vision debug viewer" ros2 run rqt_image_view rqt_image_view "${VISION_DEBUG_TOPIC}"
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

echo "[BT] Waiting for MAVROS connection before starting krac_bt_runner..."
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

if [[ "$CONNECTED" == "true" ]]; then
  # Without a real GCS, PX4's rcAndDataLinkCheck blocks arming ("No connection
  # to the GCS") whenever NAV_DLL_ACT > 0, since mavros (a companion computer,
  # not a GCS) never satisfies that check. Force it off so headless BT runs
  # (no QGroundControl) can still arm.
  echo "[BT] Disabling NAV_DLL_ACT so arming doesn't require a GCS connection..."
  ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
    "{force_set: true, param_id: 'NAV_DLL_ACT', value: {type: 2, integer_value: 0}}" \
    >"$RUN_LOG_DIR/nav_dll_act_disable.log" 2>&1 || true
fi

if [[ "$CONNECTED" != "true" ]]; then
  echo "[ERROR] MAVROS did not connect within 60 seconds."
  "$REPO_DIR/scripts/collect_bt_debug_logs.sh" "$RUN_LOG_DIR" || true
  exit 3
fi

echo "[BT] Starting krac_bt_runner..."
BT_LAUNCH_ARGS=(
  mission_upload_stub_success:="${MISSION_UPLOAD_STUB_SUCCESS:-false}"
  gripper_stub_success:="${GRIPPER_STUB_SUCCESS:-false}"
  print_bt_transitions:="${PRINT_BT_TRANSITIONS:-true}"
  enable_bt_viewer:="${ENABLE_BT_VIEWER:-false}"
  bt_viewer_direction:="${BT_VIEWER_DIRECTION:-Vertical}"
)
# BT_XML_PATH / BT_PARAMS_FILE: set to switch the mission source, e.g. to fly
# krac24.plan instead of the default way3.plan:
#   BT_XML_PATH=$(ros2 pkg prefix krac_control)/share/krac_control/bt/krac_mission_bt_krac24.xml \
#   BT_PARAMS_FILE=$(ros2 pkg prefix krac_control)/share/krac_control/config/krac_bt_params_krac24.yaml \
#   ./scripts/run_sitl_bt.sh
if [[ -n "${BT_XML_PATH:-}" ]]; then
  BT_LAUNCH_ARGS+=(bt_xml_path:="${BT_XML_PATH}")
fi
if [[ -n "${BT_PARAMS_FILE:-}" ]]; then
  BT_LAUNCH_ARGS+=(params_file:="${BT_PARAMS_FILE}")
fi
set +e
ros2 launch krac_control krac_bt_runner.launch.py \
  "${BT_LAUNCH_ARGS[@]}" \
  >"$RUN_LOG_DIR/krac_bt_runner.launch.log" 2>&1
BT_EXIT=$?
set -e

echo "[BT] krac_bt_runner exited with code ${BT_EXIT}"
"$REPO_DIR/scripts/collect_bt_debug_logs.sh" "$RUN_LOG_DIR" || true
exit "$BT_EXIT"
