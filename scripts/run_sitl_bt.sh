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
START_RESCUE_PLACEHOLDER="${START_RESCUE_PLACEHOLDER:-true}"
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
  "/model/${GZ_MODEL_NAME}/gimbal_yaw@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/${GZ_MODEL_NAME}/gimbal_pitch@std_msgs/msg/Float64]gz.msgs.Double" \
  "/survivor_tray_rep/attach@std_msgs/msg/Empty]gz.msgs.Empty" \
  "/survivor_tray_rep/detach@std_msgs/msg/Empty]gz.msgs.Empty" \
  "/survivor_tray_rep/gripper_state@std_msgs/msg/Bool[gz.msgs.Boolean" \
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

if [[ "${START_RESCUE_PLACEHOLDER}" == "true" ]]; then
  # GIMBAL_SELFTEST=true -> at REP, sweep the gimbal through 동서남북 once (demo
  # of the real gimbal motion) before the normal immediate-detect check. Does
  # not change the actual rescue/landing logic.
  # MANUAL_GRASP=false 로 끄면 예전처럼 착륙 후 바로 상승한다(수동 파지 생략).
  start_support_process "Rescue placeholder" \
    ros2 run krac_control rescue_controller_placeholder.py \
    --ros-args \
    -p selftest_sweep_enable:="${GIMBAL_SELFTEST:-false}" \
    -p manual_grasp_enable:="${MANUAL_GRASP:-true}" \
    -p gz_model_name:="${GZ_MODEL_NAME}"
fi

if [[ "${MANUAL_GRASP:-true}" == "true" ]]; then
  echo "[BT] MANUAL_GRASP on: 착륙 후 그리퍼 수동 파지 대기. 별도 터미널에서"
  echo "[BT]   cd ${REPO_DIR} && source ros2_ws/install/setup.bash"
  echo "[BT]   python3 scripts/gripper_teleop.py"
  echo "[BT] 를 실행해 집게를 조작하고 Enter 로 파지를 확정하세요(TTY 필요)."
fi

# Gimbal relay: converts the rescue controller's /gimbal/angle_cmd (deg) into
# the Gazebo joint-position-controller topics bridged above, and echoes
# /gimbal/attitude. GZ_MODEL_NAME is exported so it picks the right model.
if [[ "${START_GIMBAL_RELAY:-true}" == "true" ]]; then
  export GZ_MODEL_NAME
  start_support_process "Gimbal relay" \
    python3 "$REPO_DIR/scripts/gimbal_relay.py" \
    --ros-args \
    -p gz_model_name:="${GZ_MODEL_NAME}" \
    -p yaw_sign:="${GIMBAL_YAW_SIGN:-1.0}" \
    -p pitch_sign:="${GIMBAL_PITCH_SIGN:--1.0}"
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

  # SITL-only PRE-ARM relaxation. On the stock gz standard_vtol there is no
  # simulated airspeed sensor, so the airspeed selector reports "module down" and
  # commander refuses to arm ("Resolve system health failures first"). We skip
  # ONLY that airspeed arming gate; the FW still uses airspeed if present. Do NOT
  # set FW_USE_AIRSPD=0 here (leaves the FW with no airspeed feedback -> climbs
  # away and quad-chutes after transition, observed 2026-07-13).
  #
  # IMPORTANT (2026-07-14): we no longer force EKF2_MAG_CHECK=0 / COM_ARM_MAG_STR=0.
  # The magnetometer is actually healthy here (~48 uT, matches the world field);
  # the only real problem is that on a fresh boot EKF2 needs ~15-40 s of GPS+mag
  # fusion before its yaw/heading estimate becomes valid. Forcing those two params
  # off dropped the heading arming gate, so the BT armed *during* the invalid-yaw
  # window and the multicopter flipped over on takeoff (bad yaw -> wrong tilt
  # command -> nose into the ground; user report "바로 천이 안 되고 땅에 박음").
  # Instead we keep PX4's own heading gate active and WAIT below until the
  # estimator stops reporting an invalid heading, so arming happens with a valid
  # yaw and the MC takeoff + VTOL transition proceed normally.
  # Set BT_SITL_RELAX_PREARM=false to skip the airspeed CBRK too (e.g. real HW).
  if [[ "${BT_SITL_RELAX_PREARM:-true}" == "true" ]]; then
    echo "[BT] Relaxing SITL pre-arm airspeed gate (no simulated airspeed sensor)..."
    set_int_param() {
      ros2 service call /mavros/param/set mavros_msgs/srv/ParamSetV2 \
        "{force_set: true, param_id: '$1', value: {type: 2, integer_value: $2}}" \
        >>"$RUN_LOG_DIR/sitl_relax_prearm.log" 2>&1 || true
    }
    set_int_param CBRK_AIRSPD_CHK 162128  # skip the airspeed PREFLIGHT (arming)
                                          # check only; FW still USES airspeed.
  fi

  # Wait for EKF2 heading/yaw to converge before the BT is allowed to arm.
  # "heading estimate invalid" is a transient fresh-boot condition; arming into
  # it flips the MC on takeoff. We watch the PX4 log and proceed once no new
  # "heading estimate invalid" line has appeared for ~10 s (min 15 s settle,
  # max 75 s fallback). BT_SITL_WAIT_HEADING=false skips the wait.
  if [[ "${BT_SITL_WAIT_HEADING:-true}" == "true" ]]; then
    echo "[BT] Waiting for EKF heading estimate to become valid before arming..."
    PX4_LOG="$RUN_LOG_DIR/sitl_vtol.launch.log"
    prev_cnt=-1; stable=0
    for i in $(seq 1 75); do
      cnt=$(grep -ac "heading estimate invalid" "$PX4_LOG" 2>/dev/null || true)
      cnt=${cnt:-0}
      if [[ "$cnt" == "$prev_cnt" ]]; then stable=$((stable+1)); else stable=0; fi
      prev_cnt=$cnt
      if (( i >= 15 && stable >= 10 )); then
        echo "[BT] EKF heading settled (no new invalid-heading for ~10s; count=$cnt, t=${i}s)."
        break
      fi
      sleep 1
    done
  fi
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
