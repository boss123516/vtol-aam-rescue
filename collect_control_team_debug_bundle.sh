#!/usr/bin/env bash
set -euo pipefail

ROOT="${KRAC_ROOT:-$HOME/workspace/hzy/krac-control-team-complete}"
WS="$ROOT/ros2_ws"
LATEST_LOG="${1:-}"

if [[ -z "$LATEST_LOG" ]]; then
  LATEST_LOG="$(ls -td /tmp/krac_ros_logs/control_team_* 2>/dev/null | head -1 || true)"
fi

if [[ -z "$LATEST_LOG" || ! -d "$LATEST_LOG" ]]; then
  echo "ERROR: control-team log directory not found."
  echo "Expected: /tmp/krac_ros_logs/control_team_*"
  exit 1
fi

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_ROOT="$ROOT/debug_exports"
OUT_DIR="$OUT_ROOT/control_team_debug_$STAMP"
ARCHIVE="$OUT_ROOT/control_team_debug_$STAMP.tar.gz"

mkdir -p "$OUT_DIR/live" "$OUT_DIR/config" "$OUT_DIR/logs"
mkdir -p "$OUT_ROOT"

echo "[1/7] Copying latest run logs..."
cp -a "$LATEST_LOG"/. "$OUT_DIR/logs/" 2>/dev/null || true
printf '%s\n' "$LATEST_LOG" > "$OUT_DIR/source_log_directory.txt"

run_capture() {
  local name="$1"
  shift
  {
    echo "$ $*"
    echo
    timeout 8 "$@"
  } >"$OUT_DIR/live/$name.txt" 2>&1 || true
}

run_shell_capture() {
  local name="$1"
  shift
  {
    echo "$ $*"
    echo
    timeout 10 bash -lc "$*"
  } >"$OUT_DIR/live/$name.txt" 2>&1 || true
}

echo "[2/7] Loading ROS environment..."
set +u
source /opt/ros/humble/setup.bash 2>/dev/null || true
if [[ -f "$WS/install/setup.bash" ]]; then
  source "$WS/install/setup.bash" 2>/dev/null || true
fi
set -u

echo "[3/7] Capturing processes and ROS graph..."
run_shell_capture processes \
  'ps aux | grep -E "px4|gz sim|mavros|krac_bt_runner|rescue_offboard_bt_controller|rescue_controller_team|vision|image|qground|groot" | grep -v grep'

run_capture ros2_node_list ros2 node list
run_capture ros2_topic_list ros2 topic list
run_capture ros2_service_list ros2 service list

run_shell_capture ros2_package_prefixes \
  'for p in krac_interfaces krac_control krac_mission krac_utils mavros; do echo "===== $p ====="; ros2 pkg prefix "$p" 2>&1 || true; done'

echo "[4/7] Capturing MAVROS and BT runtime state..."
run_capture mavros_state ros2 topic echo /mavros/state --once
run_capture mavros_extended_state ros2 topic echo /mavros/extended_state --once
run_capture mavros_rel_alt ros2 topic echo /mavros/global_position/rel_alt --once
run_capture mavros_local_pose ros2 topic echo /mavros/local_position/pose --once
run_capture mavros_local_velocity ros2 topic echo /mavros/local_position/velocity_local --once
run_capture mavros_waypoints ros2 topic echo /mavros/mission/waypoints --once
run_capture mavros_mission_reached ros2 topic echo /mavros/mission/reached --once

run_capture rescue_enable ros2 topic echo /krac/rescue_module/enable --once
run_capture rescue_ready ros2 topic echo /krac/rescue_module/ready --once
run_capture rescue_result ros2 topic echo /krac/rescue_module/result --once
run_capture camera_target_error ros2 topic echo /camera/target_error --once
run_capture camera_set_target ros2 topic echo /camera/set_target --once

run_shell_capture topic_info \
  'for t in \
    /mavros/state \
    /mavros/mission/reached \
    /mavros/mission/waypoints \
    /krac/rescue_module/enable \
    /krac/rescue_module/ready \
    /krac/rescue_module/result \
    /camera/target_error \
    /camera/set_target \
    /mavros/setpoint_velocity/cmd_vel_unstamped; do \
      echo "===== $t ====="; ros2 topic info -v "$t" 2>&1 || true; echo; \
   done'

run_shell_capture bt_params \
  'for n in /krac_bt_runner /rescue_offboard_bt_controller /krac24_offboard_bt_adapter; do \
      echo "===== $n ====="; \
      ros2 param list "$n" 2>&1 || true; \
      ros2 param dump "$n" 2>&1 || true; \
      echo; \
   done'

echo "[5/7] Copying relevant source/config files..."
copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -f "$src" ]]; then
    cp -a "$src" "$OUT_DIR/config/$dst"
  fi
}

copy_if_exists "$ROOT/run.sh" "run.sh"
copy_if_exists "$ROOT/setup.sh" "setup.sh"
copy_if_exists "$ROOT/scripts/run_control_team_cycle.sh" "run_control_team_cycle.sh"
copy_if_exists "$ROOT/scripts/run_sitl_bt.sh" "run_sitl_bt.sh"
copy_if_exists "$ROOT/scripts/collect_bt_debug_logs.sh" "collect_bt_debug_logs.sh"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/bt/krac_control_team_cycle_bt.xml" "krac_control_team_cycle_bt.xml"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/config/krac_bt_params_control_team.yaml" "krac_bt_params_control_team.yaml"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/config/rescue_offboard_bt_params.yaml" "rescue_offboard_bt_params.yaml"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/src/rescue_offboard_bt_controller.cpp" "rescue_offboard_bt_controller.cpp"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/src/control_team_outbound.plan" "control_team_outbound.plan"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/src/control_team_return.plan" "control_team_return.plan"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/CMakeLists.txt" "CMakeLists.txt"
copy_if_exists "$ROOT/ros2_ws/src/krac_control/package.xml" "package.xml"

echo "[6/7] Creating quick diagnostic summary..."
{
  echo "KRAC control-team diagnostic bundle"
  echo "Generated: $(date --iso-8601=seconds)"
  echo "Project: $ROOT"
  echo "Source log: $LATEST_LOG"
  echo

  echo "===== KEY LOG ERRORS ====="
  grep -RniE \
    'FATAL|ERROR|WARN|failed|Failure|FAILURE|denied|reject|disarm|preflight|timeout|exception|exit code|WaitForWaypointReached|ArmVehicle|SetFlightMode|AUTO.MISSION|OFFBOARD' \
    "$OUT_DIR/logs" 2>/dev/null | tail -n 500 || true

  echo
  echo "===== MAVROS STATE ====="
  cat "$OUT_DIR/live/mavros_state.txt" 2>/dev/null || true

  echo
  echo "===== BT RUNNER LOG ====="
  cat "$OUT_DIR/logs/krac_bt_runner.launch.log" 2>/dev/null || true

  echo
  echo "===== SITL KEY LINES ====="
  grep -niE \
    'arm|disarm|takeoff|mission|AUTO.MISSION|preflight|denied|failsafe|land' \
    "$OUT_DIR/logs/sitl_vtol.launch.log" 2>/dev/null | tail -n 300 || true

  echo
  echo "===== RESCUE ADAPTER LOG ====="
  cat "$OUT_DIR/logs/rescue_offboard_bt_controller.log" 2>/dev/null || true
} > "$OUT_DIR/DIAGNOSTIC_SUMMARY.txt"

echo "[7/7] Packing archive..."
tar -czf "$ARCHIVE" -C "$OUT_ROOT" "$(basename "$OUT_DIR")"

echo
echo "[DEBUG BUNDLE] DONE"
echo "Folder:"
echo "  $OUT_DIR"
echo "Archive:"
echo "  $ARCHIVE"
echo
echo "Upload this archive for analysis:"
echo "  $ARCHIVE"
