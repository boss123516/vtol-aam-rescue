#!/usr/bin/env bash
set -euo pipefail
ROOT="${KRAC_ROOT:-$HOME/workspace/hzy/vtol-aam-rescue-split}"
WS="$ROOT/ros2_ws"
TEAM_NODE="$WS/src/krac_control/src/rescue_controller_team.py"
set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u
[[ -f "$TEAM_NODE" ]] || { echo "ERROR: $TEAM_NODE missing"; exit 1; }
RUN_LOG_DIR="${RUN_LOG_DIR:-/tmp/krac_ros_logs/control_team_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_LOG_DIR"
export RUN_LOG_DIR
pkill -f "rescue_controller_team.py" 2>/dev/null || true
python3 "$TEAM_NODE" >"$RUN_LOG_DIR/rescue_controller_team.log" 2>&1 &
TEAM_PID=$!
cleanup_team() { kill "$TEAM_PID" 2>/dev/null || true; }
trap cleanup_team EXIT
export START_RESCUE_PLACEHOLDER=false
export START_VISION_BT="${START_VISION_BT:-true}"
export START_IMAGE_REPUBLISHER="${START_IMAGE_REPUBLISHER:-true}"
export START_GRIPPER_BRIDGE="${START_GRIPPER_BRIDGE:-true}"
export START_AUTO_SPAWN="${START_AUTO_SPAWN:-true}"
export AUTO_SPAWN_REP_E="${AUTO_SPAWN_REP_E:--26.03}"
export AUTO_SPAWN_REP_N="${AUTO_SPAWN_REP_N:--31.41}"

export BT_XML_PATH="$WS/src/krac_control/bt/krac_control_team_cycle_bt.xml"
export BT_PARAMS_FILE="$WS/src/krac_control/config/krac_bt_params_control_team.yaml"
echo "[CONTROL TEAM] Standalone short-cycle BT"
echo "[CONTROL TEAM] Controller: $TEAM_NODE"
echo "[CONTROL TEAM] Logs: $RUN_LOG_DIR"
"$ROOT/scripts/run_sitl_bt.sh"
