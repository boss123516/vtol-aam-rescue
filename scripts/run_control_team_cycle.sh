#!/usr/bin/env bash
set -euo pipefail

# Standalone control-team short-cycle launcher.
#   HOME 이륙 -> 구조지점 직행(BT/OFFBOARD) -> 외부 제어팀 구조 모듈(cpp)
#   -> 재이륙 완료 -> HOME 직행(MC/OFFBOARD) -> 최종 착륙
#
# ROOT는 이 스크립트가 들어있는 저장소 루트로 자동 계산된다(자립 실행).
# 기체는 amsr_vtol(그리퍼+mono_cam 내장, cjfgus814123/amsr-vtol-dev 판)을 쓴다.

ROOT="${KRAC_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
WS="$ROOT/ros2_ws"
TEAM_EXEC="rescue_offboard_bt_controller"
TEAM_PARAMS="$WS/src/krac_control/config/rescue_offboard_bt_params.yaml"

set +u
source /opt/ros/humble/setup.bash
source "$WS/install/setup.bash"
set -u

[[ -f "$TEAM_PARAMS" ]] || { echo "ERROR: $TEAM_PARAMS missing"; exit 1; }

RUN_LOG_DIR="${RUN_LOG_DIR:-/tmp/krac_ros_logs/control_team_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$RUN_LOG_DIR"
export RUN_LOG_DIR

# ---------------------------------------------------------------------------
# Vehicle: amsr_vtol (gz_amsr_vtol / gz model name amsr_vtol_0).
# 카메라(mono_cam)와 그리퍼(servo_4/5/6)가 모델에 내장돼 있어 standard_vtol용
# 카메라 패치 주입은 필요 없다. apply_px4_assets.sh는 amsr_vtol 모델/에어프레임
# 배포와 CMake 등록만 담당한다.
# ---------------------------------------------------------------------------
export VEHICLE_MODEL="${VEHICLE_MODEL:-gz_amsr_vtol}"
export GZ_MODEL_NAME="${GZ_MODEL_NAME:-amsr_vtol_0}"
export GZ_CAMERA_TOPIC="${GZ_CAMERA_TOPIC:-/world/default/model/${GZ_MODEL_NAME}/link/camera_link/sensor/camera/image}"

export START_RESCUE_PLACEHOLDER=false
export START_VISION_BT="${START_VISION_BT:-true}"
export START_IMAGE_REPUBLISHER="${START_IMAGE_REPUBLISHER:-true}"
# 타겟을 놓쳤을 때 vision_tracker의 칼만필터가 오래 외삽(runaway)하며 큰 오차를
# 계속 내보내면 구조 노드가 엉뚱한 방향으로 표류한다. hold을 3 s로 짧게 잡아
# 상실 시 빠르게 is_detected=false → SEARCH 복귀(경계 속도)로 전환되게 한다.
export VISION_TRACKING_HOLD_SEC="${VISION_TRACKING_HOLD_SEC:-3.0}"
export START_GRIPPER_BRIDGE="${START_GRIPPER_BRIDGE:-true}"
export START_AUTO_SPAWN="${START_AUTO_SPAWN:-true}"
export AUTO_SPAWN_REP_E="${AUTO_SPAWN_REP_E:--26.03}"
export AUTO_SPAWN_REP_N="${AUTO_SPAWN_REP_N:--31.41}"

pkill -f "rescue_offboard_bt_controller" 2>/dev/null || true
ros2 run krac_control "$TEAM_EXEC" --ros-args --params-file "$TEAM_PARAMS" \
  >"$RUN_LOG_DIR/rescue_offboard_bt_controller.log" 2>&1 &
TEAM_PID=$!
cleanup_team() { kill "$TEAM_PID" 2>/dev/null || true; }
trap cleanup_team EXIT

export BT_XML_PATH="$WS/src/krac_control/bt/krac_control_team_cycle_bt.xml"
export BT_PARAMS_FILE="$WS/src/krac_control/config/krac_bt_params_control_team.yaml"

echo "[CONTROL TEAM] Standalone short-cycle BT (amsr_vtol)"
echo "[CONTROL TEAM] Vehicle:    $VEHICLE_MODEL ($GZ_MODEL_NAME)"
echo "[CONTROL TEAM] Controller: $TEAM_EXEC"
echo "[CONTROL TEAM] Params:     $TEAM_PARAMS"
echo "[CONTROL TEAM] Camera:     $GZ_CAMERA_TOPIC"
echo "[CONTROL TEAM] Logs:       $RUN_LOG_DIR"

"$ROOT/scripts/run_sitl_bt.sh"
