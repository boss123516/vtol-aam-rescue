#!/bin/bash
set -e

WORLD_NAME="${WORLD_NAME:-default}"
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODEL_ROOT="${AUTO_SPAWN_MODEL_ROOT:-$REPO_DIR/px4_assets/gz/models}"

# REP local offset relative to plannedHomePosition, in Gazebo x=E, y=N.
# Defaults are way3.plan's REP_DIRECT offset (unchanged default behavior).
# krac24.plan's REP sits at a different local offset (east=-26.03, north=-31.41,
# computed from krac24.plan's own plannedHomePosition) — run_sitl_bt.sh exports
# AUTO_SPAWN_REP_E/AUTO_SPAWN_REP_N automatically when BT_XML_PATH selects the
# krac24 BT variant, so this script doesn't need to know which plan is active.
REP_X="${AUTO_SPAWN_REP_E:-130.0}"
REP_Y="${AUTO_SPAWN_REP_N:-35.0}"

HOME_X=0.0
HOME_Y=0.0

log() {
  printf '[%(%Y-%m-%d %H:%M:%S %Z)T] %s\n' -1 "$*"
}

spawn_model() {
  local label="$1"
  local sdf_path="$2"
  local name="$3"
  local x="$4"
  local y="$5"
  local z="$6"

  log ">> Spawning ${label}: name=${name} x=${x} y=${y} z=${z}"
  gz service -s "/world/${WORLD_NAME}/create" \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"file://${sdf_path}\" name: \"${name}\" pose: {position: {x: ${x}, y: ${y}, z: ${z}}}"
}

echo "=== [Auto Spawn] Spawn models at mission REP ==="
echo "WORLD_NAME=$WORLD_NAME"
echo "MODEL_ROOT=$MODEL_ROOT"
echo "REP position: x=$REP_X y=$REP_Y"

echo ">> Checking model files..."
ls "$MODEL_ROOT/v_marker/model.sdf" >/dev/null
ls "$MODEL_ROOT/land_marker/model.sdf" >/dev/null
ls "$MODEL_ROOT/box/model.sdf" >/dev/null
ls "$MODEL_ROOT/victim/model.sdf" >/dev/null

echo ">> Spawn order: after takeoff, marker targets first, rescue props at the REP point."
# v_marker (fiducial/vision target) marks the vertiport/home (start+destination);
# land_marker (red-cross rescue symbol) marks the REP (victim rescue site) —
# these were previously swapped.
spawn_model "home/vertiport vision marker" "$MODEL_ROOT/v_marker/model.sdf" "v_marker_home" "$HOME_X" "$HOME_Y" "0.02"

sleep 1

# --- REP 바닥 적층 ---
#   지면        z=0
#   마커판       z=0.020  (1.5x1.5x0.01 -> 0.015~0.025)  land_marker_rep
#   rescue_box  z=0.030 에서 떨궈 마커판 윗면(0.025)에 안착
#
# 마커판은 한 장이고 텍스처는 단색 초록(rescue_background.png)이다. 십자(land_marker.png)를
# 합성해 깔아봤더니 인식이 더 나빠져서 뺐다 - 아래 61줄의 원작자 주석과 같은 결론.
# 십자를 다시 넣더라도 슬래브를 얹지 말고 land_marker 텍스처에 합성할 것: 크기가 다른 두
# 판은 턱을 만들어 기체가 기울어 앉고, 박스가 아래 판 윗면에 앉는 이상 그 위 어떤 판도
# 박스를 10mm 파고든다(피할 z 가 없다).
# 판을 지면에서 0.02 로 띄운 건 면이 정확히 맞닿으면 z-fighting 이 나서다.
#
# 이름은 월드에서 유일해야 한다. 이미 쓰인 이름으로 spawn 하면 gz create 서비스가
# data:true 를 돌려주면서도 엔티티를 조용히 안 만든다 - 에러 로그도 안 남아서
# 성공한 것처럼 보인다. (rescue_background 를 rescue_box 라는 이름으로 띄우려다
# 이걸로 한참 헤맸다.)
spawn_model "rescue_background+cross at REP" "$MODEL_ROOT/land_marker/model.sdf" "land_marker_rep" "$REP_X" "$REP_Y" "0.02"

sleep 1

# 박스 메시의 원점은 바닥중심(z 범위가 0 에서 시작)이다. 판 속에서 spawn 하면 물리엔진이
# 관통을 푸느라 튕겨나가므로, 마커판 윗면보다 5mm 위에서 떨궈 자연스럽게 안착시킨다.
# (예전 z=0.043 은 사라진 survivor_tray 의 바닥면 높이였다. 띄워두면 안 되는 이유는
# 과거에 겪은 그대로다: 접지 없이 떠 있는 물체가 정밀하강/그리퍼 동작 경로에서 실제
# 물리 장애물이 돼 기체가 걸린다.)
spawn_model "rescue_box at REP" "$MODEL_ROOT/box/model.sdf" "rescue_box" "$REP_X" "$REP_Y" "0.030"


# # DISABLED: wooden dummy/victim is not available yet
# DISABLED: wooden dummy/victim is not available yet
# spawn_model "victim at REP" "$MODEL_ROOT/victim/model.sdf" "victim" "$REP_X" "$REP_Y" "0.2"

# DISABLED 2026-07-16: survivor_tray 제거. 파지 대상은 rescue_box 하나이고 트레이는
# 그 밑에 깔려 있을 뿐이라 없앤다. standard_vtol 패치의 DetachableJoint 플러그인이
# child_model 로 survivor_tray_rep 을 잡고 있지만, 지금 돌아가는 krac24_split BT 는
# 그 attach/detach 를 쓰지 않는다(RescuePickupModule -> ExecuteExternalRescueModule ->
# rescue_controller_placeholder.py 의 MANUAL_GRASP 가 집게로 직접 물어서 집는다).
# 플러그인도 suppress_child_warning=true 라 child 가 없어도 조용하다. 트레이를 되살릴
# 땐 이 줄과 아래 detach 줄을 같이 되돌릴 것.
# spawn_model "survivor_tray at REP" "$MODEL_ROOT/survivor_tray/model.sdf" "survivor_tray_rep" "$REP_X" "$REP_Y" "0.0"
# log ">> Ensuring survivor_tray_rep starts detached from the gripper"
# gz topic -t "/survivor_tray_rep/detach" -m gz.msgs.Empty -p "unused: true" || true

echo "=== [DONE] Auto spawn complete ==="
