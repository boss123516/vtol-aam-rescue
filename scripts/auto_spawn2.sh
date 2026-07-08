#!/bin/bash
set -e

# krac24's latest auto_spawn2.sh (commit 9f525f45, offboard_test_added):
# hardcoded local ENU target for the vtol_offboard precision-landing/gripper
# test (NOT the GPS-derived REP used by the full BT patrol mission/way3.plan).
WORLD_NAME="${WORLD_NAME:-default}"
MODEL_ROOT="$HOME/vtol-aam-rescue/px4_assets/gz/models"

RX=-22.48
RY=-31.59

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

echo "=== [Auto Spawn 2] vtol_offboard test target (krac24 auto_spawn2.sh) ==="
echo "WORLD_NAME=$WORLD_NAME"
echo "MODEL_ROOT=$MODEL_ROOT"
echo "Rescue target position: x=$RX y=$RY"

ls "$MODEL_ROOT/v_marker/model.sdf" >/dev/null
ls "$MODEL_ROOT/box/model.sdf" >/dev/null
ls "$MODEL_ROOT/land_marker/model.sdf" >/dev/null

spawn_model "V marker at home" "$MODEL_ROOT/v_marker/model.sdf" "v_marker" "$HOME_X" "$HOME_Y" "0.1"

sleep 1

spawn_model "rescue box" "$MODEL_ROOT/box/model.sdf" "rescue_box" "$RX" "$RY" "0.5"

sleep 1

spawn_model "rescue background" "$MODEL_ROOT/land_marker/model.sdf" "rescue_background" "$RX" "$RY" "0.0"

echo "=== [DONE] Auto spawn 2 complete ==="
