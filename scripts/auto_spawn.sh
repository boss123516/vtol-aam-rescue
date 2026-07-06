#!/bin/bash
set -e

WORLD_NAME="${WORLD_NAME:-default}"
MODEL_ROOT="$HOME/vtol-aam-rescue/px4_assets/gz/models"

# way3.plan 기준:
# REP amsr_local_offset_ne_m = [N, E] = [35.0, 130.0]
# Gazebo spawn 좌표는 x=E, y=N 으로 사용
REP_X=130.0
REP_Y=35.0

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
ls "$MODEL_ROOT/survivor_tray/model.sdf" >/dev/null

echo ">> Spawn order: after takeoff, marker targets first, rescue props at the REP point."
spawn_model "home landing marker" "$MODEL_ROOT/land_marker/model.sdf" "land_marker_home" "$HOME_X" "$HOME_Y" "0.02"

sleep 1

spawn_model "REP visual target marker" "$MODEL_ROOT/v_marker/model.sdf" "v_marker_rep" "$REP_X" "$REP_Y" "0.35"

sleep 1

spawn_model "rescue_box at REP" "$MODEL_ROOT/box/model.sdf" "rescue_box" "$REP_X" "$REP_Y" "0.1"

sleep 1

spawn_model "victim at REP" "$MODEL_ROOT/victim/model.sdf" "victim" "$REP_X" "$REP_Y" "0.2"

sleep 1

spawn_model "survivor_tray at REP" "$MODEL_ROOT/survivor_tray/model.sdf" "survivor_tray_rep" "$REP_X" "$REP_Y" "0.0"

echo "=== [DONE] Auto spawn complete ==="
