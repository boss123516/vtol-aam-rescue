#!/bin/bash
set -e

WORLD_NAME="default"
MODEL_ROOT="$HOME/vtol-aam-rescue/px4_assets/gz/models"

# way3.plan 기준:
# REP amsr_local_offset_ne_m = [N, E] = [35.0, 130.0]
# Gazebo spawn 좌표는 x=E, y=N 으로 사용
REP_X=130.0
REP_Y=35.0

HOME_X=0.0
HOME_Y=0.0

echo "=== [Auto Spawn] Spawn models at mission REP ==="
echo "WORLD_NAME=$WORLD_NAME"
echo "MODEL_ROOT=$MODEL_ROOT"
echo "REP position: x=$REP_X y=$REP_Y"

echo ">> Checking model files..."
ls "$MODEL_ROOT/v_marker/model.sdf" >/dev/null
ls "$MODEL_ROOT/box/model.sdf" >/dev/null
ls "$MODEL_ROOT/victim/model.sdf" >/dev/null
ls "$MODEL_ROOT/survivor_tray/model.sdf" >/dev/null

echo ">> (1/4) Spawning V Marker at Home..."
gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"file://$MODEL_ROOT/v_marker/model.sdf\" name: \"v_marker\" pose: {position: {x: $HOME_X, y: $HOME_Y, z: 0.1}}"

sleep 1

echo ">> (2/4) Spawning rescue_box at REP..."
gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"file://$MODEL_ROOT/box/model.sdf\" name: \"rescue_box\" pose: {position: {x: $REP_X, y: $REP_Y, z: 0.1}}"

sleep 1

echo ">> (3/4) Spawning victim at REP..."
gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"file://$MODEL_ROOT/victim/model.sdf\" name: \"victim\" pose: {position: {x: $REP_X, y: $REP_Y, z: 0.25}}"

sleep 1

echo ">> (4/4) Spawning survivor_tray at REP..."
gz service -s /world/$WORLD_NAME/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"file://$MODEL_ROOT/survivor_tray/model.sdf\" name: \"survivor_tray_rep\" pose: {position: {x: $REP_X, y: $REP_Y, z: 0.0}}"

echo "=== [DONE] Auto spawn complete ==="
