#!/bin/bash

# ==========================================
# 1. GPS 좌표 설정 (YAML 파일 값 그대로 입력하세요)
# ==========================================

# [WP1] 이륙 위치 (Home) -> 기준점 (0,0)이 됩니다.
HOME_LAT=35.06898
HOME_LON=128.0863

# [Rescue] 바구니 위치 (구조 지점)
RESCUE_LAT=35.06945
RESCUE_LON=128.0864

# [Victim] 조난자 위치 (구조 지점과 동일하게 설정됨)
VICTIM_LAT=35.06945
VICTIM_LON=128.0864

# [Drop] 하기 위치
DROP_LAT=35.06883
DROP_LON=128.0858

# ==========================================
# 2. 환경 설정
# ==========================================
WORLD_NAME="default"  # 실행 중인 월드 이름 (변경 필요 시 수정, 예: 'vtol_world')
MODEL_ROOT="$HOME/.gz/fuel/models"

echo "=== [Auto Spawn] GPS 좌표를 미터(m)로 변환하여 스폰을 시작합니다 ==="

# ---------------------------------------------------------
# 3. 파이썬을 이용한 좌표 변환
# ---------------------------------------------------------
# 수정됨: 출력값 6개를 모두 받도록 변수 추가 (VX VY)
read RX RY VX VY DX DY <<< $(python3 -c "
import math

# 입력 변수 받기
h_lat = $HOME_LAT
h_lon = $HOME_LON
r_lat = $RESCUE_LAT
r_lon = $RESCUE_LON
v_lat = $VICTIM_LAT
v_lon = $VICTIM_LON
d_lat = $DROP_LAT
d_lon = $DROP_LON

R = 6371000.0 # 지구 반지름

def get_xy(lat, lon):
    dLat = math.radians(lat - h_lat)
    dLon = math.radians(lon - h_lon)
    lat1 = math.radians(h_lat)
    lat2 = math.radians(lat)
    
    # 평면 근사 계산 (Haversine보다 간단하며 단거리에서 충분)
    x = dLon * math.cos((lat1 + lat2) / 2.0) * R
    y = dLat * R
    return x, y

rx, ry = get_xy(r_lat, r_lon)
vx, vy = get_xy(v_lat, v_lon)
dx, dy = get_xy(d_lat, d_lon)

# 순서대로 6개 출력
print(f'{rx:.2f} {ry:.2f} {vx:.2f} {vy:.2f} {dx:.2f} {dy:.2f}')
")

echo ">> 변환 결과:"
echo "   - Home   (WP1) : 0.0, 0.0"
echo "   - Rescue (Box) : X=$RX, Y=$RY"
echo "   - Victim (Man) : X=$VX, Y=$VY"
echo "   - Drop   (Land): X=$DX, Y=$DY"
echo "---------------------------------------------------------"

# ------------------------------------------
# 4. 모델 스폰 실행
# ------------------------------------------

# (1) V Marker (Home)
echo ">> (1/4) Spawning V Marker at Home (0,0)..."
gz service -s /world/$WORLD_NAME/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--req "sdf_filename: \"file://$MODEL_ROOT/v_marker/model.sdf\" name: \"v_marker\" pose: {position: {x: 0.0, y: 0.0, z: 0.1}}"

sleep 1

# (2) Rescue Box (구조 바구니 등)
echo ">> (2/4) Spawning Box at Rescue Point..."
gz service -s /world/$WORLD_NAME/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--req "sdf_filename: \"file://$MODEL_ROOT/box/model.sdf\" name: \"rescue_box\" pose: {position: {x: $RX, y: $RY, z: 0.1}}"

sleep 3

# (3) Victim (조난자)
# 주의: Box와 좌표가 같다면 겹치지 않게 Z축을 살짝 올리거나(0.5) 위치를 미세 조정해야 물리 충돌이 안 일어납니다.
echo ">> (3/4) Spawning Victim at Rescue Point..."
gz service -s /world/$WORLD_NAME/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--req "sdf_filename: \"file://$MODEL_ROOT/victim/model.sdf\" name: \"victim\" pose: {position: {x: $VX, y: $VY, z: 0.2}}"

sleep 3

# (4) Land Marker (하기 지점)
echo ">> (4/4) Spawning Land Marker at Drop Point..."
gz service -s /world/$WORLD_NAME/create \
--reqtype gz.msgs.EntityFactory \
--reptype gz.msgs.Boolean \
--req "sdf_filename: \"file://$MODEL_ROOT/land_marker/model.sdf\" name: \"drop_marker\" pose: {position: {x: $DX, y: $DY, z: 0.1}}"

echo "=== [완료] 스폰 작업이 끝났습니다. ==="
