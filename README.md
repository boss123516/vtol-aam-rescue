# Rescue Placeholder Mode

현재 구조 구간은 제어팀 알고리즘이 완성되기 전까지 임시 노드로 대체됩니다.

## 현재 시나리오

```text
HOME → REP
→ OFFBOARD 인계
→ 구조물 상공 (-26.03, -31.41, 3.0 m)
→ 안정 hover
→ 임시 구조 노드 실행
→ AUTO.LAND
→ 착륙 확인
→ 지상에서 5초 대기
→ SUCCESS
→ 기존 return mission 업로드
→ return mission의 VTOL_TAKEOFF
→ 기존 귀환·FW 천이·최종 착륙
```

따라서 전체 BT 시나리오는 중단되지 않습니다.

## 제어팀이 나중에 수정할 파일

```text
ros2_ws/src/krac_control/src/rescue_controller_placeholder.py
```

제어팀은 이 파일의 내부 동작만 다음과 같이 교체하면 됩니다.

```text
현재:
AUTO.LAND → 5초 대기 → SUCCESS

나중:
비전 검출 → 정렬 → 하강 → 파지 → 검증 → 상승 → 안정 hover → SUCCESS
```

토픽 인터페이스는 유지합니다.

```text
subscribe:
  /krac/rescue_module/enable
  std_msgs/msg/Bool

publish:
  /krac/rescue_module/ready
  std_msgs/msg/Bool

publish:
  /krac/rescue_module/result
  std_msgs/msg/String
```

성공:

```text
SUCCESS
```

실패:

```text
FAILURE:reason
```

## 실행

터미널 3개를 씁니다: (1) QGroundControl, (2) SITL+BT, (3) 그리퍼 수동 파지 조작.
전체 가이드(로그 위치·종료·환경변수 참고표·메모리 주의사항)는
`~/workspace/hzy/krac24_split_terminal_run_guide.txt` 에 있습니다.

### 사전 준비 (한 번만)

```bash
cd ~/workspace/hzy/vtol-aam-rescue-split/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash

# YOLO weight 확인
ls ~/workspace/hzy/vtol-aam-rescue-split/ros2_ws/src/krac_vision/weights/best.pt
```

### 터미널 1 — QGroundControl

```bash
~/QGroundControl.AppImage &
```

PX4/mavros 가 뜨기 **전에** 켜야 합니다. QGC 는 최초 vehicle connect 때만 미션
리스트를 요청하기 때문에, 나중에 붙이면 웨이포인트가 지도에 안 보일 수 있습니다
(QGC 정상 동작이며, Plan 탭을 열었다 닫으면 새로고침됩니다).

### 터미널 2 — SITL + BT

```bash
cd ~/workspace/hzy/vtol-aam-rescue-split
source ros2_ws/install/setup.bash

export ENABLE_BT_VIEWER=true
export START_RQT_GRAPH=true
export START_VISION_VIEW=true
export VISION_DEBUG_TOPIC=/vision/dbg_image
export START_VISION_BT=true
export VISION_MODEL_PATH="$PWD/ros2_ws/src/krac_vision/weights/best.pt"
export VISION_OBB_CONFIDENCE=0.45
export VISION_TRACKING_HOLD_SEC=2.5
export VISION_INFERENCE_INTERVAL_SEC=0.0
export BT_XML_PATH="$(ros2 pkg prefix krac_control)/share/krac_control/bt/krac_mission_bt_krac24_split.xml"
export BT_PARAMS_FILE="$(ros2 pkg prefix krac_control)/share/krac_control/config/krac_bt_params_krac24.yaml"

./scripts/run_sitl_bt.sh
```

뜨는 창: Gazebo(시뮬/GUI), BT viewer, rqt_graph, rqt_image_view(`/vision/dbg_image`).

주요 토글:

```bash
START_RESCUE_PLACEHOLDER=false ./scripts/run_sitl_bt.sh   # placeholder 자체를 끔
MANUAL_GRASP=false ./scripts/run_sitl_bt.sh               # 수동 파지 생략(착륙 후 바로 상승)
GIMBAL_SELFTEST=true ./scripts/run_sitl_bt.sh             # REP 도착 시 짐벌 동서남북 시연
```

### 터미널 3 — 그리퍼 수동 파지 조작

REP 착륙 후 `MANUAL_GRASP` 단계에서 사용자 입력을 기다립니다. 키를 raw 로 읽어야
해서 TTY 가 필요하며, `run_sitl_bt.sh` 안에서는 띄울 수 없습니다.

```bash
cd ~/workspace/hzy/vtol-aam-rescue-split
source /opt/ros/humble/setup.bash    # 새 터미널이라 이거 빼먹으면 rclpy import 부터 실패
source ros2_ws/install/setup.bash

python3 scripts/gripper_teleop.py
```

| 키 | 동작 |
| --- | --- |
| `a` / `d` | 집게 회전 ∓5° (servo_4, 360° 전 범위) |
| `A` / `D` | 집게 회전 ∓20° |
| `w` / `s` | 손가락 열기/닫기 ∓5° (servo_5,6 동시) |
| `W` / `S` | 손가락 열기/닫기 ∓20° |
| `o` | 활짝 열기 (-60°, 틈 211mm) |
| `g` | 파지 (-38°, 박스 짧은변 170mm 조임) |
| `r` | 회전 0° 복귀 |
| `z` | 짧은변 자동정렬 재적용 |
| **Enter** | 확실히 잡았음 → 상승·복귀 |
| **ESC** | 못 잡았음/떨어뜨림 → 이 구간만 재시작 (집게 열고 핸드오버 고도 복귀 후 재하강). 상승 중에도 받습니다 |
| `q` | 종료 |

- 키가 먹으려면 이 터미널에 포커스가 있어야 합니다. 착륙 전에 눌러도 무시됩니다.
- 카메라(body x=+0.2)와 그리퍼(x=0)가 0.2m 떨어져 있어 직하로는 집게가 안 보입니다.
  그래서 `MANUAL_GRASP` 진입 시 짐벌이 자동으로 yaw=180°/직하에서 59° 로 돌아
  손끝을 비춥니다 — `rqt_image_view` 로 파지를 확인하세요.
- 서보 토픽(`/model/<model>/servo_4,5,6`)은 rescue 컨트롤러만 발행합니다. teleop 은
  `/krac/gripper/cmd`(델타/프리셋)와 `/krac/manual_grasp/{confirm,retry}` 만 보냅니다.
  둘이 같은 토픽에 쏘면 서로 덮어써서 집게가 떨립니다.

> BT 의 `result=SUCCESS` 는 **실제 픽업을 검증하지 않습니다.** 대상이 딸려 올라왔는지는
> `gz topic -e -t /world/default/pose/info -n 1` 로 `rescue_box` 의 z 를 확인하세요.

## 로그

```text
/tmp/krac_ros_logs/bt_run_<timestamp>/Rescue_placeholder.log
```

확인:

```bash
LOG_DIR=$(ls -td /tmp/krac_ros_logs/bt_run_* | head -1)

grep -RniE \
'placeholder|AUTO.LAND|Landing confirmed|result=SUCCESS|External rescue' \
"$LOG_DIR"
```

정상 순서:

```text
enable=true received
ready=true
Requested AUTO.LAND
Landing confirmed
Waiting 5.0 s
result=SUCCESS
External rescue handback complete: SUCCESS
UploadReturnLeg
AUTO.MISSION
```

## 관련 BT 파일

```text
ros2_ws/src/krac_control/bt/krac_mission_bt_krac24_split.xml
```

BT에서는 구조물 상공 도착과 외부 모듈 결과 대기만 수행합니다. 실제 임시 착륙 동작은 `rescue_controller_placeholder.py`에 있습니다.
