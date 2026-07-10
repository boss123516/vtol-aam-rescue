# KRAC Control Team Complete

ROS 2 Humble, PX4 SITL, Gazebo, MAVROS, BehaviorTree 기반의 KRAC VTOL 구조 미션 통합 실행 프로젝트입니다.

## 실행 구성

`run.sh` 실행 시 다음 구성요소를 함께 실행합니다.

- PX4 SITL
- Gazebo
- MAVROS
- KRAC BehaviorTree Runner
- BT Viewer
- Vision Node
- Vision Camera View
- Rescue Offboard Controller
- BT 상태 전환 로그

## 프로젝트 위치

```bash
~/workspace/hzy/krac-control-team-complete
```

## 최초 설치

저장소를 처음 받은 환경에서 아래 명령을 실행합니다.

```bash
mkdir -p ~/workspace/hzy
cd ~/workspace/hzy

git clone https://github.com/YOUR_GITHUB_ID/krac-control-team-complete.git
cd krac-control-team-complete

chmod +x setup.sh
chmod +x verify.sh
chmod +x run.sh
find . -name "*.sh" -exec chmod +x {} \;

./setup.sh
./verify.sh
```

ROS 2 환경이 자동으로 적용되지 않는 경우:

```bash
source /opt/ros/humble/setup.bash
```

프로젝트에 `install/setup.bash`가 존재하는 경우:

```bash
source ~/workspace/hzy/krac-control-team-complete/install/setup.bash
```

## 전체 실행 가이드

### 1. QGroundControl 실행

첫 번째 터미널에서 실행합니다.

```bash
~/QGroundControl.AppImage &
```

### 2. 기존 프로세스 종료 후 전체 시스템 실행

두 번째 터미널에서 아래 명령 전체를 그대로 실행합니다.

```bash
cd ~/workspace/hzy/krac-control-team-complete

pkill -9 -f "krac_bt_runner" || true
pkill -9 -f "rescue_offboard_bt_controller" || true
pkill -9 -f "mavros_node" || true
pkill -9 -f "px4_sitl_default/bin/px4" || true
pkill -9 -f "gz sim" || true

sleep 3

ENABLE_BT_VIEWER=true \
START_VISION_VIEW=true \
PRINT_BT_TRANSITIONS=true \
./run.sh
```

위 명령은 기존에 남아 있는 BT, 구조 제어기, MAVROS, PX4 SITL, Gazebo 프로세스를 종료한 후 새 미션을 실행합니다.

환경 변수 기능:

```text
ENABLE_BT_VIEWER=true
BT Viewer 실행

START_VISION_VIEW=true
카메라 및 비전 영상 창 실행

PRINT_BT_TRANSITIONS=true
BT 노드 상태 전환 로그 출력
```

## 실행 확인

새 터미널에서 ROS 2 환경을 적용합니다.

```bash
source /opt/ros/humble/setup.bash

cd ~/workspace/hzy/krac-control-team-complete

if [ -f install/setup.bash ]; then
    source install/setup.bash
fi
```

### ROS 2 노드 확인

```bash
ros2 node list
```

### MAVROS 상태 확인

```bash
ros2 topic echo /mavros/state --once
```

미션 진행 중 정상 상태 예시:

```text
connected: true
armed: true
mode: AUTO.MISSION
```

### 업로드된 미션 확인

```bash
ros2 topic echo /mavros/mission/waypoints --once
```

### Waypoint 도달 상태 확인

```bash
ros2 topic echo /mavros/mission/reached
```

### 구조 모듈 준비 상태 확인

```bash
ros2 topic echo /krac/rescue_module/ready
```

### 구조 모듈 결과 확인

```bash
ros2 topic echo /krac/rescue_module/result
```

정상 완료 결과:

```text
data: SUCCESS
```

## 구조 모듈 인터페이스

BehaviorTree에서 구조 제어기로 전달:

```text
/krac/rescue_module/enable
std_msgs/msg/Bool
```

구조 제어기에서 BehaviorTree로 전달:

```text
/krac/rescue_module/ready
std_msgs/msg/Bool

/krac/rescue_module/result
std_msgs/msg/String
```

결과 문자열:

```text
SUCCESS
FAILURE
```

## 구조 제어기 단독 활성화 테스트

BT를 거치지 않고 구조 제어기만 활성화할 때:

```bash
ros2 topic pub --once \
  /krac/rescue_module/enable \
  std_msgs/msg/Bool \
  "{data: true}"
```

비활성화:

```bash
ros2 topic pub --once \
  /krac/rescue_module/enable \
  std_msgs/msg/Bool \
  "{data: false}"
```

## 카메라 영상이 보이지 않을 때

영상 토픽을 확인합니다.

```bash
ros2 topic list | grep -Ei 'camera|image'
```

카메라 또는 비전 노드를 확인합니다.

```bash
ros2 node list | grep -Ei 'camera|vision|image'
```

영상 발행 주기를 확인합니다.

```bash
ros2 topic hz /camera/image_raw
```

별도 영상 창을 직접 실행하려면:

```bash
source /opt/ros/humble/setup.bash
rqt_image_view
```

`rqt_image_view` 실행 후 실제 발행 중인 이미지 토픽을 선택합니다.

## 로그 확인

가장 최근 로그 디렉터리를 찾습니다.

```bash
LOG_DIR=$(ls -td /tmp/krac_ros_logs/control_team_* 2>/dev/null | head -1)
echo "$LOG_DIR"
```

전체 로그 파일:

```bash
ls -al "$LOG_DIR"
```

BT 로그:

```bash
tail -f "$LOG_DIR/krac_bt_runner.launch.log"
```

PX4 SITL 로그:

```bash
tail -f "$LOG_DIR/sitl_vtol.launch.log"
```

구조 제어기 로그:

```bash
tail -f "$LOG_DIR/rescue_offboard_bt_controller.log"
```

오류 검색:

```bash
grep -RniE \
  "error|failed|failure|exception|abort|disarm|timeout|denied" \
  "$LOG_DIR"
```

Arm, Disarm, Failsafe 관련 검색:

```bash
grep -RniE \
  "arm|armed|disarm|preflight|failsafe|mode" \
  "$LOG_DIR"
```

BT 진행 상태 검색:

```bash
grep -RniE \
  "Mission|ArmVehicle|AUTO.MISSION|Waypoint|ExecuteExternalRescueModule|SUCCESS|FAILURE" \
  "$LOG_DIR"
```

## 강제 종료

전체 실행이 비정상 종료되었거나 프로세스가 남았을 때:

```bash
pkill -9 -f "krac_bt_runner" || true
pkill -9 -f "rescue_offboard_bt_controller" || true
pkill -9 -f "mavros_node" || true
pkill -9 -f "px4_sitl_default/bin/px4" || true
pkill -9 -f "gz sim" || true
```

## 코드 수정 후 Git 반영

프로젝트 폴더에서 변경사항을 확인합니다.

```bash
cd ~/workspace/hzy/krac-control-team-complete

git status
git diff
```

변경사항을 저장하고 Push합니다.

```bash
git add .
git commit -m "Update execution guide and mission runtime configuration"
git push origin main
```

다른 사용자가 최신 코드를 받을 때:

```bash
cd ~/workspace/hzy/krac-control-team-complete
git pull --rebase origin main
```
