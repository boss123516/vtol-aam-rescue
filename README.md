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

기본값으로 placeholder 노드가 자동 실행됩니다.

```bash
cd ~/workspace/hzy/vtol-aam-rescue-split
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash

./scripts/run_sitl_bt.sh
```

끄려면:

```bash
START_RESCUE_PLACEHOLDER=false ./scripts/run_sitl_bt.sh
```

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
