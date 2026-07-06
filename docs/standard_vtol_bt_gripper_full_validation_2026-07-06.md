# Standard VTOL BT + Gripper Full Validation

작성일: 2026-07-06 16:01 KST

## 목적

PX4 `gz_standard_vtol` 모델을 기준으로 BT 전체 미션을 끝까지 수행하고, 새로 연결한 Gazebo gripper servo topic 기반 action이 실제 BT 경로에서 동작하는지 확인했다.

이번 검증은 AMSR 기체가 아니라 PX4 standard VTOL로 수행했다. 비전 입력은 기존 full mission 검증과 동일하게 simulated `/vision/target_error`를 사용했다.

## 실행 조건

- Vehicle: `gz_standard_vtol`
- BT runner: `krac_control/krac_bt_runner`
- Mission upload: real upload (`mission_upload_stub_success:=false`)
- Gripper: real Gazebo servo topic path (`gripper_stub_success:=false`)
- Vision target input: simulated `/vision/target_error`
- Precision lander: `krac_control precision_lander`
- Gripper bridge:
  - `/model/standard_vtol_0/servo_4`
  - `/model/standard_vtol_0/servo_5`
  - `/model/standard_vtol_0/servo_6`

## 실행 명령 요약

전체 검증은 다음 구성으로 실행했다.

```bash
ros2 run ros_gz_bridge parameter_bridge \
  "/model/standard_vtol_0/servo_4@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/standard_vtol_0/servo_5@std_msgs/msg/Float64]gz.msgs.Double" \
  "/model/standard_vtol_0/servo_6@std_msgs/msg/Float64]gz.msgs.Double"

ros2 run krac_control precision_lander

ros2 topic pub /vision/target_error krac_interfaces/msg/TargetError \
  "{is_detected: true, pixel_err_x: 0.0, pixel_err_y: 0.0, yaw_err_rad: 0.0}" -r 20

ros2 launch krac_mission sitl_vtol.launch.py \
  vehicle_model:=gz_standard_vtol \
  start_px4:=true \
  start_mavros:=true \
  start_fsm:=false \
  start_mission_loader:=false \
  start_logger:=false

ros2 launch krac_control krac_bt_runner.launch.py \
  mission_upload_stub_success:=false \
  gripper_stub_success:=false \
  sim_bypass:=true \
  print_bt_transitions:=true
```

## 전체 미션 결과

전체 BT mission은 성공으로 종료됐다.

확인 로그:

```text
[mission_loader]: Uploading 12 items from .plan file...
[mission_loader]: Mission Upload SUCCESS!
[krac_bt_runner]: BT finished with SUCCESS at tick 2592
PX4: Disarmed by landing
```

주요 단계 확인:

- MAVROS 연결 성공
- `way3.plan` real mission upload 성공
- `AUTO.MISSION` 진입
- arm 성공
- standard VTOL 이륙 및 mission 진행
- rescue 지점 도달 후 BT가 `OFFBOARD`로 개입
- MC transition 명령 수행
- `precision_lander` 기반 simulated target 정밀 하강 수행
- rescue pickup 구간에서 `CloseGripper` 실행
- 구조 후 waypoint resume 및 FW transition
- return leg waypoint 진행
- final landing 구간에서 `OFFBOARD` + MC + precision landing 수행
- `AUTO.LAND` 전환 후 착륙 및 disarm
- BT root `SUCCESS`

## Mission Upload 확인

로그 파일:

```text
/tmp/krac_ros_logs/python3_325711_1783320678699.log
```

핵심 로그:

```text
[mission_loader]: Loading mission plan: .../way3.plan
[mission_loader]: Old mission cleared.
[mission_loader]: Plan altitude check: first=10.0, last=0.0
[mission_loader]: Uploading 12 items from .plan file...
[mission_loader]: Mission Upload SUCCESS!
```

MAVROS mission list 기준으로 return leg altitude가 30m로 들어간 것도 확인했다.

```text
WP item #5  z: 30
WP item #6  z: 30
WP item #7  z: 30
WP item #8  z: 30
WP item #9  z: 30
WP item #10 z: 30
WP item #11 C: 85 z: 0
```

## Precision Lander 확인

로그 파일:

```text
/tmp/krac_ros_logs/precision_lander_323531_1783320656131.log
```

Rescue 구간:

```text
Lander 상태: ON
정밀 하강 중 ... 고도: 4.9m
...
정밀 하강 중 ... 고도: 0.7m
Lander 상태: OFF
```

Final landing 구간:

```text
Lander 상태: ON
정밀 하강 중 ... 고도: 31.1m
...
정밀 하강 중 ... 고도: 0.3m
Lander 상태: OFF
```

## Gripper Bridge 확인

로그 파일:

```text
/tmp/krac_ros_logs/parameter_bridge_323532_1783320656158.log
```

확인 로그:

```text
Creating ROS->GZ Bridge: [/model/standard_vtol_0/servo_4 ... -> ... gz.msgs.Double]
Creating ROS->GZ Bridge: [/model/standard_vtol_0/servo_5 ... -> ... gz.msgs.Double]
Creating ROS->GZ Bridge: [/model/standard_vtol_0/servo_6 ... -> ... gz.msgs.Double]
```

## Full Mission Gripper 확인

전체 미션 중 실제로 실행된 gripper action은 rescue pickup 구간의 `CloseGripper`다.

로그 파일:

```text
/tmp/krac_ros_logs/krac_bt_runner_325617_1783320676076.log
```

핵심 로그:

```text
[krac_bt_runner]: Closing gripper via /model/standard_vtol_0/servo_5=0.50, /model/standard_vtol_0/servo_6=0.50
```

즉, 기존 stub가 아니라 BT action이 실제 ROS topic으로 `servo_5`, `servo_6` command를 publish하는 경로가 full mission 안에서 실행됐다.

## Open/Close Gripper Smoke Test

현재 MissionSequence에서는 drop branch가 제외되어 있어 full mission 경로에서 `OpenGripper`는 호출되지 않는다. 그래서 별도 최소 BT XML로 `OpenGripper`, `CloseGripper` action 자체를 각각 검증했다.

테스트 파일:

```text
ros2_ws/src/krac_control/bt/open_gripper_smoke.xml
ros2_ws/src/krac_control/bt/close_gripper_smoke.xml
```

OpenGripper 결과:

```text
[krac_bt_runner]: Opening gripper via /model/standard_vtol_0/servo_5=-1.20, /model/standard_vtol_0/servo_6=-1.20
BT finished with SUCCESS at tick 11

/model/standard_vtol_0/servo_5: data: -1.2
/model/standard_vtol_0/servo_6: data: -1.2
```

CloseGripper 결과:

```text
[krac_bt_runner]: Closing gripper via /model/standard_vtol_0/servo_5=0.50, /model/standard_vtol_0/servo_6=0.50
BT finished with SUCCESS at tick 11

/model/standard_vtol_0/servo_5: data: 0.5
/model/standard_vtol_0/servo_6: data: 0.5
```

## 결론

이번 검증 기준으로 다음은 통과했다.

- PX4 `gz_standard_vtol` 기반 전체 BT mission 자동 완주
- real mission upload
- `precision_lander` 기반 rescue/final precision descent
- full mission 중 `CloseGripper` 실제 Gazebo servo topic command 실행
- 별도 BT smoke 기준 `OpenGripper`/`CloseGripper` topic publish 동작
- 최종 landing 및 PX4 disarm

## 남은 제약

- 비전은 real YOLO가 아니라 simulated `/vision/target_error` 기준이다.
- 현재 `MissionSequence`에서는 drop branch가 제외되어 있어 full mission 중 `OpenGripper`는 호출되지 않는다.
- `OpenGripper`는 별도 smoke test로 action/topic publish만 검증했다.
- drop waypoint/seq가 확정되면 `CruiseToDrop`, `DropOperation`, `ResumeAfterDrop`을 MissionSequence에 다시 연결해야 한다.

## 최종 요약

`gz_standard_vtol` 기준 BT 전체 미션은 gripper stub 없이 성공했다. 구조 pickup 구간의 `CloseGripper`는 실제 `/model/standard_vtol_0/servo_5`, `/model/standard_vtol_0/servo_6` topic을 통해 실행됐고, `OpenGripper`도 별도 BT smoke에서 정상 publish를 확인했다.
