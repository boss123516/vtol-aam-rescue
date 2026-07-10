# Standard VTOL 기반 BT 임무 검증 정리

작성일: 2026-07-05

## 1. 배경

처음에는 AMSR VTOL 기체로 SITL 임무를 진행하려고 했지만, AMSR 기체는 고정익 전환과 안정적인 mission cruise 검증이 충분히 되지 않았다. 따라서 당장 임무 시나리오를 완성하고 발표/검증 흐름을 만들기 위해 PX4에서 제공하는 `gz_standard_vtol` 기체로 전환해서 테스트하기로 했다.

기본 실행 흐름은 다음과 같았다.

```bash
ros2 launch krac_mission krac_24_sitl.launch.py
ros2 run ros_gz_image image_bridge /world/default/model/standard_vtol_0/link/camera_link/sensor/camera/image --ros-args -r /world/default/model/standard_vtol_0/link/camera_link/sensor/camera/image:=/camera/image_raw
ros2 run rqt_image_view rqt_image_view
./QGroundControl-x86_64.AppImage
~/ros2_ws/auto_spawn2.sh
ros2 launch mavros px4.launch fcu_url:=udp://127.0.0.1:14540@14557
ros2 service call /cmd/mission_proceed std_srvs/srv/Trigger
```

이후 실제 검증은 `krac_control`의 BT runner 중심으로 진행했다.

## 2. 목표 시나리오

BT 기반으로 아래 전체 시나리오를 수행하는 것이 목표다.

1. 쿼드 모드 이륙
2. 고정익 전환
3. 지정 waypoint 순항
4. REP 지점 도착
5. REP에서 쿼드 모드 전환
6. YOLO 기반 객체 탐지
7. 정밀 착륙 및 구조물 pickup
8. 쿼드 모드 재이륙
9. waypoint 역순 순항
10. 출발점 복귀
11. 최종 VTOL landing

## 3. 실제로 확인된 부분

PX4 `gz_standard_vtol` 기준으로 아래는 직접 확인했다.

- SITL launch 및 MAVROS 연결 성공
- mission upload 성공
- PX4 standard VTOL의 쿼드 이륙 및 고정익 mission 진입 성공
- REP 부근 waypoint 도달 확인
- REP에서 `OFFBOARD` 진입 확인
- REP에서 MC 전환 명령 및 MC 상태 확인
- simulated `/vision/target_error` 기반 `precision_lander` 동작 확인
- 정밀 하강 명령 출력 확인
- `CloseGripper` stub 성공을 통한 구조 pickup 구간 통과 확인
- 구조 후 `SetMissionCurrentWaypoint seq=5` 및 `AUTO.MISSION` 복귀 확인
- return leg waypoint 진행 및 home VTOL land item 접근 확인
- 최종 `OFFBOARD`/MC 전환 후 simulated target 기반 final precision landing 확인
- PX4 `Landing detected`, `Disarmed by landing` 확인
- BT root가 `SUCCESS`로 종료되는 것 확인

## 4. 검증 중 확인했던 문제와 처리

검증 중 확인했던 주요 문제는 다음과 같고, BT/mission plan 쪽에서 처리했다.

- 구조 후 재상승/복귀 단계에서 BT가 `GlobalMissionRecovery`로 반복 재진입하는 현상이 있었다.
- PX4는 mission 중간에 `MAV_CMD_NAV_VTOL_TAKEOFF`가 들어가면 `Mission rejected: takeoff not first waypoint item`으로 전체 mission을 거부했다.
- 그래서 `REP_TAKEOFF_AGAIN`을 `VTOL_TAKEOFF`로 넣는 방식은 사용할 수 없다.
- 구조 후 지면에 너무 가까운 상태에서 FW 전환/mission 재개를 시도하면 PX4에서 attitude failure가 발생할 수 있다.
- return leg의 `AMSLAltAboveTerrain`은 30m였지만 실제 MAVROS upload altitude인 `params[6]`가 10m로 남아 있어, rescue 이후 재상승한 기체가 mission 재개 시 다시 낮은 고도로 끌려가는 문제가 있었다.
- final landing은 32m 부근에서 시작하는데 `PrecisionLandOnTarget timeout_sec=60.0`은 simulated precision lander 하강속도 기준으로 부족했다.
- BT runner 종료 시 `rclcpp` shutdown 경계에서 DDS client/publisher 정리 crash가 발생했다.
- real YOLO 검증은 아직 완료되지 않았다. Gazebo camera topic은 확인했지만 ROS `/image_raw` bridge와 YOLO pipeline은 별도 검증이 필요하다.

## 5. 반영한 주요 변경

### 기체 전환

기본 SITL 기체를 AMSR 대신 PX4 standard VTOL로 바꾸는 방향으로 정리했다.

- `vehicle_model:=gz_standard_vtol`
- `scripts/run_sitl.sh`
- `scripts/run_sitl_bt.sh`
- `ros2_ws/src/krac_mission/launch/sitl_vtol.launch.py`

### 카메라

PX4 `standard_vtol` 모델에 `mono_cam`을 붙이도록 PX4 asset patch를 수정했다.

확인된 Gazebo camera topic:

```bash
/world/default/model/standard_vtol_0/link/camera_link/sensor/camera/image
/world/default/model/standard_vtol_0/link/camera_link/sensor/camera/camera_info
```

단, ROS image bridge는 아직 안정 검증이 필요하다.

### mission plan

PX4 mission feasibility를 통과시키기 위해 `way3.plan` 마지막에 `HOME_VTOL_LAND`를 추가했다.

```text
HOME_VTOL_LAND
command: 85  # MAV_CMD_NAV_VTOL_LAND
```

이 변경으로 PX4의 “landing waypoint/pattern required” 계열 문제를 줄였다.

또한 return leg인 `WP5_BACK`부터 `HOME_RETURN`까지 실제 upload altitude인 `params[6]`를 30m로 맞췄다. QGC 표시용 `Altitude`만 30m이고 `params[6]`가 10m이면 MAVROS mission upload 후 PX4는 10m waypoint로 비행한다.

### BT 안정화

BT 쪽에서 다음을 반영했다.

- `MissionUpload`가 비행 중 불필요하게 mission을 다시 clear/upload하지 않도록 조정
- `RescueOperation` 완료 후 재시작 시 구조 단계를 반복하지 않도록 `IsRescueCompleted` 조건 추가
- `AlignHeadingToWaypoint` timeout을 전체 실패가 아니라 best-effort warning으로 처리
- `SafetyGuard`의 global position freshness 조건을 SITL topic 주기에 맞게 완화
- `PrecisionLandOnTarget` target altitude를 너무 낮게 두지 않도록 조정
- `WaitForHoverStable`에 `timeout_sec`를 추가해 무한 `RUNNING` 상태를 막음
- `RecoveryAction`을 상태형 action으로 바꿔 실제 hover/climb/reacquire 동작을 수행하도록 수정
- `RescueOperation` retry 횟수를 늘려 precision landing recovery 이후 재시도를 허용
- `FinalLanding`의 `PrecisionLandOnTarget timeout_sec`를 150초로 늘려 30m급 고도에서 실제 하강 완료가 가능하게 조정
- BT 종료 전 logger, tree, global context, `MissionContext`, ROS node를 명시적으로 정리해 shutdown crash를 줄임

## 6. 중요한 결론

PX4 standard VTOL로 바꾼 판단은 맞다. AMSR 기체보다 이륙, FW 전환, mission cruise가 훨씬 안정적으로 진행된다.

현재 상태에서는 simulated `/vision/target_error`와 gripper stub 기준으로 전체 BT mission이 끝까지 자동 완주했다.

- 전반부: 이륙 -> FW 전환 -> waypoint -> REP 접근 성공
- 중반부: REP에서 MC 전환 -> target 기반 정밀 하강 -> pickup 성공
- 후반부: 구조 후 재상승 -> FW 복귀 -> return waypoints -> final VTOL landing 성공
- 종료부: BT `SUCCESS` 확인, runner shutdown path는 별도 `max_ticks` smoke test에서 exit code 0 확인

## 7. 다음에 해야 할 작업

남은 작업은 비행 구조 자체가 아니라 실제 perception 연결과 반복성 검증이다.

1. real YOLO pipeline을 별도 검증한다.
   - Gazebo camera topic은 있음
   - `/image_raw` bridge 또는 republisher를 고쳐야 함
   - YOLO가 `/vision/target_error`를 실제 이미지 기반으로 내는지 확인 필요

2. clean SITL 반복 실행으로 성공률을 확인한다.
   - 현재는 한 번의 full mission 성공을 확인한 상태
   - 발표/시연 전에는 같은 설정으로 2~3회 연속 실행해 timeout margin을 확인하는 것이 좋다

3. 실 gripper service를 붙인다.
   - 현재는 `gripper_stub_success=true` 기준으로 BT 구조 흐름을 검증했다
   - 실제 service 연결 시 `CloseGripper`, `VerifyPayloadSecured` 실패 경로를 같이 확인해야 한다

## 8. 현재 권장 전략

발표/시연 관점에서는 다음 전략이 가장 현실적이다.

- 기체는 PX4 `standard_vtol` 사용
- AMSR 기체는 “추후 실제 기체 모델 반영” 항목으로 분리
- BT 기반 mission architecture를 중심으로 설명
- 현재 완주 검증은 simulated `/vision/target_error` 기준이라고 명확히 표시
- 이후 image bridge를 고쳐 real YOLO를 붙이는 순서로 진행

## 9. 최종 상태 요약

현재까지의 검증 결과는 다음 한 문장으로 정리할 수 있다.

> PX4 `standard_vtol` 기반 BT 임무는 simulated `/vision/target_error`와 gripper stub 기준으로 이륙, FW 순항, REP 구조, 재상승, FW 복귀, home 접근, 최종 VTOL landing까지 전체 자동 완주를 확인했다.

## 10. 2026-07-05 추가 BT 실행 로그 요약

`mission_upload_stub_success:=false`, simulated `/vision/target_error`, `precision_lander` 조합으로 재검증했다.

확인된 개선:

- `MissionUpload`가 실제 `way3.plan`을 clear/upload했고, 업로드 성공을 확인했다.
- `CloseGripper`까지 도달해 rescue pickup 구간이 BT상 통과했다.
- `SetMissionCurrentWaypoint seq=5` 이후 `AUTO.MISSION` 복귀가 확인됐다.
- return leg의 실제 MAVROS `z_alt`를 30m로 수정한 뒤, `current_seq: 5`, `z_alt: 30.0` 및 seq6/seq7/seq8 진행이 확인됐다.
- full run에서 `wp_seq: 11`, `current_seq: 11`까지 도달해 home VTOL land item까지 접근했다.
- final landing에서 `OFFBOARD`/MC 상태로 simulated target을 따라 약 32m 부근부터 0.5m 부근까지 정밀 하강했다.
- 이후 `AUTO.LAND` 전환, PX4 `Landing detected`, `Disarmed by landing`을 확인했다.
- BT runner가 `BT finished with SUCCESS at tick 2678`까지 도달했다.

반영한 수정:

- `way3.plan`의 return leg `WP5_BACK`부터 `HOME_RETURN`까지 실제 upload altitude(`params[6]`, `z_alt`)를 30m로 맞췄다.
- `FinalLanding`의 `WaitForHoverStable` 조건을 rescue 구간과 같은 수준으로 완화하고 `timeout_sec`를 추가했다.
- `FinalLanding`의 `PrecisionLandOnTarget timeout_sec`를 150초로 늘렸다.
- `RecoverVisionLanding`/`RecoverFinalLanding`이 실제 hold altitude와 offboard stream을 수행하도록 상태형 action으로 바꿨다.
- runner shutdown crash를 줄이기 위해 shutdown 전에 BT tree/logger/context/node를 명시적으로 정리했다.

추가 확인:

- runner shutdown 패치 후 `max_ticks:=5` smoke test를 실행했고 exit code 0으로 종료됐다.
- 이 smoke test는 전체 비행 재검증은 아니고, shutdown cleanup 경로가 즉시 crash하지 않는지 확인한 것이다.
- real YOLO/image bridge와 실 gripper service는 아직 별도 검증 대상이다.
