# 실행 명령어 모음 (2026-07-06 기준, 설치/빌드 완료 후)

이 문서는 환경 설치·빌드가 끝난 상태에서 시나리오를 직접 돌려보기 위해
**지금 실행하면 되는 명령어만** 순서대로 정리한 것이다. 설치 과정은
`docs/environment_setup.md`, 브랜치의 변경 배경은 `docs/HANDOFF_2026-07-06.md`
참고.

## 0. 사전 준비 (매 터미널 공통)

```bash
source /opt/ros/humble/setup.bash
source ~/vtol-aam-rescue/ros2_ws/install/setup.bash
```

## 1. PX4 SITL 단독 동작 확인 (standard_vtol)

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

성공 기준: Gazebo 창이 뜨고 `standard_vtol` 기체가 보이며 `pxh>` 셸이 열림.
종료는 `Ctrl+C`.

## 2. AMSR 커스텀 기체 확인

에셋은 이미 적용/빌드되어 있음 (`px4_assets/gz/models/amsr_vtol`,
airframe `1984_gz_amsr_vtol`). 다시 적용하려면:

```bash
cd ~/vtol-aam-rescue
./scripts/apply_px4_assets.sh
```

확인 실행:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_amsr_vtol
```

성공 기준: 커스텀 `amsr_vtol` 기체가 보이고 mesh 누락 에러 없음.

## 3. 전체 BT 미션 실행 (실제 시나리오)

터미널 하나로 PX4 + MAVROS + gripper bridge + auto-spawn + vision까지
전부 띄우는 스크립트.

```bash
cd ~/vtol-aam-rescue
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/run_sitl_bt.sh
```

기본값: `VEHICLE_MODEL=gz_standard_vtol`, `GZ_MODEL_NAME=standard_vtol_0`.

AMSR 모델로 실행하려면:

```bash
VEHICLE_MODEL=gz_amsr_vtol GZ_MODEL_NAME=amsr_vtol_0 ./scripts/run_sitl_bt.sh
```

로그는 `/tmp/krac_ros_logs/bt_run_<timestamp>/`에 남는다.

## 4. QGroundControl (미션 업로드/모니터링용, 선택)

```bash
~/QGroundControl.AppImage
```

## 5. 디버그용 확인 명령어

```bash
ros2 node list
ros2 topic list
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /vision/target_error --once
gz topic -l | grep vtol
```

미션 강제 진행(다음 웨이포인트로 넘기고 싶을 때):

```bash
ros2 service call /cmd/mission_proceed std_srvs/srv/Trigger
```

## 6. 디버그 로그까지 수집하며 실행

```bash
cd ~/vtol-aam-rescue
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
timeout 360 scripts/run_bt_debug.sh
```

## 현재 상태 요약

- 이 브랜치(`fix/repro-run`)에서 `gz_standard_vtol` 기준 real mission upload
  → rescue precision landing → gripper close → return → final landing →
  `AUTO.LAND` → disarm까지 전체 BT 미션 성공 확인됨 (2026-07-06,
  `docs/standard_vtol_bt_gripper_full_validation_2026-07-06.md`).
- 이 머신에서는 PX4 재빌드(`make px4_sitl_default`, 62/62 성공)와
  `ros2_ws` colcon 빌드(7개 패키지 전부 성공, `ros2 pkg list | grep krac`
  확인됨)까지 끝난 상태이며, 위 1~3번 실제 GUI 실행/시나리오 확인만 남아있다.
- 아직 남은 이슈 (`docs/HANDOFF_2026-07-06.md` 참고):
  - real YOLO 기반 full mission은 아직 최종 검증 전 (현재는 simulated
    `/vision/target_error` 기준으로만 성공 확인).
  - drop leg(`CruiseToDrop`/`DropOperation`/`ResumeAfterDrop`)는 실제
    drop-zone waypoint가 없어 BT main path에서 제외된 상태.
  - AMSR 커스텀 기체는 asset은 갖춰졌으나 최종 full mission 검증 기준은
    아직 `gz_standard_vtol`.
