# 제어·비전팀 독립 BT 테스트 패키지

## 짧은 테스트 시나리오

```text
HOME 이륙
→ 조난자 구조지점 직행
→ OFFBOARD 인계
→ 제어팀 구조 모듈
→ 기본값: 착륙 → 5초 → 재이륙 → 안정 hover
→ SUCCESS
→ HOME 직행
→ 최종 VTOL 착륙
```

## 제어팀이 계속 수정할 파일

```text
ros2_ws/src/krac_control/src/rescue_controller_team.py
```

standalone BT와 원래 split BT가 이 파일을 공유합니다. 실행 wrapper가 source Python을 직접 실행하므로 Python 로직을 수정한 뒤 다시 빌드하지 않고 프로세스만 재실행해도 됩니다.

## 실행

```bash
cd ~/workspace/hzy/vtol-aam-rescue-split
./scripts/run_control_team_cycle.sh
```

원래 split 전체 미션에서 같은 제어 코드를 확인:

```bash
./scripts/run_split_with_control_team.sh
```

## 별도 미션 파일

```text
control_team_outbound.plan
  seq0 VTOL_TAKEOFF 10m
  seq1 RESCUE 직행 10m, autoContinue=false

control_team_return.plan
  seq0 HOME 직행 10m
  seq1 HOME VTOL_LAND
```

짧은 반복 테스트이므로 고정익 천이는 사용하지 않고 전 구간 MC로 실행합니다.

## 공유 인터페이스

```text
/krac/rescue_module/enable  Bool   BT→제어팀
/krac/rescue_module/ready   Bool   제어팀→BT
/krac/rescue_module/result  String 제어팀→BT
```

`result`는 `SUCCESS` 또는 `FAILURE:reason`입니다. `SUCCESS`는 상승과 안정 hover까지 끝난 뒤에만 보냅니다.

## 원래 split 반영

설치 스크립트는 원래 split XML의 `RescuePickupModule`만 공유 구조로 바꿉니다.

```text
CommandVTOLTransition MC
→ FlyToLocalPoint 구조 상공 3m
→ WaitForHoverStable
→ ExecuteExternalRescueModule
```

HOME→REP, 귀환, 최종 착륙 흐름은 유지합니다.
