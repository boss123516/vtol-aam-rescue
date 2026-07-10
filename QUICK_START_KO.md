# 3단계 빠른 실행

## 1. 압축 풀기

```bash
mkdir -p ~/workspace/hzy
cd ~/workspace/hzy
unzip -o ~/다운로드/krac-control-team-complete-v2.zip
```

## 2. 최초 설정 및 빌드

```bash
cd ~/workspace/hzy/krac-control-team-complete
chmod +x setup.sh run.sh verify.sh edit_controller.sh scripts/*.sh
./setup.sh
```

`setup.sh`는 다음 순서로 자동 빌드합니다.

```text
krac_interfaces → krac_control
```

정상 출력:

```text
[SETUP] READY
```

## 3. 실행

```bash
./run.sh
```

제어팀 수정 파일:

```text
ros2_ws/src/krac_control/src/rescue_controller_team.py
```

수정 후 프로세스를 종료하고 다시 `./run.sh`를 실행합니다.
