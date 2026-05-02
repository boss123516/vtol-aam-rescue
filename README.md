# VTOL AAM Rescue Simulation

PX4 SITL + Gazebo 기반 VTOL AAM 구조 임무 시뮬레이션 프로젝트입니다.

본 프로젝트는 `amsr_vtol` 커스텀 기체를 사용하며, MAVROS를 통해 PX4와 ROS 2 노드를 연결하고, YOLO 기반 비전 노드를 통해 구조 지점, 착륙 지점, 투하 지점 등을 탐지합니다.

---

## 1. Tested Environment

현재 동작 확인된 환경은 아래와 같습니다.

| Item | Version / Setting |
|---|---|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| PX4 | PX4-Autopilot SITL |
| Gazebo | Gazebo Harmonic / gz-sim8 |
| MAVROS | `ros-humble-mavros` |
| Vision | YOLOv8 + PyTorch |
| Image Input | Custom `gz_image_republisher` |

> 원본 환경 가이드에는 Gazebo Garden 기준 내용도 포함되어 있으나, 현재 테스트 환경은 Gazebo Harmonic 계열입니다.  
> `ros_gz_image` / `ros_gz_bridge`에서 Gazebo image가 ROS Image로 정상 전달되지 않아, 현재는 커스텀 republisher 노드를 사용합니다.

---

## 2. Repository Structure

```text
vtol-aam-rescue/
├── px4_assets/
│   ├── airframes/
│   └── gz/
│       ├── models/
│       └── worlds/
├── ros2_ws/
│   └── src/
│       ├── krac_control/
│       ├── krac_mission/
│       ├── krac_utils/
│       ├── krac_vision/
│       ├── px4_msgs/
│       └── px4_ros_com/
├── scripts/
│   ├── apply_px4_assets.sh
│   ├── setup_ros2_ws.sh
│   ├── run_sitl.sh
│   ├── auto_spawn.sh
│   ├── run_gz_image_republisher.sh
│   ├── run_vision.sh
│   └── run_takeoff.sh
├── docs/
└── logs/
    └── competition/
```

---

## 3. Setup

### 3.1 Apply PX4 Assets

커스텀 기체, world, marker, object model을 PX4 경로로 복사합니다.

```bash
cd ~/vtol-aam-rescue
./scripts/apply_px4_assets.sh
```

PX4 단독 실행 확인:

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_amsr_vtol
```

---

### 3.2 Build ROS 2 Workspace

```bash
cd ~/vtol-aam-rescue/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

특정 패키지만 다시 빌드할 경우:

```bash
colcon build --symlink-install --packages-select krac_mission
colcon build --symlink-install --packages-select krac_utils
colcon build --symlink-install --packages-select krac_vision
```

---

## 4. Important Path Fixes

원본 코드 일부에는 친구 PC 기준 경로(`/home/kch/...`)가 남아 있었습니다.  
현재 환경에서는 아래처럼 수정해야 합니다.

### 4.1 Mission Plan Path

파일:

```text
ros2_ws/src/krac_mission/launch/sitl_vtol.launch.py
```

수정 전:

```python
parameters=[{'plan_file': '/home/kch/ros2_ws/src/krac_control/src/way3.plan'}]
```

수정 후:

```python
parameters=[{'plan_file': '/home/boss/vtol-aam-rescue/ros2_ws/src/krac_control/src/way3.plan'}]
```

수정 후 빌드:

```bash
cd ~/vtol-aam-rescue/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select krac_mission
source install/setup.bash
```

---

### 4.2 YOLO Weight Path

YOLO weight 파일 위치:

```text
~/vtol-aam-rescue/ros2_ws/src/krac_vision/weights/best.pt
```

확인:

```bash
find ~/vtol-aam-rescue -name "best.pt"
```

---

## 5. Run Sequence

아래 순서대로 터미널을 나누어 실행합니다.

### Terminal 1 — PX4 + Gazebo + MAVROS + Mission Nodes

```bash
cd ~/vtol-aam-rescue
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/run_sitl.sh
```

Gazebo에 `amsr_vtol_0` 기체가 나타나야 합니다.

확인:

```bash
gz topic -l | grep amsr_vtol
```

---

### Terminal 2 — QGroundControl

PX4 arming을 위해 GCS 연결이 필요할 수 있습니다.

```bash
chmod +x ~/QGroundControl.AppImage
~/QGroundControl.AppImage
```

PX4 로그에 아래 메시지가 뜨면 정상입니다.

```text
INFO [mavlink] partner IP: 127.0.0.1
INFO [commander] Ready for takeoff!
```

---

### Terminal 3 — Spawn Mission Objects

```bash
cd ~/vtol-aam-rescue
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/auto_spawn.sh
```

정상 실행 시 Gazebo entity tree에 아래 객체들이 생성됩니다.

```text
v_marker
rescue_box
victim
drop_marker
```

---

### Terminal 4 — Gazebo Camera to ROS Image

기존 `ros_gz_image` / `ros_gz_bridge` 방식 대신 커스텀 republisher를 사용합니다.

```bash
cd ~/vtol-aam-rescue
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/run_gz_image_republisher.sh
```

확인:

```bash
ros2 topic hz /image_raw
```

현재 우회 방식은 `gz topic`을 subprocess로 호출하는 구조이므로 약 `0.5~1.5 Hz` 수준입니다.

---

### Terminal 5 — YOLO Vision Node

```bash
cd ~/vtol-aam-rescue
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
./scripts/run_vision.sh
```

정상 로그 예시:

```text
YOLOv8 Model loaded
Detected: vertiport
Detected: basket
```

---

### Terminal 6 — Image Viewer

원본 이미지:

```bash
ros2 run image_view image_view --ros-args -r image:=/image_raw
```

YOLO debug image:

```bash
ros2 run image_view image_view --ros-args -r image:=/dbg_image
```

설치가 안 되어 있으면:

```bash
sudo apt install -y ros-humble-image-view
```

---

## 6. Useful ROS Topics

| Topic | Description |
|---|---|
| `/image_raw` | Gazebo camera image republished as ROS Image |
| `/dbg_image` | YOLO bounding box debug image |
| `/detections` | YOLO detection result |
| `/camera/target_error` | Target center error for visual servoing |
| `/camera/set_target` | Target class command |
| `/mavros/state` | PX4/MAVROS connection and arming state |
| `/mavros/local_position/pose` | Local position |
| `/mavros/global_position/global` | GPS position |
| `/mavros/mission/waypoints` | Uploaded mission waypoints |

확인 명령어:

```bash
ros2 topic hz /image_raw
ros2 topic hz /dbg_image
ros2 topic echo /detections --once
ros2 topic echo /camera/target_error --once
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /mavros/mission/waypoints --once
```

---

## 7. Takeoff

PX4가 아래 상태여야 이륙 가능합니다.

```yaml
connected: true
armed: false
mode: AUTO.LOITER
```

QGC 연결 후 PX4 로그에 아래 문구가 떠야 합니다.

```text
Ready for takeoff!
```

PX4 shell을 직접 사용할 수 있는 경우:

```text
pxh> commander arm -f
pxh> commander takeoff
```

주의:

```text
Disarmed by auto preflight disarming
```

이 로그가 뜨면 arm 후 takeoff 명령이 늦게 들어가 자동 disarm된 것입니다.  
`commander arm -f` 직후 바로 `commander takeoff`를 실행해야 합니다.

MAVROS service 방식:

```bash
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{base_mode: 0, custom_mode: 'AUTO.TAKEOFF'}"
```

단, MAVROS arming service는 PX4 shell의 `commander arm -f`처럼 force arm을 수행하지 못합니다.  
PX4 health check가 실패하면 `success=False`가 반환됩니다.

---

## 8. Mission Upload

Mission file:

```text
ros2_ws/src/krac_control/src/way3.plan
```

Mission loader:

```text
ros2_ws/src/krac_control/src/mission_loader.py
```

정상 로그:

```text
Uploading 7 items from .plan file...
WP: mission sended
Mission Upload SUCCESS!
```

주의:

```text
Mission Upload SUCCESS
```

는 미션이 PX4로 업로드되었다는 뜻이지, 기체가 자동으로 이륙해서 미션을 시작했다는 뜻은 아닙니다.

별도로 아래 조건이 필요합니다.

```text
GCS connection
Preflight check pass
Arming
Takeoff or AUTO.MISSION start
```

---

## 9. Competition GPS Log

GPS 로그는 아래 위치에 저장됩니다.

```text
~/vtol-aam-rescue/logs/competition/
```

CSV 형식:

```csv
Auto_Manual,Event_Flag,GPS_Time,Latitude,Longitude,Altitude
```

관련 코드:

```text
ros2_ws/src/krac_utils/src/competition_logger.cpp
```

확인:

```bash
ls -lh ~/vtol-aam-rescue/logs/competition
```

---

## 10. Known Issues

### 10.1 ros_gz_image / ros_gz_bridge Image Problem

Gazebo camera topic은 정상적으로 frame을 publish합니다.

```text
/world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image
```

Gazebo 원본 확인:

```bash
timeout 3 gz topic -e -t /world/default/model/amsr_vtol_0/link/camera_link/sensor/camera/image | grep -E "width|height|pixel_format|step"
```

정상 예시:

```text
width: 640
height: 480
step: 1920
pixel_format_type: RGB_INT8
```

하지만 현재 Gazebo Harmonic 환경에서는 `ros_gz_image` 또는 `ros_gz_bridge` 사용 시 `/image_raw` 토픽은 생성되지만 실제 frame이 흐르지 않는 문제가 있었습니다.

실패 예시:

```bash
ros2 topic info /image_raw
# Publisher count: 1

ros2 topic hz /image_raw
# no output
```

해결:

```bash
ros2 run krac_vision gz_image_republisher
```

현재 이 노드는 `gz topic`을 subprocess로 호출해 이미지를 가져오므로 약 `0.5~1.5 Hz` 수준입니다.  
시뮬레이션 검증용 workaround이며, Jetson 배포용 구조는 아닙니다.

---

### 10.2 YOLO Always-On Load

현재 YOLO는 `/image_raw`가 들어올 때마다 계속 inference를 수행합니다.  
Jetson Nano에서는 전체 비행 구간에서 YOLO를 계속 실행하기보다, goal point 또는 rescue waypoint에 도착했을 때만 inference를 활성화하는 구조가 적합합니다.

추천 구조:

```text
PX4 AUTO.MISSION
→ rescue waypoint 도착
→ BT/FSM에서 /camera/set_target publish
→ YOLO inference 활성화
→ target_error 기반 정밀 제어
→ 작업 완료 후 inference 비활성화
```

향후 개선 방향:

```text
YOLO node는 계속 실행
target이 off이면 inference skip
/camera/set_target = victim / basket / vertiport / off 로 동작 제어
```

---

## 11. Jetson Nano Deployment Note

Jetson Nano에서는 Gazebo image bridge를 사용할 필요가 없습니다.  
실기체/온보드 환경에서는 실제 CSI/USB 카메라를 직접 사용합니다.

권장 구조:

```text
USB/CSI Camera
→ Jetson YOLO node
→ /camera/target_error
→ MAVROS/PX4 companion control
```

시뮬레이션용 `gz_image_republisher`는 Jetson 배포용 노드가 아니라, Gazebo camera bridge 문제를 우회하기 위한 테스트용 노드입니다.

Jetson에서 비교해야 할 대상:

```text
YOLOv8 PyTorch FPS
ONNX FPS
TensorRT FPS
input resolution 640x480 vs 320x240
target_error publish rate
```
