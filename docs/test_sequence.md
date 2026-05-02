# Test sequence

## Stage 0: PX4/Gazebo sanity check

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_standard_vtol
```

Expected:

- Gazebo starts.
- Standard VTOL appears.
- PX4 shell opens.

## Stage 1: Custom AMSR VTOL model check

```bash
cd ~/vtol-aam-rescue
./scripts/apply_px4_assets.sh
cd ~/PX4-Autopilot
make px4_sitl gz_amsr_vtol
```

Expected:

- Custom `amsr_vtol` appears.
- No missing mesh error.

## Stage 2: ROS 2 workspace build

```bash
cd ~/vtol-aam-rescue/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 pkg list | grep krac
```

Expected packages:

- `krac_control`
- `krac_interfaces`
- `krac_mission`
- `krac_utils`
- `krac_vision`

## Stage 3: Full launch

Terminal 1:

```bash
cd ~/vtol-aam-rescue
./scripts/run_sitl.sh
```

Terminal 2:

```bash
cd ~/vtol-aam-rescue
./scripts/auto_spawn.sh
```

Terminal 3:

```bash
cd ~/vtol-aam-rescue
./scripts/run_image_bridge_amsr.sh
```

Terminal 4:

```bash
cd ~/vtol-aam-rescue
./scripts/run_vision.sh
```

Terminal 5:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Debug commands

```bash
ros2 node list
ros2 topic list
ros2 topic echo /mavros/state
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /camera/target_error --once
ros2 topic hz /image_raw
gz topic -l
rqt_graph
```

## Mission proceed service

```bash
ros2 service call /cmd/mission_proceed std_srvs/srv/Trigger
```
