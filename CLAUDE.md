# CLAUDE.md

This file provides guidance to Claude Code when working with this repository.

## Scope

**This workspace is configured and tested for the Neuronics Katana 400 6M180 arm only.**
Other Katana variants (300, 450) have partial code in the tree but are not wired up or tested.
The active packages are `katana_driver`, `katana400_moveit_config`, `katana_description`, and `kni`.

## Environment

- ROS 2 Jazzy, Ubuntu 24.04, zsh shell
- Workspace root: `~/Documents/INternSHipA4robotic-kanatarRobotarm/kanata_ws/`
- All source under `src/katana_driver_ros2/`
- User alias: `sp` → `source install/setup.zsh`

## Build

```bash
cd ~/Documents/INternSHipA4robotic-kanatarRobotarm/kanata_ws

# Full build
colcon build --symlink-install

# Single package
colcon build --symlink-install --packages-select katana400_moveit_config

# Source (or just type: sp)
source install/setup.zsh
```

Build logs go to `log/`. `build_out.txt` in the workspace root is a manually saved snapshot.

## Running the arm

### Everything at once (preferred)
```bash
ros2 launch katana400_moveit_config full_system.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1 calibrate_on_startup:=true
```

### Three separate terminals
```bash
# T1 — hardware driver + ros2_control
ros2 launch katana400_moveit_config real_hardware.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1 calibrate_on_startup:=true

# T2 — MoveIt move_group
ros2 launch katana400_moveit_config move_group.launch.py

# T3 — RViz
ros2 launch katana400_moveit_config moveit_rviz.launch.py
```

### Manual keyboard tester (no MoveIt needed)
```bash
ros2 run katana_test ros2control_tester
# Keys: 1-7 select joint, +/- move, e=enable motors, d=disable motors, q=quit
```

## Architecture — active driver path

The only working driver is the **`ros2_control` plugin** in `katana_driver`:

```
KatanaHardwareInterface  (katana_driver/src/katana_hardware_interface.cpp)
  └── implements hardware_interface::SystemInterface
  └── wraps CLMBase (KNI SDK) for read/write
  └── loaded by controller_manager via pluginlib
```

The older `katana` package (ROS 1 style, `KatanaNode.cpp`) **does not build** — it references removed ROS 2 message fields. Do not attempt to use or fix it without reading `build_out.txt` first.

### ros2_control stack
- `controller_manager` (`ros2_control_node`) loads `KatanaHardwareInterface`
- Three controllers spawned: `joint_state_broadcaster`, `arm_controller`, `gripper_controller`
- Config: `katana400_moveit_config/config/ros2_controllers.yaml`
- Update rate: 8 Hz (KNI read+write cycle ~110 ms)
- Spawners are delayed 5 s (`TimerAction`) to avoid a pluginlib cache segfault on fresh boot

### URDF offset/flip calibration
`KatanaHardwareInterface` applies per-joint offsets and direction flips to map KNI encoder space → URDF joint space. Parameters come from `katana_400_6m180_with_controlbox.ros2_control.xacro`:

```
urdf_pos = (kni_rad - urdf_offset_<joint>) * urdf_flip_<joint>
kni_rad  = (rad * urdf_flip_<joint>) + urdf_offset_<joint>
```

Current calibrated values (Katana 400 6M180, all joints flipped -1 except fingers):
- motor1 offset=0.8290, flip=-1
- motor2 offset=2.3208, flip=-1
- motor3 offset=2.7353, flip=-1
- motor4 offset=2.8212, flip=-1
- motor5 offset=0.9013, flip=-1
- l_finger offset=1.5927, flip=+1
- r_finger offset=0.0,   flip=+1

If the RViz model stops matching the real arm after recalibration, re-derive offsets with:
`new_offset = current_offset + (desired_value_for_that_joint)`

### KNI SDK (`kni` package)
`CLMBase` / `CCdlSocket` / `CCplSerialCRC`. Config files for Katana 400: `kni/KNI_4.3.0/configfiles400/katana6M180.cfg`. Built as a shared library via ament.

### Joints
- 5 arm joints: `katana_motor1_pan_joint` → `katana_motor5_wrist_roll_joint`
- 2 gripper joints: `katana_l_finger_joint`, `katana_r_finger_joint`
- Gripper is treated as a separate controller from the arm everywhere.

## MoveIt 2 (`katana400_moveit_config`)

Key files:
| File | Purpose |
|------|---------|
| `config/katana_400_6m180_with_controlbox.urdf.xacro` | Top-level URDF xacro (includes ros2_control xacro) |
| `config/katana_400_6m180_with_controlbox.ros2_control.xacro` | Hardware interface params (offsets, flips, connection args) |
| `config/moveit_controllers.yaml` | MoveIt flat-format controller list (read by MoveItConfigsBuilder) |
| `config/move_group_params.yaml` | **ROS 2 format** controller params passed to move_group node |
| `config/joint_limits.yaml` | Velocity/acceleration limits and position overrides |
| `config/initial_positions.yaml` | Starting joint values used by fake/sim hardware |
| `config/ros2_controllers.yaml` | ros2_control controller config (real hardware) |
| `launch/real_hardware.launch.py` | Hardware driver stack |
| `launch/move_group.launch.py` | MoveIt move_group node |
| `launch/moveit_rviz.launch.py` | RViz with MoveIt plugin |
| `launch/full_system.launch.py` | Combines the three above |

### Critical: two YAML files for controllers
`moveit_controllers.yaml` is in MoveIt's flat format — MoveItConfigsBuilder reads it.
`move_group_params.yaml` is in proper ROS 2 `/**:  ros__parameters:` format — passed as a file path to the move_group Node. Both must stay in sync. See `problem/moveit_0_controllers_in_list.md` for why this split is necessary.

### move_group.launch.py parameter loading order
```python
parameters=[
    moveit_config.to_dict(),   # robot desc, SRDF, kinematics, OMPL
    move_group_params,         # file path — controller manager (ROS 2 format)
    {"start_state_max_bounds_error": 0.05},
]
```

### real_hardware.launch.py design notes
- Passes `robot_description` directly to `ros2_control_node` (avoids QoS mismatch with RSP's latched topic in Jazzy)
- All spawners delayed 5 s via `TimerAction` (prevents JTC segfault from pluginlib cache race on fresh boot)

## Known build issues

- `katana` package: `JointTrajectoryControllerState` removed `.desired`/`.actual`/`.error` in ROS 2. Package does not build. Exclude it or fix the field names to `reference`/`feedback`/`error`.
- `katana_arm_gazebo`, `katana_gazebo_plugins`: still use ROS 1 XML launch syntax, not ported.

## Troubleshooting quick reference

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `ros2_control_node` crashes: "no ros2_control tag found" | QoS mismatch reading `/robot_description` topic | Pass `robot_description` directly as Node parameter — already fixed in `real_hardware.launch.py` |
| JTC segfault on fresh boot | pluginlib cache race condition | 5 s spawner delay — already fixed |
| `Returned 0 controllers in list` | `moveit_controllers.yaml` (flat format) not parsed by ROS 2 parameter loader | Use `move_group_params.yaml` (ROS 2 format) — already fixed. See `problem/moveit_0_controllers_in_list.md` |
| `START_STATE_INVALID` — joint out of bounds | Real arm position slightly outside URDF limits | Expand limit in URDF (`katana_400_6m180.urdf.xacro`) and `joint_limits.yaml` |
| RViz model doesn't match real arm | URDF offset/flip wrong | Re-derive offsets; update `.ros2_control.xacro` |

## Helper scripts (workspace root)

- `fix_log.py` — replaces ROS 1 logging macros with ROS 2 equivalents in `katana/src/Katana.cpp`
- `port_jtac.py` — writes the initial ROS 2 `joint_trajectory_action_controller.{h,cpp}` from embedded strings. Run once only.
- `debug_builder.py` — uses `MoveItConfigsBuilder` to inspect moveit config without launching

## Known issues folder

`problem/` at the workspace root contains detailed write-ups of bugs encountered and how they were fixed. Add new entries there when solving non-obvious issues.
