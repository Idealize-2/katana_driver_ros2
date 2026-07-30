# katana_teleop

Keyboard teleoperation for the **Neuronics Katana 400** robotic arm, operating entirely through the `ros2_control` layer. No direct KNI SDK calls — all commands go through `arm_controller`, `gripper_controller`, and the `katana_hw` hardware interface.

---

## Prerequisites

Before running the teleop, the following must already be running:

| What | Why |
|------|-----|
| `joint_state_broadcaster` | Publishes `/joint_states` so the teleop can read live positions |
| `arm_controller` (`JointTrajectoryController`) | Accepts jog & home commands |
| `gripper_controller` (`GripperActionController`) | Accepts open/close commands |
| `katana_hw/set_motors_enabled` service | (Optional) Motor enable/disable |

Typically you start everything with the hardware launch file:

```bash
# Terminal 1 — bring up ros2_control + controllers
ros2 launch katana400_moveit_config real_hardware.launch.py
```

---

## Running

```bash
# Source the workspace first
source install/setup.bash

# Run the teleop
ros2 run katana_teleop katana_teleop_key

# Or via the launch file
ros2 launch katana_teleop katana_teleop_key.launch.py
```

---

## Key Bindings

```
╔══════════════════════════════════════════════════════╗
║   Katana ros2_control Keyboard Teleop                ║
╠══════════════════════════════════════════════════════╣
║  1 – 5   Select arm joint to jog                    ║
║  W / S   Jog selected joint  + / - step             ║
║  A / D   Jog joint 1 (pan)   + / - step             ║
║  H       Home  (all joints → 0 rad, 3 s)            ║
║  G       Open  gripper                              ║
║  C       Close gripper                              ║
║  E       Enable  motors                             ║
║  P       Print joint states                         ║
║  + / =   Double  step size                          ║
║  -       Halve   step size                          ║
║  ?       Show help                                  ║
║  Q       Quit                                       ║
╚══════════════════════════════════════════════════════╝
```

### Joint numbering

| Key | Joint name |
|-----|-----------|
| 1 | `katana_motor1_pan_joint` (pan / yaw) |
| 2 | `katana_motor2_lift_joint` |
| 3 | `katana_motor3_lift_joint` |
| 4 | `katana_motor4_lift_joint` |
| 5 | `katana_motor5_wrist_roll_joint` |

---

## How it works

| Command | Transport |
|---------|-----------|
| **Jog** (W/S/A/D) | Publishes directly to `/arm_controller/joint_trajectory` topic — preempts any active trajectory immediately for snappy response |
| **Home** (H) | Sends `FollowJointTrajectory` action goal to `/arm_controller/follow_joint_trajectory` |
| **Gripper** (G/C) | Sends `GripperCommand` action goal to `/gripper_controller/gripper_cmd` |
| **Motor power** (E) | Calls `katana_hw/set_motors_enabled` service |

---

## Troubleshooting

| Symptom | Likely cause |
|---------|-------------|
| `Waiting for /joint_states...` hangs | `joint_state_broadcaster` not active — check `ros2 control list_controllers` |
| Jog does nothing | `arm_controller` not in `active` state |
| Gripper G/C times out | `gripper_controller` not active, or action server name mismatch |
| Motor E does nothing | Hardware interface not running / service not advertised |
