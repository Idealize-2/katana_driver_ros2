# katana400_moveit_config

MoveIt 2 configuration and interactive commander for the **Katana 400 6M180** robot arm.

---

## `moveit_pose_mover` — Interactive MoveIt Commander

An interactive terminal tool for moving the Katana arm to Cartesian targets via the full
MoveIt 2 planning pipeline (`move_group`).  Unlike `ik_pose_mover` (which talks directly
to the KNI SDK), this tool uses MoveIt for IK, collision checking, and smooth trajectory
execution over `ros2_control`.

---

## Prerequisites

The full hardware + MoveIt stack must be running **before** starting the tool.

### Terminal 1 — full system launch

```bash
source install/setup.bash
ros2 launch katana400_moveit_config full_system.launch.py
```

Default arguments (all optional):

| Argument               | Default         | Description                          |
|------------------------|-----------------|--------------------------------------|
| `connection_type`      | `tcp`           | `tcp` or `serial`                    |
| `ip_address`           | `192.168.1.1`   | Arm IP address (TCP mode)            |
| `tcp_port`             | `5566`          | KNI TCP port (TCP mode)              |
| `calibrate_on_startup` | `true`          | Run calibration on hardware activate |

Example with explicit arguments:

```bash
ros2 launch katana400_moveit_config full_system.launch.py \
    connection_type:=tcp ip_address:=192.168.1.1 calibrate_on_startup:=true
```

> **What `full_system.launch.py` starts:**
> - `real_hardware.launch.py` — `ros2_control` + KNI hardware interface
> - `move_group.launch.py` — MoveIt `move_group` node
> - `moveit_rviz.launch.py` — RViz 2 with MoveIt plugin

---

## Running the commander

### Terminal 2 — moveit_pose_mover

```bash
source install/setup.bash
ros2 launch katana400_moveit_config moveit_pose_mover.launch.py
```

> **Important:** You must use the launch file, **not** `ros2 run`.  
> The launch file injects `robot_description`, `robot_description_semantic`, and
> `robot_description_kinematics` parameters that `MoveGroupInterface` requires.
> Running with `ros2 run` will print a fatal error and exit immediately.

---

## Interactive menu

On startup the tool prints the current TCP pose, then shows a command prompt:

```
Current TCP:  X=  0.2804 m  Y=  0.0001 m  Z=  0.3510 m  R= -0.0001 rad  P=  1.5706 rad  Yaw= -0.0003 rad

  Commands:  [p] Pose goal   [c] Cartesian path   [j] Joint positions   [q] Quit
  >
```

### `[p]` — Single pose goal

Plans and executes a move to a single Cartesian target.

```
> p

┌── Pose Goal ──────────────────────────────────────────────────┐
│  Enter target pose in the katana_base_link frame:             │
│  Format:  X   Y   Z   Roll   Pitch   Yaw  (metres, radians)  │
│  Example: 0.25 0.0 0.30 0.0 1.5708 0.0                       │
└───────────────────────────────────────────────────────────────┘

  X Y Z Roll Pitch Yaw: 0.25 0.0 0.30 0.0 1.5708 0.0
```

**Workflow:**
1. Enter target pose (6 numbers: X Y Z Roll Pitch Yaw).
2. MoveIt plans a collision-free trajectory (up to 10 s planning time).
3. A joint preview table shows current → target angle and delta for each motor.
4. Press **Enter** to execute on the real arm, or **Ctrl+C** to abort.
5. The reached TCP pose is printed after execution.

### `[c]` — Cartesian path

Plans and executes a straight-line path through multiple waypoints.

```
> c

┌── Cartesian Path ──────────────────────────────────────────────┐
│  Enter waypoints one per line (X Y Z Roll Pitch Yaw, m/rad).  │
│  The arm will follow a STRAIGHT LINE between each pair.       │
│  Type  done  (or press Ctrl+D) when finished.                 │
└────────────────────────────────────────────────────────────────┘

  Start (current TCP pose):
    WP0:  X=  0.2804 m  ...

  WP1 (X Y Z Roll Pitch Yaw) or 'done': 0.25 0.0 0.35 0.0 1.5708 0.0
  WP2 (X Y Z Roll Pitch Yaw) or 'done': 0.20 0.0 0.35 0.0 1.5708 0.0
  WP3 (X Y Z Roll Pitch Yaw) or 'done': done
```

**Workflow:**
1. Enter waypoints one per line; type `done` when finished.
   The path automatically starts from the current TCP pose.
2. `computeCartesianPath` interpolates at **1 cm EEF steps**.
3. Planning succeeds only if **≥ 95 %** of the path is reachable.
   If coverage is lower, adjust the waypoints and try again.
4. A joint preview table is shown (start → end joints).
5. Press **Enter** to execute, or **Ctrl+C** to abort.

### `[j]` — Print current joint positions

Prints all 5 arm joint angles in radians and degrees:

```
    katana_motor1_pan_joint               -0.0003 rad  (-0.0°)
    katana_motor2_lift_joint               2.1604 rad  (123.8°)
    ...
```

### `[q]` — Quit

Gracefully shuts down the ROS 2 node and exits.

---

## Units and reference frame

| Quantity        | Unit     | Notes                                              |
|-----------------|----------|----------------------------------------------------|
| X / Y / Z       | metres   | e.g. `0.25` = 25 cm                               |
| Roll/Pitch/Yaw  | radians  | Extrinsic XYZ order; e.g. `1.5708` ≈ 90°          |
| Planning frame  | —        | `katana_base_link` (origin at the arm pivot)       |
| End-effector    | —        | `katana_gripper_link` (centre of gripper jaw gap)  |

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `moveit_pose_mover must be started with its launch file` | Used `ros2 run` instead of `ros2 launch` | Use the launch file |
| Planning fails (`FAILED code=…`) | Target unreachable, in collision, or outside joint limits | Try a different pose |
| Cartesian path coverage < 95 % | Path crosses a singularity or joint limit | Space waypoints closer together or change orientation |
| `Could not read current pose` | `move_group` not yet ready | Wait a few seconds after launching `full_system.launch.py` |

---

## Related tools

| Tool | Package | Description |
|------|---------|-------------|
| `ik_pose_mover` | `katana_tutorials` | Direct KNI IK + move (no MoveIt, no `ros2_control`) |
| `arm_disable_encoder` | `katana_test` | Disable motors, read encoders live |
| `katana_kni_teleop` | `katana_test` | Keyboard teleoperation via KNI |
