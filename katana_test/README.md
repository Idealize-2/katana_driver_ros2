# katana_test

Diagnostic and test tools for the **Neuronics Katana 400** arm. Contains both low-level KNI (direct hardware) utilities and a MoveIt-based pose runner.

---

## Tools overview

| Executable | Transport | Purpose |
|-----------|-----------|---------|
| `katana_kni_teleop` | KNI SDK (TCP/serial) | Low-level keyboard teleop — direct encoder control, bypasses ros2_control |
| `arm_disable_encoder` | KNI SDK (TCP) | Disables motors so the arm can be moved by hand; prints live encoder positions |
| `pose_runner` | MoveIt 2 | Loads named poses from YAML, plans & executes with preview |

---

## 1. `katana_kni_teleop` — Low-level KNI keyboard teleop

> ⚠️ **Direct hardware access.** Do NOT run while `ros2_control` / `move_group` is active — both will try to own the KNI TCP connection and collide.

### Prerequisites
- Katana 400 powered on and reachable at its IP (default `192.168.1.1:5566`)
- `kni` package installed and workspace sourced

### Run

```bash
source install/setup.bash

# Default IP/port
ros2 run katana_test katana_kni_teleop

# Custom IP / port
ros2 run katana_test katana_kni_teleop 192.168.1.1 5566

# Via launch file (supports ip:= and port:= args)
ros2 launch katana_test katana_kni_teleop.launch.py
ros2 launch katana_test katana_kni_teleop.launch.py ip:=192.168.1.1 port:=5566
```

### What happens on startup
1. Connects to the arm controller via TCP
2. Clears any fault flags (`unBlock`)
3. **Calibrates** — the arm moves to all joint limit switches (takes ~30 s)
4. Moves to a "straight up" home position
5. Enters the keyboard loop

### Key bindings

```
+--------------------------------------------------+
|   Katana 400 Low-Level KNI Teleop (diagnostic)   |
+--------------------------------------------------+
|  0 - 5  ->  select active motor                  |
|  W / S  ->  jog active motor up / down           |
|  A / D  ->  motor 0 (pan)  left / right          |
|  H      ->  go to home position                  |
|  E      ->  enable motors                        |
|  F      ->  freeze motors                        |
|  + / -  ->  double / halve step size             |
|  P      ->  print encoder values                 |
|  Q      ->  quit (freeze + power off)            |
+--------------------------------------------------+
```

> **Motor numbering** (0-based KNI index):
> `0` = pan, `1` = lift1, `2` = lift2, `3` = lift3, `4` = wrist, `5` = gripper

## 2. `arm_disable_encoder` — Disable motors / read encoders

Connects to the arm, **disables all motors** (arm goes limp — safe to move by hand), then continuously prints encoder positions so you can record joint angles. Press Ctrl+C to quit.

### Run

```bash
# All args optional — config auto-resolved from kni package share dir
ros2 run katana_test arm_disable_encoder

# Override connection
ros2 run katana_test arm_disable_encoder tcp 192.168.1.1

# Override all (serial example)
ros2 run katana_test arm_disable_encoder serial 0

# Override all including config file
ros2 run katana_test arm_disable_encoder tcp 192.168.1.1 /path/to/katana6M180.cfg
```

**Argument order:** `[tcp|serial]  [IP_or_PortNum]  [CONFIG_FILE]`  
All are positional and optional. Defaults: `tcp  192.168.1.1  <kni_share>/KNI_4.3.0/configfiles400/katana6M180.cfg`

---

## 3. `pose_runner` — MoveIt interactive pose runner

Loads named arm configurations from `config/katana_poses.yaml`, plans to a selected pose with MoveIt (trajectory is previewed as a ghost in RViz), shows a joint-by-joint table, then executes after you confirm with Enter.

### Prerequisites

All three must be running:

```bash
# Terminal 1 — ros2_control + hardware
ros2 launch katana400_moveit_config real_hardware.launch.py

# Terminal 2 — MoveIt move_group
ros2 launch katana400_moveit_config move_group.launch.py

# Terminal 3 — RViz (optional, for trajectory ghost preview)
ros2 launch katana400_moveit_config moveit_rviz.launch.py
```

### Run

```bash
source install/setup.bash
ros2 run katana_test pose_runner
```

### Adding / editing poses

Edit `config/katana_poses.yaml`:

```yaml
poses:
  home:
    katana_motor1_pan_joint:         0.0
    katana_motor2_lift_joint:        0.0
    katana_motor3_lift_joint:        0.0
    katana_motor4_lift_joint:        0.0
    katana_motor5_wrist_roll_joint:  0.0

  pick_ready:
    katana_motor1_pan_joint:         0.2
    katana_motor2_lift_joint:       -0.5
    katana_motor3_lift_joint:        0.8
    katana_motor4_lift_joint:       -0.3
    katana_motor5_wrist_roll_joint:  0.0
```

---

## Quick-reference cheat sheet

```bash
# ── Direct KNI tools (ros2_control must be OFF) ───────────────────────────────
# All args optional — defaults: tcp  192.168.1.1  <auto config>
ros2 run katana_test arm_disable_encoder
ros2 run katana_test arm_disable_encoder tcp 192.168.1.1
ros2 run katana_test katana_kni_teleop
ros2 run katana_test katana_kni_teleop 192.168.1.1 5566
ros2 launch katana_test katana_kni_teleop.launch.py [ip:=...] [port:=...]

# ── MoveIt tools (ros2_control + move_group must be ON) ───────────────────────
ros2 run katana_test pose_runner
```
