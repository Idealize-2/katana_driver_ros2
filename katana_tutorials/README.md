# katana_tutorials

Code tutorials for the **Neuronics Katana 400 6M180** arm under ROS 2 Jazzy.
Covers trajectory control via `ros2_control`, geometric inverse kinematics, and
direct KNI-SDK Cartesian motion — progressing from simple to advanced.

---

## Prerequisites

### For ros2_control tutorials (1, 2, 3, 5, 6)

The arm must be running through `ros2_control` — specifically:

```bash
# Terminal 1 — hardware interface + controllers
ros2 launch katana400_moveit_config real_hardware.launch.py
```

The following must be in `active` state (verify with `ros2 control list_controllers`):

| Controller | Role |
|-----------|------|
| `joint_state_broadcaster` | Publishes `/joint_states` |
| `arm_controller` | Accepts `FollowJointTrajectory` goals |

### For the KNI direct tutorial (4)

> ⚠️ **Stop ros2_control first.** Both processes compete for the same TCP socket.

- Katana 400 powered on and reachable (`192.168.1.1:5566` by default)
- `kni` package installed and workspace sourced

---

## Shared library: `KatanaArmClient`

All ros2_control tutorials share a helper class defined in
`include/katana_tutorials/katana_arm_client.hpp` / `src/katana_arm_client.cpp`.

| Method | What it does |
|--------|-------------|
| `waitForJointState(timeout)` | Spins until `/joint_states` is received |
| `currentJointPositions()` | Thread-safe read of current arm positions (rad) |
| `sendTrajectoryAndWait(traj)` | Sends `FollowJointTrajectory` goal and blocks until done |
| `makeTrajectory(joints, current, target, t1, t2)` | Builds a smooth 3-point trajectory (start → target → hold) |

---

## Tutorials

### Tutorial 1 — `follow_joint_trajectory_client`

**Type:** ros2_control  
**Concept:** Hardcoded 2-waypoint trajectory with confirmation prompt

Moves the arm through two named poses: a **calibration pose** and a **ready pose**.
Before each move it prints a side-by-side table of current vs target joint angles
and waits for you to press Enter (press Ctrl+C to abort).

```bash
ros2 run katana_tutorials follow_joint_trajectory_client
```

**Waypoints (rad):**

| Pose | motor1 | motor2 | motor3 | motor4 | motor5 |
|------|--------|--------|--------|--------|--------|
| Calibration | -3.03 | 2.17 | -2.22 | -2.03 | -2.99 |
| Ready | 0.00 | 1.57 | 0.00 | 0.00 | 0.00 |

---

### Tutorial 2 — `sixR_inverse_kinematics`

**Type:** ros2_control  
**Concept:** 3D geometric IK — simplified 3R model (no link offsets)

You type an `X Y Z` target in **cm** (relative to the arm base).
The program solves the 3-joint law-of-cosines IK and moves the arm.

```bash
ros2 run katana_tutorials sixR_inverse_kinematics
# Prompt: Enter target X Y Z (in cm, e.g. 15 10 10):
```

**IK model:** Two-link planar model in 3D.  
`θ1 = atan2(y,x)`, `θ2 = shoulder`, `θ3 = elbow` (law of cosines).  
Max reach ≈ **50.4 cm** (`a2=19.1 cm + a3=31.33 cm`).  
Motors 3 and 5 are fixed at 0 in this simplified model.

> Use Tutorial 4 for exact reachability validation via KNI's own `IKCalculate()`.

---

### Tutorial 3 — `offset_inverse_kinematics`

**Type:** ros2_control  
**Concept:** 3D geometric IK — full DH-parameter model with link offset

More accurate IK that correctly handles the lateral shoulder offset (`d`) in
the Denavit–Hartenberg parameterisation.

```bash
ros2 run katana_tutorials offset_inverse_kinematics
# Prompt: Enter target X Y Z (in cm, e.g. 15 10 10):
```

**IK model:**  
`θ1 = atan2(y,x) + atan2(-√(x²+y²−d²), d)`  
`θ3 = atan2(-√(1−D²), D)` where `D = (x²+y²+z²−d²−a2²−a3²) / (2·a2·a3)`  
`θ2 = atan2(z, √(x²+y²−d²)) − atan2(a3·sin(θ3), a2+a3·cos(θ3))`

For the 6M180 variant the offset `d = 0`, so this reduces to Tutorial 2's
model — but the code handles the general case correctly.

---

### Tutorial 4 — `ik_pose_mover`

**Type:** KNI SDK direct (⚠️ stop ros2_control first)  
**Concept:** Full Cartesian motion using KNI's built-in IK + guided 4-step workflow

The most complete IK tutorial. Uses `KNI::IKCalculate()` (the same solver
the hardware uses internally) followed by `moveRobotTo()` for exact Cartesian
positioning.

#### Run

```bash
# All args optional (defaults to tcp 192.168.1.1, config auto-resolved if workspace is sourced)
ros2 run katana_tutorials ik_pose_mover
```

> You can also override defaults, for example: `ros2 run katana_tutorials ik_pose_mover tcp 192.168.1.1 /path/to/config.cfg`

#### 4-step guided workflow

| Step | What happens |
|------|-------------|
| **0 — Calibrate** | Arm moves to all joint limits to find zero encoders (~30 s) |
| **1 — Limp** | Motors off; move arm by hand to desired start pose. Live TCP pose (`X Y Z Al Be Ga`) prints at ~5 Hz. Press Enter when done. |
| **2 — IK validate** | Enter target `X Y Z Al Be Ga solution_iter` (mm + rad). `IKCalculate()` checks reachability. Re-prompts if unreachable. |
| **3 — Execute** | Motors on, arm moves to the confirmed Cartesian target. Final pose printed. Choose `n` to enter a new target or `q` to quit. |

**Input units:** `X Y Z` in **mm**, `Al Be Ga` in **rad** (Euler ZYX), `solution_iter` = integer (0 = nearest to current config).

---

### Tutorial 5 — `pr2_joint_trajectory_client`

**Type:** ros2_control  
**Concept:** Multi-waypoint sequence, each waypoint confirmed before execution

Sends the arm through a scripted 3-move sequence, waiting for each move to
complete before sending the next:

```
current  →  calibration pose  →  straight up  →  calibration pose
```

```bash
ros2 run katana_tutorials pr2_joint_trajectory_client
```

**Poses (rad):**

| Pose | motor1 | motor2 | motor3 | motor4 | motor5 |
|------|--------|--------|--------|--------|--------|
| Calibration | -2.96 | 2.14 | -2.16 | -1.97 | -2.93 |
| Straight up | 0.00 | 1.57 | 0.00 | 0.00 | 0.00 |

---

### Tutorial 6 — `test_inverse_kinematics`

**Type:** ros2_control  
**Concept:** 2D planar IK (sagittal plane, pan fixed at 0)

Simplest IK tutorial. The arm always faces forward (`motor1 = 0`).
You type an `X Y` target (forward / up, in cm) and the 2-link
law-of-cosines solver moves the arm in the vertical plane.

```bash
ros2 run katana_tutorials test_inverse_kinematics
# Prompt: Enter target X Y (in cm, X=forward, Y=up, e.g.  20 10):
```

For `X < 0` (target behind the base) the arm automatically bends backward (`k = -1`).

---

## Quick-reference cheat sheet

```bash
source install/setup.bash

# ── Requires ros2_control active ──────────────────────────────────────────────
ros2 run katana_tutorials follow_joint_trajectory_client   # Tutorial 1
ros2 run katana_tutorials sixR_inverse_kinematics          # Tutorial 2
ros2 run katana_tutorials offset_inverse_kinematics        # Tutorial 3
ros2 run katana_tutorials pr2_joint_trajectory_client      # Tutorial 5
ros2 run katana_tutorials test_inverse_kinematics          # Tutorial 6

# ── KNI direct (stop ros2_control first) ─────────────────────────────────────
ros2 run katana_tutorials ik_pose_mover 
```

---

## Arm geometry reference

```
Katana 400 6M180 — link lengths (from katana6M180.cfg [ENDEFFECTOR])
  segment2             = 191.0 mm  =  19.10 cm   (shoulder → elbow)
  segment3 + segment4  = 313.3 mm  =  31.33 cm   (elbow → TCP)
  Maximum reach                    ≈  50.4 cm

Joint names:
  motor1  katana_motor1_pan_joint        — pan / yaw
  motor2  katana_motor2_lift_joint       — shoulder lift
  motor3  katana_motor3_lift_joint       — elbow (upper)
  motor4  katana_motor4_lift_joint       — elbow (lower)
  motor5  katana_motor5_wrist_roll_joint — wrist roll
```
