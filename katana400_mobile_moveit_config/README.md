# katana400_mobile_moveit_config

MoveIt 2 configuration for the Katana 400 6M180 arm mounted on a differential-drive mobile base.
Supports coordinated base+arm whole-body planning via a planar virtual joint.

## Quick start (simulation)

```bash
colcon build --symlink-install --packages-select katana400_mobile_moveit_config
source install/setup.zsh
ros2 launch katana400_mobile_moveit_config demo.launch.py
```

---

## Planning groups

| Group | Type | Solver | Purpose |
|-------|------|--------|---------|
| `arm` | Kinematic chain `katana_base_link` → `katana_gripper_link` | KDL | Arm-only planning |
| `mobile_base` | Virtual joint `base_planar_joint` only | None | Base pose in odom frame |
| `base_arm` | Subgroups: `arm` + `mobile_base` | KDL* | Whole-body coordinated planning |
| `gripper` | `katana_l_finger_joint`, `katana_r_finger_joint` | None | Gripper open/close |

*KDL is a placeholder for `base_arm`. Replace with `pick_ik` for real whole-body IK:
```bash
sudo apt install ros-jazzy-pick-ik
```
Then in `config/kinematics.yaml` change `base_arm` solver to `pick_ik/PickIkPlugin`.

---

## Virtual joint

A `planar` virtual joint named `base_planar_joint` connects `base_footprint` to the `odom` frame.
This gives MoveIt 3 extra DOF (x, y, θ) to plan base motion alongside arm motion.

**Name chosen carefully:** The URDF already contains a joint named `base_joint` (fixed,
base_footprint → mobile_base_link). Using that name would silently shadow the real joint and
break the mobile_base group. `base_planar_joint` does not appear anywhere in the URDF.

---

## Named poses

| Pose | Group | Description |
|------|-------|-------------|
| `gripper_open` | gripper | Both fingers at +0.31 rad (fully open) |
| `gripper_closed` | gripper | Both fingers at −0.46 rad (fully closed) |
| `Home` | arm | Arm upright resting position |

---

## MoveIt Setup Assistant — how this config was made

### Prerequisites

- Combined URDF: `mobile_katana_description/urdf/katana400_mobile.urdf.xacro`
- Both `mobile_base_description` and `katana_description` packages built and sourced

### Steps

**1. Launch**
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```
Create New → load `katana400_mobile.urdf.xacro` → Load Files.

**2. Self-Collisions**
Generate Collision Matrix (default density). Done.

**3. Virtual Joints → Add Virtual Joint**

| Field | Value |
|-------|-------|
| Name | `base_planar_joint` |
| Child Link | `base_footprint` |
| Parent Frame | `odom` |
| Type | `planar` |

**4. Planning Groups**

*arm*
- Add Kin. Chain: base=`katana_base_link`, tip=`katana_gripper_link`
- Solver: `kdl_kinematics_plugin/KDLKinematicsPlugin`

*mobile_base*
- Add Joints: `base_planar_joint` only
- Solver: None
- Verify: joint list shows exactly one entry

*gripper*
- Add Joints: `katana_l_finger_joint`, `katana_r_finger_joint`
- Solver: None

*base_arm*
- Add Subgroups: `arm`, `mobile_base` — do NOT use Add Joints or Add Kin. Chain
- Solver: None (set to `pick_ik` in `kinematics.yaml` after generation)

**5. Robot Poses**

| Name | Group | l_finger | r_finger |
|------|-------|----------|----------|
| `gripper_open` | gripper | 0.31 | 0.31 |
| `gripper_closed` | gripper | −0.46 | −0.46 |

**6. End Effectors**
Name=`gripper`, Group=`gripper`, Parent Link=`katana_gripper_link`, Parent Group=`arm`.

**7. Passive Joints**
Mark `base_right_wheel_joint`, `base_left_wheel_joint`, `base_caster_wheel_joint` as passive.

**8. Generate Package** → output to `katana400_mobile_moveit_config/`.

### Post-generation fixes applied

- `kinematics.yaml`: added `base_arm` solver entry (KDL placeholder, swap to pick_ik)
- `katana400_mobile.srdf`: fixed gripper poses — added `katana_r_finger_joint` to both,
  corrected `gripper_closed` value from −0.25 → −0.46
- Added `config/ros2_controllers.yaml` — controller definitions for mock simulation
- Added `config/moveit_controllers.yaml` — MoveIt controller manager mapping

---

## Real hardware

This config uses `mock_components/GenericSystem` for simulation.
For real hardware, the `katana400_moveit_config` package handles the Katana arm via
`KatanaHardwareInterface`. The mobile base needs a `diff_drive_controller` wired to
the actual wheel encoders/motors. `base_planar_joint` feedback would come from odometry
(Nav2 → `odom` frame).

---

## `base_arm_pose_mover` — Whole-Body Pose Commander

A non-interactive tool that drives the **combined `base_arm` planning group** to a
Cartesian target in two phases:

1. **Phase 1 — Pre-solve (IK validation):** Pins the virtual planar joint to the
   requested base pose (`pos_x`, `pos_y`, `heading`) on a virtual copy of the robot
   state, then calls `setFromIK` on the `arm` group to confirm the target is reachable
   from that parking spot.  Exits immediately with an error if IK fails.
2. **Phase 2 — OMPL planning + execution:** Extracts the full `base_arm` joint vector
   from the solved virtual state and passes it to OMPL via `setJointValueTarget`.
   If OMPL finds a path the trajectory is executed immediately.

This approach avoids asking the whole-body planner to search the full (x, y, θ, 5-DOF)
space from scratch — the IK pre-solve provides a concrete joint-space goal that OMPL
can reach with standard planners.

---

### Prerequisites

The MoveIt `move_group` and simulation (or real hardware) must be running first.

```bash
# Simulation (Gazebo)
source install/setup.bash
ros2 launch katana400_mobile_moveit_config mobile_gazebo.launch.py

# --- OR ---

# Demo (mock hardware, RViz only)
ros2 launch katana400_mobile_moveit_config demo.launch.py
```

---

### Running

```bash
source install/setup.bash
ros2 launch katana400_mobile_moveit_config base_arm_pose_mover.launch.py \
    x:=0.5  y:=0.5  z:=0.4 \
    pos_x:=0.0  pos_y:=0.0  heading:=0.0
```

All arguments are optional — defaults are shown below.

#### Launch arguments

| Argument      | Default | Unit | Description |
|---------------|---------|------|-------------|
| `x`           | `0.5`   | m    | Target TCP X in the `odom` / world frame |
| `y`           | `0.5`   | m    | Target TCP Y in the `odom` / world frame |
| `z`           | `0.4`   | m    | Target TCP Z (height) |
| `pos_x`       | `0.0`   | m    | Base parking position X (where the mobile base will stand) |
| `pos_y`       | `0.0`   | m    | Base parking position Y |
| `heading`     | `0.0`   | rad  | Base yaw at the parking spot |
| `use_sim_time`| `false` | —    | Set `true` when running with Gazebo |

> **Target orientation** is fixed to identity (`w=1`, no rotation) — the tool commands
> position only.  Edit the source to add RPY control if needed.

---

### Algorithm in detail

```
                ┌─────────────────────────────────────────────┐
                │  Phase 1: IK pre-solve                      │
                │                                             │
                │  virtual_state  ← getCurrentState()         │
                │  set base_planar_joint/x   = pos_x          │
                │  set base_planar_joint/y   = pos_y          │
                │  set base_planar_joint/theta = heading       │
                │  virtual_state.update()                     │
                │                                             │
                │  setFromIK(arm_group, target_pose, 0.1 s)  │
                │    ✓ IK found → proceed to Phase 2          │
                │    ✗ IK failed → print error, exit(1)       │
                └─────────────────────────────────────────────┘
                                  │
                                  ▼
                ┌─────────────────────────────────────────────┐
                │  Phase 2: OMPL plan + execute               │
                │                                             │
                │  winning_joints ← virtual_state             │
                │                    .copyJointGroupPositions │
                │                    ("base_arm")             │
                │                                             │
                │  mg.setJointValueTarget(winning_joints)     │
                │  mg.setPlanningTime(10.0 s)                 │
                │  mg.plan(plan)  →  mg.execute(plan)         │
                └─────────────────────────────────────────────┘
```

---

### Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| `IK FAILED! Arm cannot reach target` | The arm cannot reach (x, y, z) from the given parking spot | Move the base closer (`pos_x`/`pos_y`) or try a different `heading` |
| `OMPL failed to plan a path` | Collision or joint-limit obstacle between start and goal | Try a different base parking pose or adjust the workspace `setWorkspace` bounds |
| `Could not find parameter 'x'` | Tool started with `ros2 run` instead of `ros2 launch` | Always use the launch file; it injects all required parameters |
| Execution starts but arm doesn't move | `move_group` or controllers not running | Make sure `demo.launch.py` or `mobile_gazebo.launch.py` is up |

---

### Related tools

| Tool | Package | Description |
|------|---------|-------------|
| `moveit_pose_mover` | `katana400_moveit_config` | Interactive MoveIt commander (arm only, real hardware) |
| `ik_pose_mover` | `katana_tutorials` | Direct KNI IK + move (no MoveIt) |
