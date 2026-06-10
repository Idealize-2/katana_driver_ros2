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
