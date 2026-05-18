# MoveIt2: "Returned 0 controllers in list" from simple_controller_manager

## Symptom

Planning succeeds but execution always fails with:

```
[move_group] [INFO] simple_controller_manager: Returned 0 controllers in list
[move_group] [ERROR] trajectory_execution_manager: Unable to identify any set of controllers
                     that can actuate the specified joints: [joint1, joint2, ...]
[move_group] [ERROR] trajectory_execution_manager: Known controllers and their joints:
```

The "Known controllers" line is blank — `simple_controller_manager` has zero controllers
configured, even though `moveit_controllers.yaml` exists and is being passed to move_group.

---

## Root Cause — Two YAML format worlds

MoveIt and ROS 2 use **different YAML formats** for parameters.

**MoveIt flat format** (what `moveit_controllers.yaml` looks like):
```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
moveit_simple_controller_manager:
  controller_names:
    - arm_controller
```

**ROS 2 parameter format** (what `rcl_yaml_param_parser` actually expects):
```yaml
/**:
  ros__parameters:
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
    moveit_simple_controller_manager:
      controller_names:
        - arm_controller
```

When `moveit_controllers.yaml` (flat format) is passed as a **file-path string** to
`Node(parameters=[...])` in a launch file, ROS 2's YAML parser looks for the
`ros__parameters:` wrapper and silently ignores the file if it's absent.

When it's loaded with `yaml.safe_load()` and passed as a **Python dict**, some versions of
`launch_ros` silently drop nested string-array values during type conversion.

In both cases `simple_controller_manager` initializes with 0 controllers because
`moveit_simple_controller_manager.controller_names` is never set on the ROS 2 parameter
server of the move_group node.

You can confirm this by checking at runtime:
```bash
ros2 param get /move_group moveit_simple_controller_manager.controller_names
# If this errors or returns empty, the parameters never loaded.
```

---

## Fix

Create a **second YAML file** in proper ROS 2 parameter format alongside the existing
`moveit_controllers.yaml`. Pass this new file (not the original) to the move_group Node.

### 1. Create `config/move_group_params.yaml`

```yaml
/**:
  ros__parameters:
    moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager
    moveit_simple_controller_manager:
      controller_names:
        - arm_controller
        - gripper_controller
      arm_controller:
        type: FollowJointTrajectory
        action_ns: follow_joint_trajectory
        default: true
        joints:
          - joint1
          - joint2
          - joint3
      gripper_controller:
        type: FollowJointTrajectory
        action_ns: follow_joint_trajectory
        default: true
        joints:
          - gripper_joint_left
          - gripper_joint_right
```

- `/**:` is a ROS 2 wildcard that matches all nodes — parameters apply to move_group.
- `action_ns: follow_joint_trajectory` tells MoveIt where the action server lives under
  each controller name (e.g. `/arm_controller/follow_joint_trajectory`).
- `default: true` marks the controller as auto-selected for execution.

### 2. Update `move_group.launch.py`

```python
pkg = get_package_share_directory("your_moveit_config")
move_group_params = os.path.join(pkg, "config", "move_group_params.yaml")

move_group_node = Node(
    package="moveit_ros_move_group",
    executable="move_group",
    output="screen",
    parameters=[
        moveit_config.to_dict(),   # robot desc, SRDF, kinematics, OMPL, etc.
        move_group_params,         # controller manager — proper ROS 2 format
        {"start_state_max_bounds_error": 0.05},
    ],
)
```

Keep `moveit_controllers.yaml` unchanged — `MoveItConfigsBuilder` still reads it
internally via `.trajectory_execution("config/moveit_controllers.yaml")`.

### 3. Verify after restarting move_group

```bash
ros2 param get /move_group moveit_controller_manager
ros2 param get /move_group moveit_simple_controller_manager.controller_names
```

Both should now return values. The move_group log should show:
```
[move_group] Returned 2 controllers in list
```

---

## Environment

- ROS 2 Jazzy
- MoveIt2 2.12.x (`moveit_configs_utils`, `moveit_simple_controller_manager`)
- Python launch files using `MoveItConfigsBuilder`
