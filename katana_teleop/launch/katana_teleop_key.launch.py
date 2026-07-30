"""
katana_teleop_key.launch.py
Launch the Katana ros2_control keyboard teleop.

Prerequisites (must be running first):
  - joint_state_broadcaster
  - arm_controller        (JointTrajectoryController)
  - gripper_controller    (JointTrajectoryController)
  - katana_hw/set_motors_enabled service  (from the hardware interface)

Usage:
    ros2 launch katana_teleop katana_teleop_key.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    teleop_node = Node(
        package="katana_teleop",
        executable="katana_teleop_key",
        name="katana_teleop",
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([teleop_node])
