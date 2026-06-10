from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "katana400_mobile",
            package_name="katana400_mobile_moveit_config",
        )
        .to_moveit_configs()
    )

    base_arm_pose_mover = Node(
        package="katana400_mobile_moveit_config",
        executable="base_arm_pose_mover",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([base_arm_pose_mover])
