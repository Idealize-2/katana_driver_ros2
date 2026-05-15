from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "katana_400_6m180_with_controlbox",
            package_name="katana400_moveit_config",
        )
        .to_moveit_configs()
    )

    pose_runner = Node(
        package="katana_tutorials",
        executable="pose_runner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([pose_runner])
