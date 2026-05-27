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

    moveit_pose_mover = Node(
        package="katana400_moveit_config",
        executable="moveit_pose_mover",
        output="screen",
        # Pass robot description + kinematics so MoveGroupInterface can
        # initialise without waiting for the parameter server topics.
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([moveit_pose_mover])
