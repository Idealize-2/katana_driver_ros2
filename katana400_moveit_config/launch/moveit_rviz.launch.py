from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "katana_400_6m180_with_controlbox",
            package_name="katana400_moveit_config",
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("katana400_moveit_config"),
        "config",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription([rviz_node])
