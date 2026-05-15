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
        .trajectory_execution("config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    pkg = get_package_share_directory("katana400_moveit_config")
    move_group_params = os.path.join(pkg, "config", "move_group_params.yaml")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_params,
            {"start_state_max_bounds_error": 0.05},
        ],
    )

    return LaunchDescription([move_group_node])
