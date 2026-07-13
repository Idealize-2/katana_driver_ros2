"""
Full-system Gazebo + MoveIt 2 launch for the Katana 400 6M180 on a mobile base.

Usage:
    ros2 launch katana400_mobile_moveit_config gazebo_moveit.launch.py
    ros2 launch katana400_mobile_moveit_config gazebo_moveit.launch.py world:=/path/to/my.world

Control the arm via MoveIt interactive marker in RViz (Plan → Execute).
Drive the base separately:
    ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}}}"
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_moveit    = get_package_share_directory('katana400_mobile_moveit_config')
    pkg_arm_gazebo = get_package_share_directory('katana_arm_gazebo')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_arm_gazebo, 'worlds', 'grasp.world'),
        description='Gazebo world file',
    )

    # Gazebo layer: RSP + Gazebo sim + ros2_control + all controllers
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'mobile_gazebo.launch.py')
        ),
        launch_arguments={'world': LaunchConfiguration('world')}.items(),
    )

    # MoveIt reads robot geometry from the MoveIt URDF (katana400_mobile.urdf.xacro via
    # MoveItConfigsBuilder). It does not load any hardware plugin itself — Gazebo is the hw.
    moveit_config = (
        MoveItConfigsBuilder(
            'katana400_mobile',
            package_name='katana400_mobile_moveit_config',
        )
        .trajectory_execution('config/moveit_controllers.yaml')
        .to_moveit_configs()
    )

    move_group_params = os.path.join(pkg_moveit, 'config', 'move_group_params.yaml')

    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            move_group_params,
            {'start_state_max_bounds_error': 0.05, 'use_sim_time': True},
        ],
    )

    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[moveit_config.to_dict(), {'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        gazebo_launch,
        move_group_node,
        rviz_node,
    ])
