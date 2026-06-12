import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('katana400_mobile_moveit_config')

    initial_positions_file = os.path.join(pkg, 'config', 'initial_positions.yaml')
    xacro_file = os.path.join(pkg, 'config', 'katana400_mobile.urdf.xacro')
    controllers_yaml = os.path.join(pkg, 'config', 'ros2_controllers.yaml')

    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={'initial_positions_file': initial_positions_file},
    ).toxml()
    robot_description = {'robot_description': robot_description_xml}

    def _spawner(name, delay):
        return TimerAction(
            period=float(delay),
            actions=[Node(
                package='controller_manager',
                executable='spawner',
                arguments=[name, '--controller-manager-timeout', '60'],
                output='screen',
            )],
        )

    launch_dir = os.path.join(pkg, 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('db',       default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'static_virtual_joint_tfs.launch.py')),
        ),
        # RSP publishes /robot_description + /tf for joint states
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'rsp.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'move_group.launch.py')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_dir, 'moveit_rviz.launch.py')),
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),

        # Pass robot_description directly — avoids the Jazzy QoS mismatch on
        # the /robot_description topic that leaves CM with no hardware interface,
        # causing JointTrajectoryController::on_init() to segfault at vtable 0xc8.
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            output='screen',
            parameters=[robot_description, controllers_yaml],
        ),

        # arm must load before gripper: GripperActionController (deprecated) corrupts
        # CM heap state during init; JTC segfaults at vtable 0xc8 when it loads after.
        _spawner('joint_state_broadcaster', delay=20),
        _spawner('arm_controller',          delay=22),
        _spawner('gripper_controller',      delay=35),
    ])
