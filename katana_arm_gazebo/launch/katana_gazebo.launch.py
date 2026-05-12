import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_katana_description = get_package_share_directory('katana_description')
    pkg_katana_arm_gazebo = get_package_share_directory('katana_arm_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Arguments
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x', default='0.0')
    y = LaunchConfiguration('y', default='0.0')
    z = LaunchConfiguration('z', default='0.0')

    # Xacro to URDF
    xacro_file = os.path.join(pkg_katana_description, 'urdf', 'katana_450_6m90a.urdf.xacro')
    robot_description = {'robot_description': Command(['xacro ', xacro_file])}

    # Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    # Gazebo Sim
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r empty.sdf']}.items()
    )

    # Spawn Robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'katana',
                   '-topic', 'robot_description',
                   '-x', x, '-y', y, '-z', z],
        output='screen'
    )

    # ros2_control Spawners
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['katana_arm_controller'],
        output='screen'
    )

    # ROS-GZ Bridge for Clock
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='empty.sdf', description='Gazebo World'),
        node_robot_state_publisher,
        gz_sim,
        spawn,
        bridge,
        joint_state_broadcaster,
        arm_controller
    ])
