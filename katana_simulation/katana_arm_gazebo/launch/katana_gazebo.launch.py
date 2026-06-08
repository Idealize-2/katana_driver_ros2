import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_katana_description = get_package_share_directory('katana_description')
    pkg_katana_arm_gazebo  = get_package_share_directory('katana_arm_gazebo')
    pkg_ros_gz_sim         = get_package_share_directory('ros_gz_sim')

    # GZ_SIM_RESOURCE_PATH: parent of katana_description so Gazebo can find
    # "katana_description/meshes/..." package:// URIs inside STL/DAE files.
    gz_resource_path = os.path.dirname(pkg_katana_description)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    full_resource_path = gz_resource_path + (':' + existing if existing else '')

    # ── Launch arguments ────────────────────────────────────────────────────
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_katana_arm_gazebo, 'worlds', 'grasp.world'),
        description='Path to the Gazebo world file',
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.0', description='Spawn Z')

    # ── Robot description (Gazebo URDF from katana_description) ─────────────
    # Pass use_gazebo:=true so transmissions.urdf.xacro and gazebo.urdf.xacro
    # are included (provides gz_ros2_control/GazeboSimSystem interface).
    xacro_file = os.path.join(
        pkg_katana_description, 'urdf',
        'katana_400_6m180_with_controlbox.urdf.xacro',
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_gazebo:=true']),
            value_type=str,
        )
    }

    # ── Nodes ───────────────────────────────────────────────────────────────
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': ['-r ', LaunchConfiguration('world')],
        }.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'katana',
            '-topic', 'robot_description',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
        output='screen',
    )

    # Clock bridge: Gazebo → ROS 2 (Harmonic uses gz.msgs.Clock)
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Controller spawners — staggered to let gz_ros2_control finish loading
    spawn_jsb = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )],
    )
    spawn_arm = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['katana_arm_controller'],
            output='screen',
        )],
    )
    spawn_gripper = TimerAction(
        period=7.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        )],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', full_resource_path),
        world_arg, x_arg, y_arg, z_arg,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        clock_bridge,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
    ])
