"""
Gazebo Harmonic simulation launch for the Katana 400 6M180 on a mobile base.

Usage:
    ros2 launch katana400_mobile_moveit_config mobile_gazebo.launch.py
    ros2 launch katana400_mobile_moveit_config mobile_gazebo.launch.py world:=/path/to/my.world

Drive the base with:
    ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
        "{header: {frame_id: base_footprint}, twist: {linear: {x: 0.2}}}"
"""
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
    pkg_katana_description        = get_package_share_directory('katana_description')
    pkg_mobile_base_description   = get_package_share_directory('mobile_base_description')
    pkg_mobile_katana_description = get_package_share_directory('mobile_katana_description')
    pkg_katana_arm_gazebo         = get_package_share_directory('katana_arm_gazebo')
    pkg_ros_gz_sim                = get_package_share_directory('ros_gz_sim')
    pkg_mobile_moveit_config      = get_package_share_directory('katana400_mobile_moveit_config')

    # Gazebo must find mesh files via package:// URIs — add parent dirs to resource path.
    gz_resource_path = ':'.join([
        os.path.dirname(pkg_katana_description),
        os.path.dirname(pkg_mobile_base_description),
    ])
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    full_resource_path = gz_resource_path + (':' + existing if existing else '')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_katana_arm_gazebo, 'worlds', 'grasp.world'),
        description='Path to the Gazebo world file',
    )

    # use_gazebo:=true  → activates gz_ros2_control/GazeboSimSystem for arm + wheels
    # use_gazebo_plugin:=false is the default in katana400_mobile.urdf.xacro so the arm's
    #   own gazebo.urdf.xacro (arm-only plugin) is suppressed; the combined plugin lives in
    #   mobile_katana_gazebo.urdf.xacro.
    xacro_file = os.path.join(
        pkg_mobile_katana_description, 'urdf', 'katana400_mobile.urdf.xacro'
    )
    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file, ' use_gazebo:=true']),
            value_type=str,
        )
    }

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
        arguments=['-name', 'katana_mobile', '-topic', 'robot_description', 'z', '0'],
        output='screen',
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    # Staggered spawners — gz_ros2_control needs time to start the controller_manager.
    # arm must load before gripper (GripperActionController heap-corruption issue).
    spawn_jsb = TimerAction(
        period=15.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        )],
    )
    spawn_arm = TimerAction(
        period=17.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['arm_controller'],
            output='screen',
        )],
    )
    spawn_gripper = TimerAction(
        period=19.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        )],
    )
    spawn_drive = TimerAction(
        period=21.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            arguments=['diff_drive_controller'],
            output='screen',
        )],
    )

    nav2_params = os.path.join(pkg_mobile_moveit_config, 'config', 'nav2_params.yaml')

    # Bridge node: MoveIt FollowJointTrajectory → Nav2 NavigateToPose pass-through.
    # Nav2 handles closed-loop odom correction; this node just extracts the final goal.
    mobile_base_controller = Node(
        package='katana400_mobile_moveit_config',
        executable='mobile_base_controller.py',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Nav2 nodes — start after diff_drive_controller is up and publishing odom TF (t=21s).
    # lifecycle_manager activates the other three automatically (autostart: true).
    # controller_server remapped: /cmd_vel → /diff_drive_controller/cmd_vel (TwistStamped).
    nav2_nodes = TimerAction(
        period=25.0,
        actions=[
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                parameters=[nav2_params],
                remappings=[('/odom', '/diff_drive_controller/odom')],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                parameters=[nav2_params],
                remappings=[
                    ('/odom', '/diff_drive_controller/odom'),
                    ('/cmd_vel', '/diff_drive_controller/cmd_vel'),
                ],
            ),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                parameters=[nav2_params],
                remappings=[
                    ('/odom', '/diff_drive_controller/odom'),
                    ('/cmd_vel', '/diff_drive_controller/cmd_vel'),
                ],
            ),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                parameters=[nav2_params],
                remappings=[('/odom', '/diff_drive_controller/odom')],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[nav2_params],
            ),
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', full_resource_path),
        world_arg,
        robot_state_publisher,
        gz_sim,
        spawn_robot,
        clock_bridge,
        spawn_jsb,
        spawn_arm,
        spawn_gripper,
        spawn_drive,
        mobile_base_controller,
        nav2_nodes,
    ])
