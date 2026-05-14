"""
real_hardware.launch.py  (katana400_moveit_config)

Minimal ros2_control stack for the physical Katana 400 6M180 arm — no MoveIt, no simulation.
Starts: robot_state_publisher, controller_manager, and all three controllers.

Launch arguments (all optional — defaults suit a serial-connected Katana 400 6M180):
  connection_type       'serial' or 'tcp'
  ip_address            arm IP address (TCP mode)
  tcp_port              KNI port, default 5566 (TCP mode)
  serial_port           /dev/ttyS<N> index (serial mode, e.g. 0 for /dev/ttyS0)
  serial_baud           baud rate (serial mode, default 57600)
  config_file           absolute path to the KNI .cfg for your arm variant
  calibrate_on_startup  'true'/'false' — run full calibration on first activate

Example — serial connection:
  ros2 launch katana400_moveit_config real_hardware.launch.py \\
      connection_type:=serial serial_port:=0

Example — TCP connection:
  ros2 launch katana400_moveit_config real_hardware.launch.py \\
      connection_type:=tcp ip_address:=192.168.1.1

Example — skip calibration (arm already homed this power cycle):
  ros2 launch katana400_moveit_config real_hardware.launch.py \\
      calibrate_on_startup:=false
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_moveit = get_package_share_directory('katana400_moveit_config')
    pkg_kni    = get_package_share_directory('kni')

    # Resolve launch arg values
    connection_type = LaunchConfiguration('connection_type').perform(context)
    ip_address      = LaunchConfiguration('ip_address').perform(context)
    tcp_port        = LaunchConfiguration('tcp_port').perform(context)
    serial_port     = LaunchConfiguration('serial_port').perform(context)
    serial_baud     = LaunchConfiguration('serial_baud').perform(context)
    config_file     = LaunchConfiguration('config_file').perform(context)
    calibrate       = LaunchConfiguration('calibrate_on_startup').perform(context)

    # Fall back to the installed katana6M180.cfg if nothing was specified
    if not config_file:
        config_file = os.path.join(
            pkg_kni, 'KNI_4.3.0', 'configfiles400', 'katana6M180.cfg')

    initial_positions_file = os.path.join(
        pkg_moveit, 'config', 'initial_positions.yaml')

    xacro_file = os.path.join(
        pkg_moveit, 'config', 'katana_400_6m180_with_controlbox.urdf.xacro')

    robot_description_xml = xacro.process_file(
        xacro_file,
        mappings={
            'initial_positions_file': initial_positions_file,
            'connection_type':        connection_type,
            'ip_address':             ip_address,
            'tcp_port':               tcp_port,
            'serial_port':            serial_port,
            'serial_baud':            serial_baud,
            'config_file':            config_file,
            'calibrate_on_startup':   calibrate,
        },
    ).toxml()

    robot_description = {'robot_description': robot_description_xml}

    controllers_yaml = os.path.join(
        pkg_moveit, 'config', 'ros2_controllers.yaml')

    # robot_state_publisher publishes /robot_description topic (latched).
    # ros2_control_node subscribes to that topic, so RSP must start first.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Pass robot_description directly so the controller_manager does not rely
    # on the /robot_description topic subscription (which can miss the latched
    # message in Jazzy when there is a QoS mismatch).
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen',
        parameters=[robot_description, controllers_yaml],
    )

    def spawner(name, delay=0.0):
        node = Node(
            package='controller_manager',
            executable='spawner',
            arguments=[name, '--controller-manager-timeout', '60'],
            output='screen',
        )
        return TimerAction(period=delay, actions=[node]) if delay > 0.0 else node

    return [
        robot_state_publisher,
        controller_manager,
        # Delay all spawners so the CM and pluginlib caches have time to
        # initialise before load_controller is called — prevents a JTC
        # on_init() segfault that occurs on fresh boot.
        spawner('joint_state_broadcaster', delay=5.0),
        spawner('arm_controller',          delay=5.0),
        spawner('gripper_controller',      delay=5.0),
    ]


def generate_launch_description():
    pkg_kni = get_package_share_directory('kni')
    default_cfg = os.path.join(
        pkg_kni, 'KNI_4.3.0', 'configfiles400', 'katana6M180.cfg')

    return LaunchDescription([
        DeclareLaunchArgument(
            'connection_type', default_value='serial',
            description="'tcp' or 'serial'"),
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.1.1',
            description='Arm controller IP address (TCP mode)'),
        DeclareLaunchArgument(
            'tcp_port', default_value='5566',
            description='KNI TCP port (TCP mode)'),
        DeclareLaunchArgument(
            'serial_port', default_value='0',
            description='Index N for /dev/ttyS<N> (serial mode)'),
        DeclareLaunchArgument(
            'serial_baud', default_value='57600',
            description='Serial baud rate (serial mode)'),
        DeclareLaunchArgument(
            'config_file', default_value=default_cfg,
            description='Absolute path to KNI .cfg file for your arm variant'),
        DeclareLaunchArgument(
            'calibrate_on_startup', default_value='true',
            description="'true' to run full calibration sequence on first activate"),

        OpaqueFunction(function=launch_setup),
    ])
