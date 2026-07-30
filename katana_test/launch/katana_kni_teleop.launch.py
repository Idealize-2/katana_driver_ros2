"""
katana_kni_teleop.launch.py
Launch the low-level KNI keyboard teleop diagnostic tool.

This node talks DIRECTLY to the hardware via the KNI SDK (TCP).
It does NOT go through ros2_control.  Use it only for raw hardware
diagnostics when ros2_control / MoveIt is NOT running.

Usage:
    ros2 launch katana_test katana_kni_teleop.launch.py
    ros2 launch katana_test katana_kni_teleop.launch.py ip:=192.168.1.1 port:=5566
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ip_arg = DeclareLaunchArgument(
        "ip",
        default_value="192.168.1.1",
        description="IP address of the Katana 400 controller",
    )
    port_arg = DeclareLaunchArgument(
        "port",
        default_value="5566",
        description="TCP port of the Katana 400 controller",
    )

    kni_teleop_node = Node(
        package="katana_test",
        executable="katana_kni_teleop",
        name="katana_kni_teleop",
        output="screen",
        # Pass ip and port as positional arguments (argv[1], argv[2])
        arguments=[LaunchConfiguration("ip"), LaunchConfiguration("port")],
        # Keep stdin attached so keyboard input reaches the node
        emulate_tty=True,
        prefix="",
    )

    return LaunchDescription([ip_arg, port_arg, kni_teleop_node])
