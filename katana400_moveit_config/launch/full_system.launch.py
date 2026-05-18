from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("katana400_moveit_config")

    connection_type = LaunchConfiguration('connection_type')
    ip_address      = LaunchConfiguration('ip_address')
    tcp_port        = LaunchConfiguration('tcp_port')
    calibrate       = LaunchConfiguration('calibrate_on_startup')

    def launch(name, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", name)),
            launch_arguments=(args or {}).items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'connection_type', default_value='serial',
            description="'tcp' or 'serial'"),
        DeclareLaunchArgument(
            'ip_address', default_value='192.168.1.1',
            description='Arm IP address (TCP mode)'),
        DeclareLaunchArgument(
            'tcp_port', default_value='5566',
            description='KNI TCP port (TCP mode)'),
        DeclareLaunchArgument(
            'calibrate_on_startup', default_value='true',
            description="'true' to run calibration on first activate"),

        launch("real_hardware.launch.py", {
            'connection_type':      connection_type,
            'ip_address':           ip_address,
            'tcp_port':             tcp_port,
            'calibrate_on_startup': calibrate,
        }),
        launch("move_group.launch.py"),
        launch("moveit_rviz.launch.py"),
    ])
