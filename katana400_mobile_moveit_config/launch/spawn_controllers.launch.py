from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def _spawner(name, delay):
    return TimerAction(
        period=float(delay),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[name, "--controller-manager-timeout", "60"],
                output="screen",
            )
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        _spawner("joint_state_broadcaster", delay=20),
        _spawner("arm_controller",          delay=22),
        _spawner("gripper_controller",      delay=35),
    ])
