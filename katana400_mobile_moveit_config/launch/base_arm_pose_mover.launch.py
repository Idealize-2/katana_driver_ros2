from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "katana400_mobile",
            package_name="katana400_mobile_moveit_config",
        )
        .to_moveit_configs()
    )

    base_arm_pose_mover = Node(
        package="katana400_mobile_moveit_config",
        executable="base_arm_pose_mover",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {
                "x": LaunchConfiguration('x'),
                "y": LaunchConfiguration('y'),
                "z": LaunchConfiguration('z'),
                "pos_x": LaunchConfiguration('pos_x'),
                "pos_y": LaunchConfiguration('pos_y'),
                "heading": LaunchConfiguration('heading'),
                "use_sim_time": LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0.5'),
        DeclareLaunchArgument('y', default_value='0.5'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('pos_x', default_value='0.0',
                              description='Base x position to plan/execute from (no longer auto-derived via atan2)'),
        DeclareLaunchArgument('pos_y', default_value='0.0',
                              description='Base y position to plan/execute from (no longer auto-derived via atan2)'),
        DeclareLaunchArgument('heading', default_value='0.0',
                              description='Base yaw/heading (rad) to plan/execute from (no longer auto-derived via atan2)'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Set true when running with Gazebo'),
        base_arm_pose_mover,
    ])