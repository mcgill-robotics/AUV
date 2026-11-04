import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_sim = DeclareLaunchArgument(
        "sim",
        default_value="false",
        description="true to run on simulation time (/clock)",
    )
    sim = LaunchConfiguration("sim")

    config = os.path.join(
        get_package_share_directory('controls'), 'params', 'navigation_server.yaml'
    )

    return LaunchDescription([
        declare_sim,
        Node(
            package='controls',
            executable='navigation_server.py',
            name='navigation_server',
            parameters=[config, {"use_sim_time": sim}],
            output='screen',
        ),
    ])
