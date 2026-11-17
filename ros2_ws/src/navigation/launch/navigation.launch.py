import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_share = get_package_share_directory('navigation')
    nav2_launch = os.path.join(package_share, 'launch', 'nav2.launch.py')
    server_launch = os.path.join(
        package_share, 'launch', 'navigation_server.launch.py'
    )

    sim = LaunchConfiguration('sim')
    params_file = LaunchConfiguration('params_file')
    default_params_file = os.path.join(
        package_share, 'params', 'nav2_params.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Use simulation time when true',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Nav2 parameter file',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(server_launch)
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'sim': sim,
                'params_file': params_file,
            }.items(),
        ),
    ])
