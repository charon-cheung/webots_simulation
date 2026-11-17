"""Compose the differential simulation and localization in one launch session."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package = Path(get_package_share_directory('webots_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(package / 'worlds' / 'hall.wbt')),
        DeclareLaunchArgument('map', default_value=str(package / 'maps' / 'hall.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=str(package / 'config' / 'localization_params.yaml')),
        DeclareLaunchArgument('rendering', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch' / 'localization_launch.py')),
            launch_arguments={name: LaunchConfiguration(name)
                              for name in ('map', 'params_file', 'rviz')}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(package / 'launch' / 'simulation_launch.py')),
            launch_arguments={name: LaunchConfiguration(name)
                              for name in ('world', 'rendering')}.items(),
        ),
    ])
