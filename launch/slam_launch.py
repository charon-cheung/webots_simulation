"""Start online mapping after simulation_launch.py; drive with keyboard teleop."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_rviz(context):
    config = Path.home() / 'configs' / 'sim.rviz'
    if not config.is_file():
        raise FileNotFoundError(f'RViz configuration does not exist: {config}')
    return [Node(
        package='rviz2', executable='rviz2', name='rviz2_slam', output='screen',
        arguments=['-d', str(config)], parameters=[{'use_sim_time': True}],
    )]


def generate_launch_description():
    package = Path(get_package_share_directory('webots_simulation'))
    slam_package = Path(get_package_share_directory('slam_toolbox'))
    return LaunchDescription([
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=str(package / 'config' / 'mapper_params_online_async.yaml'),
            description='Parameters for online async mapping.'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open ~/configs/sim.rviz.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(slam_package / 'launch' / 'online_async_launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'slam_params_file': LaunchConfiguration('slam_params_file'),
                'autostart': 'true',
                'use_lifecycle_manager': 'false',
            }.items(),
        ),
        OpaqueFunction(function=launch_rviz, condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
