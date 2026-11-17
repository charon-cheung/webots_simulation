"""Load a saved map and localize the running differential-drive robot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_localization(context):
    map_path = Path(LaunchConfiguration('map').perform(context)).expanduser().resolve()
    params = Path(LaunchConfiguration('params_file').perform(context)).expanduser().resolve()
    for path in (map_path, params):
        if not path.is_file():
            raise FileNotFoundError(f'Localization input does not exist: {path}')
    nodes = [
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[str(params), {'yaml_filename': str(map_path)}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen', parameters=[str(params)]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen', parameters=[str(params)]),
    ]

    def shutdown_on_exit(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='Localization process exited'))]

    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=shutdown_on_exit))
                for node in nodes]
    if IfCondition(LaunchConfiguration('rviz')).evaluate(context):
        config = Path.home() / 'configs' / 'sim.rviz'
        if not config.is_file():
            raise FileNotFoundError(f'RViz configuration does not exist: {config}')
        nodes.append(Node(package='rviz2', executable='rviz2', name='rviz2_localization',
                          output='screen', arguments=['-d', str(config)],
                          parameters=[{'use_sim_time': True}]))
    return [*handlers, *nodes]


def generate_launch_description():
    package = Path(get_package_share_directory('webots_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=str(package / 'maps' / 'hall.yaml')),
        DeclareLaunchArgument('params_file',
                              default_value=str(package / 'config' / 'localization_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=launch_localization),
    ])
