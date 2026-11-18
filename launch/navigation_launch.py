"""Stage 1: global planning only; simulation and AMCL run separately."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_navigation(context):
    params = Path(LaunchConfiguration('params_file').perform(context)).expanduser().resolve()
    if not params.is_file():
        raise FileNotFoundError(f'Navigation parameters do not exist: {params}')
    nodes = [
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[str(params)]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen', parameters=[str(params)]),
    ]

    def shutdown_on_exit(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='Navigation process exited'))]

    handlers = [RegisterEventHandler(OnProcessExit(target_action=node, on_exit=shutdown_on_exit))
                for node in nodes]
    return [*handlers, *nodes]


def generate_launch_description():
    package = Path(get_package_share_directory('webots_simulation'))
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=str(package / 'config' / 'navigation.yaml')),
        OpaqueFunction(function=launch_navigation),
    ])
