"""Start the differential-drive world and its Twist interface."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package = Path(get_package_share_directory('webots_simulation'))
    # Separate from the existing PR2 simulator on port 1234.
    port = '1235'
    webots = WebotsLauncher(
        world=PathJoinSubstitution([LaunchConfiguration('world')]),
        gui=LaunchConfiguration('rendering'),
        port=port,
    )
    driver = WebotsController(
        robot_name='DiffDrive', port=port,
        parameters=[{
            'robot_description': str(package / 'resource' / 'diff_drive_webots.urdf'),
        }],
    )

    def shutdown_on_exit(event, context):
        if context.is_shutdown:
            return []
        return [EmitEvent(event=Shutdown(reason='Simulation process exited'))]

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=str(package / 'worlds' / 'hall.wbt')),
        DeclareLaunchArgument('rendering', default_value='true'),
        RegisterEventHandler(OnProcessExit(target_action=webots, on_exit=shutdown_on_exit)),
        RegisterEventHandler(OnProcessExit(target_action=driver, on_exit=shutdown_on_exit)),
        webots,
        driver,
    ])
