"""Start the differential-drive world and its Twist interface."""

from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package = Path(get_package_share_directory('webots_simulation'))
    urdf = package / 'resource' / 'diff_drive_webots.urdf'
    # Separate from the existing PR2 simulator on port 1234.
    port = '1235'
    webots = WebotsLauncher(
        world=PathJoinSubstitution([LaunchConfiguration('world')]),
        gui=LaunchConfiguration('rendering'),
        port=port,
        ros2_supervisor=True,
    )
    driver = WebotsController(
        robot_name='DiffDrive', port=port,
        parameters=[{
            'robot_description': str(urdf),
            'use_sim_time': True,
        }],
    )

    # The installed supervisor launcher omits the port in its native IPC URL.
    supervisor = Node(
        package='webots_ros2_driver', executable='ros2_supervisor.py',
        namespace='Ros2Supervisor', output='screen',
        remappings=[('/Ros2Supervisor/clock', '/clock')],
        additional_env={
            'WEBOTS_CONTROLLER_URL': f'ipc://{port}/Ros2Supervisor',
            'WEBOTS_HOME': get_package_prefix('webots_ros2_driver'),
        },
    )
    state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': urdf.read_text(), 'use_sim_time': True}],
    )
    pointcloud_to_scan = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='base_laser_pointcloud_to_scan', output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_laser',
            'transform_tolerance': 0.1,
            # Project only the horizontal centre layer of the nine-layer lidar.
            'min_height': -0.03, 'max_height': 0.03,
            'angle_min': -3.14159265, 'angle_max': 3.14159265,
            'angle_increment': 0.003272492347489368,
            'scan_time': 0.05, 'range_min': 0.1, 'range_max': 6.0,
            'use_inf': True, 'inf_epsilon': 1.0,
        }],
        remappings=[('cloud_in', '/DiffDrive/base_laser/point_cloud'), ('scan', '/scan')],
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
        RegisterEventHandler(OnProcessExit(target_action=supervisor, on_exit=shutdown_on_exit)),
        webots,
        supervisor,
        state_publisher,
        driver,
        pointcloud_to_scan,
    ])
