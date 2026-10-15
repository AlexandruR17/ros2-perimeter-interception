"""C2 web dashboard: rosbridge websocket, the C2 state aggregator and the web server.

Open http://localhost:8080 in a browser (Windows browsers reach WSL on
localhost as well).
"""

import os

from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    g = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    robots = [r for r in g('robots').split(',') if r]
    params = os.path.join(get_package_share_directory('perimeter_defense_dashboard'), 'config',
                          'dashboard.yaml')
    try:
        rosbridge = get_package_share_directory('rosbridge_server')
    except PackageNotFoundError:
        return [LogInfo(msg='rosbridge_server is not installed, the dashboard has no live data: '
                            'sudo apt install ros-jazzy-rosbridge-suite')]
    return [
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(os.path.join(rosbridge, 'launch', 'rosbridge_websocket_launch.xml')),
            launch_arguments={'port': g('rosbridge_port')}.items()),
        Node(package='perimeter_defense_dashboard', executable='c2_state', name='c2_state',
             parameters=[params, {'use_sim_time': True, 'robots': robots}], output='screen'),
        Node(package='perimeter_defense_dashboard', executable='dashboard_server', name='dashboard_server',
             parameters=[params, {'robots': robots, 'routes': g('routes'), 'port': int(g('port')),
                                  'rosbridge_port': int(g('rosbridge_port'))}], output='screen'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='lup1,lup2,lup3'),
        DeclareLaunchArgument('port', default_value='8080', description='web page port'),
        DeclareLaunchArgument('routes', default_value='', description="patrol routes drawn, '' = robots'"),
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        OpaqueFunction(function=_launch),
    ])
