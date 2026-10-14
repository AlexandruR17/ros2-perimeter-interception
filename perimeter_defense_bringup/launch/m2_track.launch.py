"""M2: one intruder through the north breach, fence sensor, lidar detector,
Kalman tracker, and one UGV (LUP-2 by default, east sector) leaving its patrol
to investigate the fence alert and follow the track."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bringup = get_package_share_directory('perimeter_defense_bringup')
    launch_dir = os.path.join(bringup, 'launch')
    viz_params = os.path.join(get_package_share_directory('perimeter_defense_viz'),
                              'config', 'viz.yaml')

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items())

    return LaunchDescription([
        DeclareLaunchArgument('world_variant', default_value='day'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('route', default_value='west', description='west or east'),
        DeclareLaunchArgument('robot', default_value='lup2', description='lup1, lup2 or lup3'),
        DeclareLaunchArgument('start_delay', default_value='15.0',
                              description='s before the intruder starts walking'),
        include('world.launch.py', {'world_variant': LaunchConfiguration('world_variant'),
                                    'headless': LaunchConfiguration('headless'),
                                    'actors': 'intruder1:north_breach'}),
        TimerAction(period=3.0, actions=[
            include('robot.launch.py', {'robot': LaunchConfiguration('robot'),
                                        'enable_camera': LaunchConfiguration('enable_camera')}),
            include('intruder.launch.py', {'name': 'intruder1', 'spawn': 'north_breach',
                                           'route': LaunchConfiguration('route'),
                                           'start_delay': LaunchConfiguration('start_delay')}),
        ]),
        include('defense.launch.py', {'robots': LaunchConfiguration('robot'), 'mode': 'follow'}),
        Node(package='perimeter_defense_viz', executable='layout_markers', name='layout_markers',
             parameters=[viz_params, {'aerial_variant': LaunchConfiguration('world_variant')}]),
        Node(package='perimeter_defense_viz', executable='fleet_markers', name='fleet_markers',
             parameters=[viz_params, {'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(bringup, 'rviz', 'fob3.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
