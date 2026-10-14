"""Perimeter defense nodes (individual contribution)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    params = os.path.join(get_package_share_directory('perimeter_defense'), 'config',
                          'perimeter_defense.yaml')
    robots = [r for r in LaunchConfiguration('robots').perform(context).split(',') if r]
    mode = LaunchConfiguration('mode').perform(context)
    common = {'use_sim_time': True}
    node = lambda exe, extra=None: Node(   # noqa: E731
        package='perimeter_defense', executable=exe, name=exe,
        parameters=[params, common, extra or {}], output='screen')
    return [
        node('fence_sensor'),
        node('person_detector', {'robots': robots}),
        node('tracker'),
        node('iff'),
        node('interceptor_allocator', {'robots': robots, 'mode': mode}),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robots', default_value='lup1', description='comma separated'),
        DeclareLaunchArgument('mode', default_value='follow'),
        OpaqueFunction(function=_launch),
    ])
