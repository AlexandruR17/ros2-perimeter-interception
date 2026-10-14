"""Bridge and controller of one scripted intruder (simulation side).

The actor itself is added to the world file by world.launch.py (argument
actors:=name:spawn), so that it is visible to the lidars from the start.
"""

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context, *args, **kwargs):
    g = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    name, spawn, route = g('name'), g('spawn'), g('route')
    bridge = [
        {'ros_topic_name': f'/actor/{name}/cmd_vel', 'gz_topic_name': f'/actor/{name}/cmd_vel',
         'ros_type_name': 'geometry_msgs/msg/Vector3', 'gz_type_name': 'gz.msgs.Vector3d',
         'direction': 'ROS_TO_GZ'},
        {'ros_topic_name': f'/sim/actors/{name}/pose', 'gz_topic_name': f'/actor/{name}/pose',
         'ros_type_name': 'geometry_msgs/msg/Pose', 'gz_type_name': 'gz.msgs.Pose',
         'direction': 'GZ_TO_ROS'},
    ]
    fd, bridge_file = tempfile.mkstemp(prefix=f'{name}_bridge_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(bridge, f)

    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name=f'{name}_bridge',
             parameters=[{'config_file': bridge_file, 'use_sim_time': True}], output='screen'),
        Node(package='perimeter_defense_sim', executable='intruder_controller',
             name=f'{name}_controller',
             parameters=[{'use_sim_time': True, 'actor': name, 'spawn': spawn, 'route': route,
                          'start_delay': float(g('start_delay')),
                          'run_when_detected': g('run_when_detected').lower() == 'true'}],
             output='screen'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('name', default_value='intruder1'),
        DeclareLaunchArgument('spawn', default_value='north_breach'),
        DeclareLaunchArgument('route', default_value='west', description='west or east'),
        DeclareLaunchArgument('start_delay', default_value='10.0'),
        DeclareLaunchArgument('run_when_detected', default_value='true'),
        OpaqueFunction(function=_launch),
    ])
