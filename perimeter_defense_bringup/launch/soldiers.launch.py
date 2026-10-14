"""Bridges and controller of the own soldiers on foot (simulation side).

The soldier actors are added to the world file by world.launch.py
(actors:=soldier1:soldier,...), so that they are visible to the lidars.
"""

import os
import tempfile

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from perimeter_defense_world.geometry import load_layout


def _launch(context, *args, **kwargs):
    soldiers = [s for s in LaunchConfiguration('soldiers').perform(context).split(',') if s] \
        or list(load_layout().soldiers)
    intruders = [s for s in LaunchConfiguration('intruders').perform(context).split(',') if s]
    bridge = []
    for name in soldiers:
        bridge += [
            {'ros_topic_name': f'/actor/{name}/cmd_vel', 'gz_topic_name': f'/actor/{name}/cmd_vel',
             'ros_type_name': 'geometry_msgs/msg/Vector3', 'gz_type_name': 'gz.msgs.Vector3d',
             'direction': 'ROS_TO_GZ'},
            {'ros_topic_name': f'/sim/actors/{name}/pose', 'gz_topic_name': f'/actor/{name}/pose',
             'ros_type_name': 'geometry_msgs/msg/Pose', 'gz_type_name': 'gz.msgs.Pose',
             'direction': 'GZ_TO_ROS'},
        ]
    fd, bridge_file = tempfile.mkstemp(prefix='soldiers_bridge_', suffix='.yaml')
    with os.fdopen(fd, 'w') as f:
        yaml.safe_dump(bridge, f)
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='soldiers_bridge',
             parameters=[{'config_file': bridge_file, 'use_sim_time': True}], output='screen'),
        Node(package='perimeter_defense_sim', executable='soldier_controller', name='soldier_controller',
             parameters=[{'use_sim_time': True, 'soldiers': soldiers, 'intruders': intruders}],
             output='screen'),
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('soldiers', default_value='',
                              description='comma separated, empty: all soldiers in zones.yaml'),
        DeclareLaunchArgument('intruders', default_value='intruder1'),
        OpaqueFunction(function=_launch),
    ])
