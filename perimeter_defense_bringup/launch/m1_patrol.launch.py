"""M1: FOB-3 world, one UGV (lup1) with Nav2 + AMCL patrolling its route."""

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

    world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'world.launch.py')),
        launch_arguments={'world_variant': LaunchConfiguration('world_variant'),
                          'headless': LaunchConfiguration('headless')}.items(),
    )
    robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'robot.launch.py')),
        launch_arguments={'robot': 'lup1',
                          'enable_camera': LaunchConfiguration('enable_camera'),
                          'start_patrol': LaunchConfiguration('start_patrol')}.items(),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', os.path.join(bringup, 'rviz', 'fob3.rviz')],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
    )
    return LaunchDescription([
        DeclareLaunchArgument('world_variant', default_value='day'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('enable_camera', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('start_patrol', default_value='true'),
        world,
        # Give the Gazebo server a moment to load the world before spawning.
        TimerAction(period=3.0, actions=[robot]),
        rviz,
    ])
