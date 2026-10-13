"""Start Gazebo with the FOB-3 world, the /clock bridge and the two global maps.

/map                  navigation map (solid fence), used by every costmap
/map_localization     AMCL map (fence posts only, the lidar sees through the mesh)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _gazebo(context, *args, **kwargs):
    world_pkg = get_package_share_directory('perimeter_defense_world')
    variant = LaunchConfiguration('world_variant').perform(context)
    headless = LaunchConfiguration('headless').perform(context).lower() == 'true'
    world = os.path.join(world_pkg, 'worlds', f'fob3_{variant}.sdf')
    gz_args = f'-r -s --headless-rendering {world}' if headless else f'-r {world}'
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )]


def generate_launch_description() -> LaunchDescription:
    world_pkg = get_package_share_directory('perimeter_defense_world')
    map_yaml = os.path.join(world_pkg, 'maps', 'fob3.yaml')
    loc_map_yaml = os.path.join(world_pkg, 'maps', 'fob3_localization.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('world_variant', default_value='day',
                              description='day or night'),
        DeclareLaunchArgument('headless', default_value='false',
                              description='run the Gazebo server without GUI'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(world_pkg, 'models')),
        OpaqueFunction(function=_gazebo),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen',
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True,
                         'topic_name': 'map', 'frame_id': 'map'}],
            output='screen',
        ),
        Node(
            package='nav2_map_server', executable='map_server', name='map_server_localization',
            parameters=[{'yaml_filename': loc_map_yaml, 'use_sim_time': True,
                         'topic_name': 'map_localization', 'frame_id': 'map'}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_map',
            parameters=[{'use_sim_time': True, 'autostart': True,
                         'node_names': ['map_server', 'map_server_localization']}],
            output='screen',
        ),
    ])
