"""Spawn one LUP UGV with its sensors bridge, odometry drift, AMCL and Nav2.

All nodes run in the robot namespace (/lup1, ...). TF frames are prefixed
with the namespace (lup1/base_link, lup1/odom) and published on the global
/tf, so every robot localises in the shared 'map' frame.
"""

import math
import os
import tempfile

import xacro
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from perimeter_defense_world.geometry import load_layout

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator']


def _load_yaml(path: str) -> dict:
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def _write_tmp(prefix: str, text: str, suffix: str = '.yaml') -> str:
    fd, path = tempfile.mkstemp(prefix=prefix, suffix=suffix)
    with os.fdopen(fd, 'w') as f:
        f.write(text)
    return path


def _bridge_config(robot: str, camera: bool) -> str:
    entries = [
        ('scan', 'sensor_msgs/msg/LaserScan', 'gz.msgs.LaserScan', 'GZ_TO_ROS'),
        ('odom_wheel', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry', 'GZ_TO_ROS'),
        ('joint_states', 'sensor_msgs/msg/JointState', 'gz.msgs.Model', 'GZ_TO_ROS'),
        ('cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist', 'ROS_TO_GZ'),
    ]
    if camera:
        entries += [
            ('camera/image_raw', 'sensor_msgs/msg/Image', 'gz.msgs.Image', 'GZ_TO_ROS'),
            ('camera/camera_info', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo', 'GZ_TO_ROS'),
        ]
    cfg = [{
        'ros_topic_name': f'/{robot}/{topic}',
        'gz_topic_name': f'/{robot}/{topic}',
        'ros_type_name': ros_type,
        'gz_type_name': gz_type,
        'direction': direction,
    } for topic, ros_type, gz_type, direction in entries]
    # Ground truth, for the scenario scripts and the evaluation only.
    cfg.append({'ros_topic_name': f'/sim/ground_truth/{robot}/pose',
                'gz_topic_name': f'/model/{robot}/pose',
                'ros_type_name': 'geometry_msgs/msg/Pose', 'gz_type_name': 'gz.msgs.Pose',
                'direction': 'GZ_TO_ROS'})
    return _write_tmp(f'{robot}_bridge_', yaml.safe_dump(cfg))


def _launch_robot(context, *args, **kwargs):
    robot = LaunchConfiguration('robot').perform(context)
    camera = LaunchConfiguration('enable_camera').perform(context).lower() == 'true'
    start_patrol = LaunchConfiguration('start_patrol').perform(context).lower() == 'true'
    zones_file = LaunchConfiguration('zones_file').perform(context)
    patrol_route = LaunchConfiguration('patrol_route').perform(context) or robot

    desc_pkg = get_package_share_directory('perimeter_defense_description')
    bringup_pkg = get_package_share_directory('perimeter_defense_bringup')
    fleet = _load_yaml(os.path.join(bringup_pkg, 'config', 'fleet.yaml'))
    hw = _load_yaml(os.path.join(desc_pkg, 'config', 'lup_ugv.yaml'))

    # Spawn at the first waypoint of the robot's patrol route.
    layout = load_layout(zones_file or None)
    route = layout.patrol_routes[robot]
    x, y = route[0]
    yaw = math.atan2(route[1][1] - y, route[1][0] - x)

    # Robot description.
    mappings = {k: str(v).lower() if isinstance(v, bool) else str(v) for k, v in hw.items()}
    mappings.update({'ns': robot, 'enable_camera': 'true' if camera else 'false'})
    urdf = xacro.process_file(os.path.join(desc_pkg, 'urdf', 'lup_ugv.urdf.xacro'),
                              mappings=mappings).toxml()

    # Nav2 parameters with the robot name substituted, nested under the namespace
    # so the node names match (/lup1/amcl, /lup1/local_costmap/local_costmap, ...).
    with open(os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'), 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f.read().replace('ROBOT', robot))
    nav2_params = _write_tmp(f'{robot}_nav2_', yaml.safe_dump({robot: params}))

    amcl_pose = {'initial_pose.x': x, 'initial_pose.y': y, 'initial_pose.yaw': yaw}
    sim_time = {'use_sim_time': True}

    actions = [
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=robot,
            parameters=[{'robot_description': urdf, 'frame_prefix': f'{robot}/'}, sim_time],
            output='screen',
        ),
        Node(
            package='ros_gz_sim', executable='create', name='spawn', namespace=robot,
            arguments=['-world', 'fob3', '-name', robot, '-topic', f'/{robot}/robot_description',
                       '-x', f'{x:.3f}', '-y', f'{y:.3f}', '-z', '0.05', '-Y', f'{yaw:.4f}'],
            output='screen',
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='bridge',
            namespace=robot,
            parameters=[{'config_file': _bridge_config(robot, camera)}, sim_time],
            output='screen',
        ),
        Node(
            package='perimeter_defense_sim', executable='odom_noise', namespace=robot,
            parameters=[fleet['odometry_drift'], sim_time,
                        {'odom_frame': f'{robot}/odom', 'base_frame': f'{robot}/base_link'}],
            output='screen',
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', namespace=robot,
            parameters=[nav2_params, amcl_pose], output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', namespace=robot,
            parameters=[sim_time, {'autostart': True, 'node_names': ['amcl']}],
            output='screen',
        ),
        Node(
            package='nav2_controller', executable='controller_server', namespace=robot,
            parameters=[nav2_params], output='screen',
        ),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            namespace=robot, parameters=[nav2_params], output='screen',
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            namespace=robot, parameters=[nav2_params], output='screen',
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            namespace=robot, parameters=[nav2_params], output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', namespace=robot,
            parameters=[sim_time, {'autostart': True, 'node_names': NAV2_NODES,
                                   'bond_timeout': 10.0}],
            output='screen',
        ),
        # Own position broadcast, so the other robots' lidars do not take it for an intruder.
        Node(
            package='perimeter_defense', executable='robot_transponder', namespace=robot,
            parameters=[sim_time, {'robot': robot}], output='screen',
        ),
    ]
    if start_patrol:
        actions.append(Node(
            package='perimeter_defense_sim', executable='patrol', namespace=robot,
            parameters=[sim_time, {'route_name': patrol_route, 'zones_file': zones_file,
                                   'patrol_speed': fleet['patrol']['patrol_speed'],
                                   'retry_delay': fleet['patrol']['retry_delay']}],
            output='screen',
        ))
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('robot', default_value='lup1', description='robot namespace'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('start_patrol', default_value='true'),
        DeclareLaunchArgument('patrol_route', default_value='',
                              description='patrol route in zones.yaml (default: the robot name; '
                                          '"base" is the whole-base loop)'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='zones.yaml override (default: installed one)'),
        OpaqueFunction(function=_launch_robot),
    ])
