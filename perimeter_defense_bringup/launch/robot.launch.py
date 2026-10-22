"""Spawn one LUP UGV with its sensors bridge, odometry drift, AMCL and Nav2.

All nodes run in the robot namespace (/lup1, ...). TF frames are prefixed
with the namespace (lup1/base_link, lup1/odom) and published on the global
/tf, so every robot localises in the shared 'map' frame.

With composition:=true (default) robot_state_publisher, AMCL, the Nav2
servers and their lifecycle managers run as components of one container
process per robot instead of eight processes: three robots start faster
and need about a gigabyte less memory.
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
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

from perimeter_defense_world.geometry import load_layout

NAV2_NODES = ['controller_server', 'planner_server', 'behavior_server', 'bt_navigator', 'velocity_smoother']


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


# (package, plugin, executable, node name) of the navigation stack of one robot.
NAV_COMPONENTS = [
    ('nav2_amcl', 'nav2_amcl::AmclNode', 'amcl', 'amcl'),
    ('nav2_controller', 'nav2_controller::ControllerServer', 'controller_server', 'controller_server'),
    ('nav2_planner', 'nav2_planner::PlannerServer', 'planner_server', 'planner_server'),
    ('nav2_behaviors', 'behavior_server::BehaviorServer', 'behavior_server', 'behavior_server'),
    ('nav2_bt_navigator', 'nav2_bt_navigator::BtNavigator', 'bt_navigator', 'bt_navigator'),
    ('nav2_velocity_smoother', 'nav2_velocity_smoother::VelocitySmoother', 'velocity_smoother',
     'velocity_smoother'),
]
# The controller and the behaviors command cmd_vel_nav; the velocity smoother turns it into
# the cmd_vel the robot receives.
REMAPS = {
    'controller_server': [('cmd_vel', 'cmd_vel_nav')],
    'behavior_server': [('cmd_vel', 'cmd_vel_nav')],
    'velocity_smoother': [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
}


def _navigation(robot: str, urdf: str, nav2_params: str, amcl_pose: dict, sim_time: dict,
                composition: bool) -> list:
    """robot_state_publisher, AMCL, the Nav2 servers and their lifecycle managers."""
    rsp = {'robot_description': urdf, 'frame_prefix': f'{robot}/'}
    # A generous bond timeout: with three robots starting at once a server can take more
    # than the default 4 s to answer, and the manager would give up for good.
    managers = [
        ('lifecycle_manager_localization', {'autostart': True, 'node_names': ['amcl'], 'bond_timeout': 20.0}),
        ('lifecycle_manager_navigation', {'autostart': True, 'node_names': NAV2_NODES, 'bond_timeout': 20.0}),
    ]

    def params(name: str) -> list:
        return [nav2_params, amcl_pose] if name == 'amcl' else [nav2_params]

    if composition:
        nodes = [ComposableNode(package='robot_state_publisher', plugin='robot_state_publisher::RobotStatePublisher',
                                name='robot_state_publisher', namespace=robot, parameters=[rsp, sim_time])]
        nodes += [ComposableNode(package=pkg, plugin=plugin, name=name, namespace=robot, parameters=params(name),
                                 remappings=REMAPS.get(name, []))
                  for pkg, plugin, _, name in NAV_COMPONENTS]
        nodes += [ComposableNode(package='nav2_lifecycle_manager', plugin='nav2_lifecycle_manager::LifecycleManager',
                                 name=name, namespace=robot, parameters=[sim_time, p])
                  for name, p in managers]
        # Isolated container: one executor (thread) per component, as Nav2 recommends. The
        # parameter file is also given to the container process, so the nodes the servers
        # create inside (local_costmap, global_costmap) get their parameters too.
        return [ComposableNodeContainer(name='nav_container', namespace=robot, package='rclcpp_components',
                                        executable='component_container_isolated',
                                        parameters=[nav2_params, sim_time],
                                        composable_node_descriptions=nodes, output='screen')]

    out = [Node(package='robot_state_publisher', executable='robot_state_publisher', namespace=robot,
                parameters=[rsp, sim_time], output='screen')]
    out += [Node(package=pkg, executable=exe, name=name, namespace=robot, parameters=params(name),
                 remappings=REMAPS.get(name, []), output='screen')
            for pkg, _, exe, name in NAV_COMPONENTS]
    out += [Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name=name, namespace=robot,
                 parameters=[sim_time, p], output='screen')
            for name, p in managers]
    return out


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
    variant = LaunchConfiguration('variant').perform(context)
    mappings.update({'ns': robot, 'enable_camera': 'true' if camera else 'false', 'variant': variant})
    if variant == 'transport':
        mappings['max_linear_velocity'] = str(fleet['transport']['max_speed'])
    lidar_rate = LaunchConfiguration('lidar_rate').perform(context)
    if lidar_rate:
        mappings['lidar_rate'] = lidar_rate      # lighter simulation (each scan is a render)
    urdf = xacro.process_file(os.path.join(desc_pkg, 'urdf', 'lup_ugv.urdf.xacro'),
                              mappings=mappings).toxml()

    # Nav2 parameters with the robot name substituted, nested under the namespace
    # so the node names match (/lup1/amcl, /lup1/local_costmap/local_costmap, ...).
    with open(os.path.join(bringup_pkg, 'config', 'nav2_params.yaml'), 'r', encoding='utf-8') as f:
        params = yaml.safe_load(f.read().replace('ROBOT', robot))
    if variant == 'transport':
        # Slower vehicle: the controller and the velocity smoother are capped at its top speed.
        v = float(fleet['transport']['max_speed'])
        params['controller_server']['ros__parameters']['FollowPath']['desired_linear_vel'] = v
        params['velocity_smoother']['ros__parameters']['max_velocity'][0] = v
    nav2_params = _write_tmp(f'{robot}_nav2_', yaml.safe_dump({robot: params}))

    amcl_pose = {'initial_pose.x': x, 'initial_pose.y': y, 'initial_pose.yaw': yaw}
    sim_time = {'use_sim_time': True}
    composition = LaunchConfiguration('composition').perform(context).lower() == 'true'

    actions = [
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
    ]
    actions += _navigation(robot, urdf, nav2_params, amcl_pose, sim_time, composition)
    # Own position broadcast, so the other robots' lidars do not take it for an intruder.
    actions.append(Node(
        package='perimeter_defense', executable='robot_transponder', namespace=robot,
        parameters=[sim_time, {'robot': robot}], output='screen',
    ))
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
        DeclareLaunchArgument('lidar_rate', default_value='',
                              description="lidar scans per second ('' = lup_ugv.yaml, 10)"),
        DeclareLaunchArgument('variant', default_value='interceptor',
                              description='interceptor (LUP) or transport (TRN, slower, cargo box)'),
        DeclareLaunchArgument('composition', default_value='true',
                              description='run the navigation nodes as components of one process'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('start_patrol', default_value='true'),
        DeclareLaunchArgument('patrol_route', default_value='',
                              description='patrol route in zones.yaml (default: the robot name; '
                                          '"base" is the whole-base loop)'),
        DeclareLaunchArgument('zones_file', default_value='',
                              description='zones.yaml override (default: installed one)'),
        OpaqueFunction(function=_launch_robot),
    ])
