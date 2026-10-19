"""M2: one intruder through the north breach, fence sensor, lidar detector,
Kalman tracker, IFF, and the UGVs (LUP-2 by default, east sector) leaving
their patrol to investigate the alert and follow the track. The base is
fully robotic: the UGV that captures the intruder escorts him to the
detention building.

With soldiers:=true three soldiers walk around the base: they carry
transponders (recognised as FRIEND), radio spot reports when they see the
intruder, and escort a captured intruder to the detention building instead
of the UGV.

With discovery_server:=true a Fast DDS discovery server is started first and
every process of the launch uses it. Needed on machines where the default
multicast discovery does not find all the ~50 processes of three robots
(seen on WSL2: some nodes never discover each other, robots never localize).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from perimeter_defense_world.geometry import load_layout


def _launch(context, *args, **kwargs):
    g = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    bringup = get_package_share_directory('perimeter_defense_bringup')
    launch_dir = os.path.join(bringup, 'launch')
    viz_params = os.path.join(get_package_share_directory('perimeter_defense_viz'),
                              'config', 'viz.yaml')

    def include(name, args):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments=args.items())

    robots = [r for r in g('robots').split(',') if r]
    # A single UGV patrols the whole base; several UGVs keep their sectors.
    patrol_route = 'base' if len(robots) == 1 else ''
    soldiers = list(load_layout().soldiers) if g('soldiers').lower() == 'true' else []
    callsigns = [load_layout().soldiers[s].callsign for s in soldiers]
    actors = ','.join(['intruder1:north_breach'] + [f'{s}:soldier' for s in soldiers])

    actions = [
        include('world.launch.py', {'world_variant': g('world_variant'), 'headless': g('headless'),
                                    'quality': g('quality'), 'actors': actors}),
        TimerAction(period=3.0, actions=[
            *[include('robot.launch.py', {'robot': r, 'enable_camera': g('enable_camera'),
                                          'patrol_route': patrol_route})
              for r in robots],
            include('intruder.launch.py', {'name': 'intruder1', 'spawn': 'north_breach',
                                           'route': g('route'), 'start_delay': g('start_delay')}),
        ]),
        include('defense.launch.py', {'robots': ','.join(robots), 'mode': g('mode'),
                                      'escort_by': 'soldier' if soldiers else 'robot'}),
        Node(package='perimeter_defense_viz', executable='layout_markers', name='layout_markers',
             parameters=[viz_params, {'aerial_variant': g('world_variant'),
                                      'routes': patrol_route or ','.join(robots)}]),
        Node(package='perimeter_defense_viz', executable='fleet_markers', name='fleet_markers',
             parameters=[viz_params, {'use_sim_time': True, 'robots': robots,
                                      'callsigns': [r.replace('lup', 'LUP-') for r in robots]}]),
        Node(package='perimeter_defense_viz', executable='status_board', name='status_board',
             parameters=[viz_params, {'use_sim_time': True, 'soldiers': ','.join(callsigns)}]),
        # Exercise truth overlay (where the actors really are), for the operator only.
        Node(package='perimeter_defense_sim', executable='truth_markers', name='truth_markers',
             parameters=[{'use_sim_time': True, 'actors': ['intruder1'],
                          'friendly_actors': soldiers or [''], 'friendly_labels': callsigns or ['']}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', os.path.join(bringup, 'rviz', 'fob3.rviz')],
             parameters=[{'use_sim_time': True}],
             condition=IfCondition(g('rviz'))),
    ]
    if g('dashboard').lower() == 'true':
        actions.append(include('dashboard.launch.py', {'robots': ','.join(robots), 'routes': patrol_route}))
    if soldiers:
        actions.append(TimerAction(period=3.0, actions=[
            include('soldiers.launch.py', {'soldiers': ','.join(soldiers), 'intruders': 'intruder1'})]))
    if g('discovery_server').lower() == 'true':
        port = g('discovery_port')
        server = ExecuteProcess(cmd=['fastdds', 'discovery', '-i', '0', '-l', '127.0.0.1', '-p', port],
                                name='discovery_server', output='screen')
        # Without the server no node finds any other: stop at once with the reason.
        stop = RegisterEventHandler(OnProcessExit(target_action=server, on_exit=[
            LogInfo(msg=f'ERROR: the discovery server stopped (UDP port {port} already in use by '
                        'another discovery server?). Run: pkill -9 -f fast-discovery; pkill -9 -f fastdds; '
                        'pkill -9 -f "gz sim" '
                        'and start again, or choose another port with discovery_port:=11888'),
            Shutdown(reason='discovery server stopped')]))
        return [SetEnvironmentVariable('ROS_DISCOVERY_SERVER', f'127.0.0.1:{port}'),
                server, stop, TimerAction(period=1.0, actions=actions)]
    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument('world_variant', default_value='day'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('quality', default_value='high', description='high or low'),
        DeclareLaunchArgument('enable_camera', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('route', default_value='west', description='west or east'),
        DeclareLaunchArgument('robots', default_value='lup2',
                              description='comma separated, e.g. lup1,lup2,lup3'),
        DeclareLaunchArgument('soldiers', default_value='false',
                              description='own soldiers on foot (spot reports, escort); '
                                          'default: fully robotic base'),
        DeclareLaunchArgument('mode', default_value='predictive',
                              description='predictive (interception point) or naive (last position); '
                                          'switch at runtime from the dashboard or with ros2 param set'),
        DeclareLaunchArgument('dashboard', default_value='true',
                              description='C2 web dashboard on http://localhost:8080'),
        DeclareLaunchArgument('start_delay', default_value='15.0',
                              description='s before the intruder starts walking'),
        DeclareLaunchArgument('discovery_server', default_value='false',
                              description='start a Fast DDS discovery server and use it (WSL2, many robots)'),
        DeclareLaunchArgument('discovery_port', default_value='11811'),
        OpaqueFunction(function=_launch),
    ])
