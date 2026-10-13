"""Check that animated actors are visible to gpu_lidar in the installed Gazebo.

Starts a small world with a walking actor (1.3 m/s, model://fob3_human) and a
360 degree gpu_lidar at the LUP UGV mast height, then checks that the scan
contains returns near the actor's scripted position. Prints PASS or FAIL.

Usage:
    ros2 run perimeter_defense_world check_actor_lidar
"""

from __future__ import annotations

import math
import os
import shutil
import signal
import subprocess
import tempfile
import time
from typing import List, Optional, Tuple

from perimeter_defense_world.geometry import package_dir

WALK_SPEED = 1.3
START_X = -4.0
LANE_Y = 4.0

WORLD = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <world name="actor_lidar_check">
    <physics name="p" type="ignored"><max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor></physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <light type="directional" name="sun"><direction>-0.3 0.2 -0.9</direction></light>
    <model name="ground"><static>true</static><link name="l">
      <collision name="c"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></collision>
      <visual name="v"><geometry><plane><normal>0 0 1</normal><size>60 60</size></plane></geometry></visual>
    </link></model>
    <model name="lidar"><static>true</static><pose>0 0 0.85 0 0 0</pose><link name="l">
      <sensor name="scan" type="gpu_lidar"><topic>/check/scan</topic><update_rate>10</update_rate>
        <always_on>1</always_on>
        <lidar><scan><horizontal><samples>720</samples><resolution>1</resolution>
          <min_angle>-3.14159</min_angle><max_angle>3.13286</max_angle></horizontal></scan>
          <range><min>0.25</min><max>30</max><resolution>0.01</resolution></range></lidar>
      </sensor></link></model>
    <actor name="walker">
      <skin><filename>model://fob3_human/meshes/walk.dae</filename></skin>
      <animation name="walking"><filename>model://fob3_human/meshes/walk.dae</filename>
        <interpolate_x>true</interpolate_x></animation>
      <script><loop>true</loop><auto_start>true</auto_start>
        <trajectory id="0" type="walking">
          <waypoint><time>0</time><pose>{START_X} {LANE_Y} 1.0 0 0 0</pose></waypoint>
          <waypoint><time>{8.0 / WALK_SPEED:.3f}</time><pose>{START_X + 8.0} {LANE_Y} 1.0 0 0 0</pose></waypoint>
        </trajectory></script>
    </actor>
  </world>
</sdf>
"""


def _scan_points(timeout: float) -> List[Tuple[float, List[Tuple[float, float]]]]:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node('check_actor_lidar')
    scans: List[LaserScan] = []
    node.create_subscription(LaserScan, '/check/scan', scans.append, qos_profile_sensor_data)
    t0 = time.time()
    try:
        while len(scans) < 40 and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    out = []
    for s in scans:
        stamp = s.header.stamp.sec + s.header.stamp.nanosec * 1e-9
        pts = []
        for i, r in enumerate(s.ranges):
            if s.range_min < r < s.range_max:
                a = s.angle_min + i * s.angle_increment
                pts.append((r * math.cos(a), r * math.sin(a)))
        out.append((stamp, pts))
    return out


def run(timeout: float = 60.0) -> bool:
    tmp = tempfile.mkdtemp(prefix='actor_lidar_')
    world = os.path.join(tmp, 'world.sdf')
    with open(world, 'w', encoding='utf-8') as f:
        f.write(WORLD)
    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = (os.path.join(package_dir(), 'models') + os.pathsep
                                   + env.get('GZ_SIM_RESOURCE_PATH', ''))
    procs = [
        subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', world], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True),
        subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                          '/check/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'], env=env,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                         start_new_session=True),
    ]
    try:
        scans = _scan_points(timeout)
    finally:
        for p in procs:
            os.killpg(p.pid, signal.SIGINT)
        time.sleep(1.5)
        for p in procs:
            if p.poll() is None:
                os.killpg(p.pid, signal.SIGKILL)
        shutil.rmtree(tmp, ignore_errors=True)

    if not scans:
        print('FAIL: no scan received (is gpu_lidar rendering available?)')
        return False
    period = 8.0 / WALK_SPEED
    hits = 0
    for stamp, pts in scans:
        expected_x = START_X + WALK_SPEED * (stamp % period)
        near = [p for p in pts if abs(p[1] - LANE_Y) < 0.6 and abs(p[0] - expected_x) < 1.0]
        hits += 1 if near else 0
    ratio = hits / len(scans)
    print(f'actor seen in {hits}/{len(scans)} scans ({ratio:.0%})')
    ok = ratio > 0.8
    print('PASS: actors are visible to gpu_lidar' if ok else
          'FAIL: actors are not reliably visible to gpu_lidar')
    return ok


def main(argv: Optional[list] = None) -> None:
    raise SystemExit(0 if run() else 1)


if __name__ == '__main__':
    main()
