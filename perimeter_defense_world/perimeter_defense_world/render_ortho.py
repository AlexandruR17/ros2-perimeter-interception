"""Render a top-down image of the FOB-3 world for the C2 dashboard map.

A camera looking straight down from high altitude with a narrow field of view
is added to a copy of the generated world (near-orthographic: a 4 m building
leans by less than 0.3 m at the image edge). The image is fetched through
ros_gz_bridge and saved with a small YAML file giving its world bounds.

Usage (needs a sourced ROS 2 + Gazebo environment):
    ros2 run perimeter_defense_world render_ortho --variant day --out maps/fob3_ortho_day.png
"""

from __future__ import annotations

import argparse
import math
import os
import shutil
import signal
import subprocess
import tempfile
import time
from typing import Optional

import numpy as np
from PIL import Image as PilImage

from perimeter_defense_world.geometry import BaseLayout, load_layout, package_dir

ALTITUDE = 300.0  # keeps the field of view above the SDF minimum of 0.1 rad
PX_PER_M = 60.0


def camera_model(layout: BaseLayout) -> str:
    cx, cy = layout.width / 2.0, layout.height / 2.0
    hfov = 2.0 * math.atan((layout.width / 2.0) / ALTITUDE)
    w = int(layout.width * PX_PER_M)
    h = int(layout.height * PX_PER_M)
    # Pitch down 90 deg, then yaw 90 deg so that the image top is north.
    return f"""
    <model name="ortho_camera"><static>true</static>
      <pose>{cx} {cy} {ALTITUDE} 0 1.5707963 1.5707963</pose>
      <link name="link">
        <sensor name="ortho" type="camera">
          <topic>/ortho/image</topic>
          <update_rate>2</update_rate>
          <always_on>true</always_on>
          <camera>
            <horizontal_fov>{hfov:.6f}</horizontal_fov>
            <image><width>{w}</width><height>{h}</height><format>R8G8B8</format></image>
            <clip><near>{ALTITUDE - 20.0}</near><far>{ALTITUDE + 20.0}</far></clip>
          </camera>
        </sensor>
      </link>
    </model>"""


def render(world_file: str, layout: BaseLayout, out_png: str, models_dir: str,
           timeout: float = 90.0) -> None:
    with open(world_file, 'r', encoding='utf-8') as f:
        sdf = f.read()
    sdf = sdf.replace('  </world>', camera_model(layout) + '\n  </world>')
    tmp = tempfile.mkdtemp(prefix='fob3_ortho_')
    world_tmp = os.path.join(tmp, 'world.sdf')
    with open(world_tmp, 'w', encoding='utf-8') as f:
        f.write(sdf)

    env = dict(os.environ)
    env['GZ_SIM_RESOURCE_PATH'] = models_dir + os.pathsep + env.get('GZ_SIM_RESOURCE_PATH', '')
    gz = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', world_tmp],
                          env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                          start_new_session=True)
    bridge = subprocess.Popen(
        ['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
         '/ortho/image@sensor_msgs/msg/Image[gz.msgs.Image'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        time.sleep(3.0)
        if gz.poll() is not None:
            raise RuntimeError('gz sim exited, check the world file')
        img = _wait_for_image(timeout)
    finally:
        for proc in (bridge, gz):
            os.killpg(proc.pid, signal.SIGINT)
        time.sleep(2.0)
        for proc in (bridge, gz):
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
        shutil.rmtree(tmp, ignore_errors=True)

    os.makedirs(os.path.dirname(os.path.abspath(out_png)), exist_ok=True)
    PilImage.fromarray(img).save(out_png, optimize=True)
    with open(os.path.splitext(out_png)[0] + '.yaml', 'w', encoding='utf-8') as f:
        f.write(f'image: {os.path.basename(out_png)}\n'
                f'# world bounds covered by the image, metres (x east, y north)\n'
                f'min: [0.0, 0.0]\nmax: [{layout.width}, {layout.height}]\n'
                f'pixels_per_metre: {PX_PER_M}\n')


def _wait_for_image(timeout: float) -> np.ndarray:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = Node('render_ortho')
    frames = []
    node.create_subscription(Image, '/ortho/image', frames.append, qos_profile_sensor_data)
    t0 = time.time()
    try:
        # Skip the first frames, rendering needs a moment to load all textures.
        while len(frames) < 4 and time.time() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if not frames:
        raise RuntimeError('no image received from the top-down camera')
    msg = frames[-1]
    return np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, 3)


def main(argv: Optional[list] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--zones', default=None, help='path to zones.yaml')
    parser.add_argument('--variant', default='day', choices=['day', 'night'])
    parser.add_argument('--worlds', default=None, help='directory with fob3_<variant>.sdf')
    parser.add_argument('--models', default=None, help='models directory (GZ resource path)')
    parser.add_argument('--out', required=True, help='output PNG path')
    args = parser.parse_args(argv)

    here = package_dir()
    worlds = args.worlds or os.path.join(here, 'worlds')
    models = args.models or os.path.join(here, 'models')
    layout = load_layout(args.zones)
    render(os.path.join(worlds, f'fob3_{args.variant}.sdf'), layout, args.out, models)
    print(f'written: {args.out}')


if __name__ == '__main__':
    main()
