"""Small ROS helpers shared by the perimeter defense nodes."""

from __future__ import annotations

import math
from typing import Callable, Optional, Tuple

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException


def to_sec(stamp: TimeMsg) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def to_stamp(t: float) -> TimeMsg:
    sec = int(math.floor(t))
    return TimeMsg(sec=sec, nanosec=int((t - sec) * 1e9))


def yaw_of(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def lookup_pose2d(buffer: Buffer, target: str, source: str, stamp: Optional[TimeMsg] = None,
                  max_latest_age: float = 0.1) -> Optional[Tuple[float, float, float]]:
    """(x, y, yaw) of frame `source` in frame `target`, None if not available.

    With a stamp, the transform at that time is used; if it is not available
    yet (odometry slightly behind the sensor), the latest transform is used
    when it is at most max_latest_age older than the stamp.
    """
    try:
        t = buffer.lookup_transform(target, source, Time.from_msg(stamp) if stamp else Time())
    except TransformException:
        if stamp is None:
            return None
        try:
            t = buffer.lookup_transform(target, source, Time())
        except TransformException:
            return None
        if to_sec(stamp) - to_sec(t.header.stamp) > max_latest_age:
            return None
    tr = t.transform
    return tr.translation.x, tr.translation.y, yaw_of(tr.rotation)


def spin_node(factory: Callable[[], Node], args=None) -> None:
    """Standard main: init, spin, clean shutdown."""
    rclpy.init(args=args)
    node = factory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
