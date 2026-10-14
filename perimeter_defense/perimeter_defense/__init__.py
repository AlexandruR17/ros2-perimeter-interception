"""Perimeter defense: detection, IFF, tracking and prediction, interceptor allocation.

Individual contribution of the project. The algorithms in perimeter_defense.core
are plain Python (no ROS) so they can be unit tested; the *_node modules wrap
them as ROS 2 nodes. These nodes only use simulated sensor data (lidar,
fence sensor, transponders, the robots' own localization), never ground truth.
"""
