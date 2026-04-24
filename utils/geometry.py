"""
utils/geometry.py — Pure-math helpers for distances, angles, and
coordinate transforms.  Zero ROS dependencies.
"""

import math
from typing import Tuple


def euclidean_distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two 2-D points."""
    return math.hypot(x2 - x1, y2 - y1)


def normalize_angle(angle: float) -> float:
    """Wrap *angle* (radians) into the range [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def angle_between(x1: float, y1: float, x2: float, y2: float) -> float:
    """Heading angle (radians) from point 1 to point 2 using atan2."""
    return math.atan2(y2 - y1, x2 - x1)


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Extract the yaw component from a quaternion (x, y, z, w).

    Uses the standard conversion:
        yaw = atan2(2(wz + xy), 1 − 2(y² + z²))
    """
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_grid(
    wx: float,
    wy: float,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[int, int]:
    """
    Convert world coordinates (metres) to grid cell indices.

    Parameters
    ----------
    wx, wy : world position (metres).
    origin_x, origin_y : world position of grid cell (0, 0).
    resolution : metres per cell.

    Returns
    -------
    (gx, gy) : integer grid indices.
    """
    gx = int(round((wx - origin_x) / resolution))
    gy = int(round((wy - origin_y) / resolution))
    return gx, gy


def grid_to_world(
    gx: int,
    gy: int,
    origin_x: float,
    origin_y: float,
    resolution: float,
) -> Tuple[float, float]:
    """
    Convert grid cell indices back to world coordinates (cell centre).

    Returns
    -------
    (wx, wy) : world position in metres.
    """
    wx = origin_x + gx * resolution
    wy = origin_y + gy * resolution
    return wx, wy
