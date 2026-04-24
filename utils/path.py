"""
utils/path.py — Path post-processing: pruning, smoothing, collinear
compression, and conversion to a ROS ``nav_msgs/Path`` message.
"""

import math
from typing import List, Tuple

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as RosPath
from builtin_interfaces.msg import Time


# ──────────────────────────────────────────────
#  Pruning
# ──────────────────────────────────────────────

def prune_path(
    waypoints: List[Tuple[float, float]],
    min_distance: float = 0.03,
) -> List[Tuple[float, float]]:
    """
    Remove consecutive waypoints that are closer than *min_distance*.

    The first and last points are always kept.
    """
    if len(waypoints) <= 2:
        return list(waypoints)

    pruned = [waypoints[0]]
    for pt in waypoints[1:-1]:
        if math.hypot(pt[0] - pruned[-1][0], pt[1] - pruned[-1][1]) >= min_distance:
            pruned.append(pt)
    pruned.append(waypoints[-1])
    return pruned


# ──────────────────────────────────────────────
#  Gradient-Descent Smoothing
# ──────────────────────────────────────────────

def smooth_path(
    waypoints: List[Tuple[float, float]],
    weight_data: float = 0.1,
    weight_smooth: float = 0.3,
    tolerance: float = 1e-5,
    max_iterations: int = 1000,
) -> List[Tuple[float, float]]:
    """
    Apply gradient-descent path smoothing.

    The algorithm iteratively adjusts interior waypoints, balancing two
    objectives:

    * **data term** — keep points close to their original positions.
    * **smooth term** — minimise the second derivative along the path.

    The first and last waypoints are fixed (start / goal).

    Parameters
    ----------
    waypoints : ordered list of (x, y) tuples.
    weight_data : strength of the data term (α).
    weight_smooth : strength of the smooth term (β).
    tolerance : convergence threshold on total change.
    max_iterations : hard cap to prevent runaway loops.

    Returns
    -------
    Smoothed list of (x, y) tuples.
    """
    if len(waypoints) <= 2:
        return list(waypoints)

    # Work on a mutable copy
    path = [list(p) for p in waypoints]
    original = [list(p) for p in waypoints]

    for _ in range(max_iterations):
        total_change = 0.0
        for i in range(1, len(path) - 1):
            for dim in range(2):
                before = path[i][dim]
                path[i][dim] += weight_data * (original[i][dim] - path[i][dim])
                path[i][dim] += weight_smooth * (
                    path[i - 1][dim] + path[i + 1][dim] - 2.0 * path[i][dim]
                )
                total_change += abs(path[i][dim] - before)

        if total_change < tolerance:
            break

    return [tuple(p) for p in path]


# ──────────────────────────────────────────────
#  Collinear Compression
# ──────────────────────────────────────────────

def simplify_collinear(
    waypoints: List[Tuple[float, float]],
    angle_threshold: float = 0.15,
) -> List[Tuple[float, float]]:
    """
    Remove nearly-collinear interior points.

    If the heading change between three consecutive points is below
    *angle_threshold* (radians), the middle point is dropped.
    """
    if len(waypoints) <= 2:
        return list(waypoints)

    simplified = [waypoints[0]]
    for i in range(1, len(waypoints) - 1):
        a = waypoints[i - 1]
        b = waypoints[i]
        c = waypoints[i + 1]
        heading_ab = math.atan2(b[1] - a[1], b[0] - a[0])
        heading_bc = math.atan2(c[1] - b[1], c[0] - b[0])
        delta = abs(_normalize(heading_bc - heading_ab))
        if delta >= angle_threshold:
            simplified.append(b)
    simplified.append(waypoints[-1])
    return simplified


def _normalize(angle: float) -> float:
    """Wrap angle into [-π, π]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ──────────────────────────────────────────────
#  ROS Message Conversion
# ──────────────────────────────────────────────

def waypoints_to_path_msg(
    waypoints: List[Tuple[float, float]],
    frame_id: str = "map",
    stamp: Time | None = None,
) -> RosPath:
    """
    Convert a list of (x, y) tuples into a ``nav_msgs/Path`` message.

    Each waypoint becomes a ``PoseStamped`` with orientation pointing
    toward the next waypoint (last pose keeps the penultimate heading).
    """
    path_msg = RosPath()
    path_msg.header.frame_id = frame_id
    if stamp is not None:
        path_msg.header.stamp = stamp

    for i, (x, y) in enumerate(waypoints):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        if stamp is not None:
            pose.header.stamp = stamp
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Orientation: face toward the next waypoint
        if i < len(waypoints) - 1:
            nx, ny = waypoints[i + 1]
            yaw = math.atan2(ny - y, nx - x)
        else:
            # last point — keep previous heading
            if len(waypoints) >= 2:
                px, py = waypoints[-2]
                yaw = math.atan2(y - py, x - px)
            else:
                yaw = 0.0

        # Convert yaw → quaternion (only z-axis rotation)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)

        path_msg.poses.append(pose)

    return path_msg
