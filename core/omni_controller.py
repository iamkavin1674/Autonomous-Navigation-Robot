"""
core/omni_controller.py — Regulated Pure Pursuit controller adapted
for omnidirectional (holonomic) drive.

Computes ``geometry_msgs/Twist`` commands with ``linear.x`` (forward),
``linear.y`` (lateral), and ``angular.z`` (yaw correction).
"""

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import (
    MAX_LINEAR_SPEED,
    MAX_ANGULAR_SPEED,
    LOOKAHEAD_DISTANCE,
    GOAL_TOLERANCE,
    HEADING_KP,
    APPROACH_VELOCITY_SCALING,
    REGULATE_MIN_SPEED,
)
from utils.geometry import euclidean_distance, normalize_angle


class OmniController:
    """Regulated Pure Pursuit — holonomic variant."""

    def __init__(
        self,
        lookahead: float = LOOKAHEAD_DISTANCE,
        goal_tol: float = GOAL_TOLERANCE,
        max_lin: float = MAX_LINEAR_SPEED,
        max_ang: float = MAX_ANGULAR_SPEED,
        heading_kp: float = HEADING_KP,
        approach_scale: float = APPROACH_VELOCITY_SCALING,
        min_speed: float = REGULATE_MIN_SPEED,
    ):
        self.lookahead = lookahead
        self.goal_tol = goal_tol
        self.max_lin = max_lin
        self.max_ang = max_ang
        self.heading_kp = heading_kp
        self.approach_scale = approach_scale
        self.min_speed = min_speed

    # ──────────────────────────────────────────
    #  Lookahead Point Selection
    # ──────────────────────────────────────────

    def find_lookahead_point(
        self,
        robot_x: float,
        robot_y: float,
        path: List[Tuple[float, float]],
    ) -> Tuple[Optional[Tuple[float, float]], int]:
        """
        Walk along *path* and return the first point whose distance
        from the robot exceeds the lookahead radius.

        Returns
        -------
        (target_point, path_index) — or (None, -1) if no valid point.
        """
        if not path:
            return None, -1

        # Find the closest point on the path to the robot
        min_dist = float("inf")
        closest_idx = 0
        for i, (px, py) in enumerate(path):
            d = euclidean_distance(robot_x, robot_y, px, py)
            if d < min_dist:
                min_dist = d
                closest_idx = i

        # Walk forward from the closest point to find the lookahead
        for i in range(closest_idx, len(path)):
            px, py = path[i]
            if euclidean_distance(robot_x, robot_y, px, py) >= self.lookahead:
                return (px, py), i

        # If no point is far enough, return the last point on the path
        return path[-1], len(path) - 1

    # ──────────────────────────────────────────
    #  Velocity Computation
    # ──────────────────────────────────────────

    def compute_velocity(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        target_x: float,
        target_y: float,
        goal_x: float,
        goal_y: float,
        proximity_factor: float = 1.0,
    ) -> Twist:
        """
        Compute the holonomic Twist to move toward the target point.

        Parameters
        ----------
        robot_x, robot_y, robot_yaw : current robot pose in the map frame.
        target_x, target_y : lookahead point on the path.
        goal_x, goal_y : final goal position (for approach scaling).
        proximity_factor : 0.0–1.0 speed multiplier from sensor fusion
            (1.0 = clear, 0.0 = obstacle at stop distance).

        Returns
        -------
        geometry_msgs/Twist with vx, vy, wz.
        """
        twist = Twist()

        # ── Vector from robot to target in the WORLD frame ──
        dx_world = target_x - robot_x
        dy_world = target_y - robot_y
        dist_to_target = math.hypot(dx_world, dy_world)

        if dist_to_target < 1e-6:
            return twist  # zero twist

        # ── Rotate into the ROBOT frame ──
        cos_yaw = math.cos(-robot_yaw)
        sin_yaw = math.sin(-robot_yaw)
        dx_robot = cos_yaw * dx_world - sin_yaw * dy_world
        dy_robot = sin_yaw * dx_world + cos_yaw * dy_world

        # ── Normalise direction and scale to max speed ──
        direction_mag = math.hypot(dx_robot, dy_robot)
        vx = (dx_robot / direction_mag) * self.max_lin
        vy = (dy_robot / direction_mag) * self.max_lin

        # ── Heading error → angular.z ──
        desired_yaw = math.atan2(dy_world, dx_world)
        yaw_error = normalize_angle(desired_yaw - robot_yaw)
        wz = self.heading_kp * yaw_error
        wz = max(-self.max_ang, min(self.max_ang, wz))

        # ── Approach velocity scaling (slow down near goal) ──
        dist_to_goal = euclidean_distance(robot_x, robot_y, goal_x, goal_y)
        if dist_to_goal < self.approach_scale:
            scale = max(dist_to_goal / self.approach_scale, 0.0)
            # Keep a minimum speed unless we are truly at the goal
            if dist_to_goal > self.goal_tol:
                scale = max(scale, self.min_speed / self.max_lin)
            else:
                scale = 0.0
            vx *= scale
            vy *= scale
            wz *= scale

        # ── Obstacle proximity regulation ──
        vx *= proximity_factor
        vy *= proximity_factor
        # Don't reduce angular speed as aggressively — the robot still
        # needs to turn to avoid obstacles.
        wz *= max(proximity_factor, 0.3)

        # ── Clamp final values ──
        speed = math.hypot(vx, vy)
        if speed > self.max_lin:
            ratio = self.max_lin / speed
            vx *= ratio
            vy *= ratio
        wz = max(-self.max_ang, min(self.max_ang, wz))

        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        return twist

    # ──────────────────────────────────────────
    #  Goal Check
    # ──────────────────────────────────────────

    def is_goal_reached(
        self,
        robot_x: float,
        robot_y: float,
        goal_x: float,
        goal_y: float,
    ) -> bool:
        """Return True when the robot is within goal tolerance."""
        return euclidean_distance(robot_x, robot_y, goal_x, goal_y) < self.goal_tol

    # ──────────────────────────────────────────
    #  Static helper: stop command
    # ──────────────────────────────────────────

    @staticmethod
    def stop() -> Twist:
        """Return a zero-velocity Twist."""
        return Twist()
