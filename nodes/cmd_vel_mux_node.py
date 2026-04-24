# -*- coding: utf-8 -*-
"""
nodes/cmd_vel_mux_node.py -- Velocity command multiplexer.

Subscribes to multiple cmd_vel sources, each with an assigned
priority level, and forwards only the highest-priority active
source to the real /cmd_vel topic.

Priority (highest first):
    0  /cmd_vel_emergency     emergency stop (any node)
    1  /cmd_vel_recovery      recovery behaviours
    2  /cmd_vel_nav           normal navigation

A source is considered "active" if it has published within the
last TIMEOUT seconds.  If no source is active, zero velocity
is published (safe default).

Subscriptions
    /cmd_vel_emergency       geometry_msgs/Twist
    /cmd_vel_recovery        geometry_msgs/Twist
    /cmd_vel_nav             geometry_msgs/Twist

Publications
    /cmd_vel                 geometry_msgs/Twist
"""

import sys
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from config import CONTROL_HZ


# Timeout (seconds) -- if a source hasn't published within this
# window it is considered inactive and lower-priority sources may
# take over.
_SOURCE_TIMEOUT = 0.5


class _Source:
    """Tracks the latest Twist and timestamp for one input topic."""

    __slots__ = ("name", "priority", "twist", "last_stamp")

    def __init__(self, name: str, priority: int) -> None:
        self.name = name
        self.priority = priority          # lower number = higher priority
        self.twist = Twist()
        self.last_stamp: float = 0.0      # time.monotonic()

    def is_active(self, now: float) -> bool:
        return (now - self.last_stamp) < _SOURCE_TIMEOUT


class CmdVelMuxNode(Node):
    """
    Multiplexes multiple /cmd_vel_* inputs onto a single /cmd_vel
    output based on priority.
    """

    def __init__(self) -> None:
        super().__init__("cmd_vel_mux_node")
        self.get_logger().info("CmdVelMuxNode starting ...")

        import time
        self._time = time  # keep a reference for monotonic()

        # ── Define sources (priority 0 = highest) ──
        self._sources = [
            _Source("/cmd_vel_emergency", priority=0),
            _Source("/cmd_vel_recovery",  priority=1),
            _Source("/cmd_vel_nav",       priority=2),
        ]

        # ── Subscribers ──
        for src in self._sources:
            self.create_subscription(
                Twist,
                src.name,
                lambda msg, s=src: self._source_cb(msg, s),
                10,
            )
            self.get_logger().info(
                f"  Source registered: {src.name} (priority {src.priority})"
            )

        # ── Publisher ──
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # ── Output timer ──
        period = 1.0 / CONTROL_HZ
        self._timer = self.create_timer(period, self._tick)

        # Track which source is currently winning so we can log changes
        self._active_source_name: str = ""

        self.get_logger().info("CmdVelMuxNode ready.")

    # ──────────────────────────────────────────
    #  Callback -- store latest twist from each source
    # ──────────────────────────────────────────

    def _source_cb(self, msg: Twist, source: _Source) -> None:
        source.twist = msg
        source.last_stamp = self._time.monotonic()

    # ──────────────────────────────────────────
    #  Timer -- select highest-priority active source
    # ──────────────────────────────────────────

    def _tick(self) -> None:
        now = self._time.monotonic()

        # Walk sources in priority order (list is already sorted)
        for src in self._sources:
            if src.is_active(now):
                # Log priority changes
                if src.name != self._active_source_name:
                    self.get_logger().info(
                        f"cmd_vel mux: active source -> {src.name} "
                        f"(priority {src.priority})"
                    )
                    self._active_source_name = src.name

                self._pub.publish(src.twist)
                return

        # No active source -- publish zero velocity (safe default)
        if self._active_source_name != "":
            self.get_logger().info("cmd_vel mux: no active source -> STOP")
            self._active_source_name = ""
        self._pub.publish(Twist())


# ──────────────────────────────────────────────
#  Entry Point
# ──────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelMuxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
