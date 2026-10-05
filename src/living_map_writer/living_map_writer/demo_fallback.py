"""Deterministic recovery controller for the Phase-1 judge demonstration.

The normal launch uses frontier exploration and Nav2. This explicitly labelled
fallback uses odometry-guided waypoints so it can reveal an initially unknown
map instead of asking a global planner to plan through a map that does not yet
exist. LiDAR remains active and SLAM continues building the displayed map.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool, String


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class DemoFallback(Node):
    def __init__(self) -> None:
        super().__init__("writer_demo_fallback", namespace="/living_map")
        self.failed = False
        self.enabled = False
        self.pose: tuple[float, float, float] | None = None
        self.index = 0
        self.declare_parameter("max_linear_vel", 0.50)
        self.declare_parameter("max_angular_vel", 0.8)
        self.max_linear = float(self.get_parameter("max_linear_vel").value)
        self.max_angular = float(self.get_parameter("max_angular_vel").value)
        # Safe centre-line route checked against mine.sdf. It enters the gas
        # chamber through the intentional opening, then returns to V01 in the open entry corridor.
        self.waypoints = [
            (1.8, -1.8), (6.0, -1.8), (10.0, -1.8),
            (15.4, -1.8), (15.4, 0.0), (14.5, 3.1),
            (15.4, 0.0), (15.4, -1.8), (13.0, -2.0),
            (13.0, -3.8), (6.0, -2.2),
        ]
        # The route then returns to the entrance, mirroring the explorer's best case.
        self.return_from = len(self.waypoints)
        self.waypoints += [(1.8, -1.8), (0.0, 0.0)]
        self.stuck = False
        self.cmd_pub = self.create_publisher(Twist, "/writer/cmd_vel", 10)
        self.create_subscription(Odometry, "/writer/odom", self.on_odom, 20)
        state_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "/living_map/writer_failed", self.on_failed, state_qos)
        self.create_subscription(Bool, "/living_map/writer_enabled", self.on_enabled, state_qos)
        self.create_subscription(Bool, "/living_map/writer_stuck", self.on_stuck, state_qos)
        self.state_pub = self.create_publisher(String, "/living_map/writer_explorer_state", state_qos)
        self.published_state = ""
        self.create_timer(0.1, self.tick)
        self.get_logger().warning(
            "DEMO FALLBACK ACTIVE: deterministic odometry waypoints are revealing the unknown map"
        )

    def on_odom(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw)

    def on_enabled(self, msg: Bool) -> None:
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.cmd_pub.publish(Twist())

    def on_failed(self, msg: Bool) -> None:
        self.failed = bool(msg.data)
        if self.failed:
            self.cmd_pub.publish(Twist())
            self.get_logger().error("Writer failure: recovery controller stopped")

    def on_stuck(self, msg: Bool) -> None:
        self.stuck = bool(msg.data)
        if self.stuck:
            self.cmd_pub.publish(Twist())
            self.get_logger().error("Writer immobilised: recovery controller stopped")

    def _publish_state(self) -> None:
        if self.failed:
            state = "HALTED"
        elif self.stuck:
            state = "STUCK"
        elif self.index >= len(self.waypoints):
            state = "HOME"
        elif self.index >= self.return_from:
            state = "RETURNING"
        else:
            state = "NAVIGATING" if self.enabled else "PAUSED"
        if state != self.published_state:
            self.published_state = state
            self.state_pub.publish(String(data=state))

    def tick(self) -> None:
        self._publish_state()
        if (self.failed or self.stuck or not self.enabled or self.pose is None
                or self.index >= len(self.waypoints)):
            return

        x, y, yaw = self.pose
        goal_x, goal_y = self.waypoints[self.index]
        dx, dy = goal_x - x, goal_y - y
        distance = math.hypot(dx, dy)
        if distance < 0.35:
            self.index += 1
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                f"Recovery waypoint {self.index}/{len(self.waypoints)} reached at odom=({x:.1f}, {y:.1f})"
            )
            return

        heading_error = normalize_angle(math.atan2(dy, dx) - yaw)
        command = Twist()
        command.angular.z = max(-self.max_angular, min(self.max_angular, 1.8 * heading_error))
        if abs(heading_error) < 0.45:
            command.linear.x = min(self.max_linear, 0.30 * distance)
        self.cmd_pub.publish(command)


def main() -> None:
    rclpy.init()
    node = DemoFallback()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
